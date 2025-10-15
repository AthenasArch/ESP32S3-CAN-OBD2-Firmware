#include "Peripheral/ServoControl/servoControl.hpp"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Hardware/hardware.h"

// =============================================================
// ===================== CONFIGURAÇÕES ==========================
// =============================================================

// Endereço padrão do PCA9685
#define PCA9685_ADDR     0x40

// Frequência de operação típica de servos (50 Hz)
#define SERVO_FREQ       50

// Faixa de pulsos ajustada para 50 Hz com clock de 27 MHz (~1.0–2.0 ms)
#define SERVO_MIN_PULSE  205   // Posição mínima (0°)
#define SERVO_MAX_PULSE  410   // Posição máxima (180°)

// Intervalo de rechecagem do PCA em caso de falha (ms)
#define PCA_RETRY_INTERVAL_MS  1000

// Quantidade de canais disponíveis (máx. 16 no PCA9685)
#define SERVO_CHANNEL_COUNT    4   // pode ser aumentado futuramente

// =============================================================
// ===================== DEBUG OPCIONAL =========================
// =============================================================
#define SERVO_CONTROL_DEBUG_ENABLE
#ifdef SERVO_CONTROL_DEBUG_ENABLE
  #define SERVO_CONTROL_DEBUG_PRINT(...)   { Serial.print(__VA_ARGS__); }
  #define SERVO_CONTROL_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
  #define SERVO_CONTROL_DEBUG_PRINTF(...)  { Serial.printf(__VA_ARGS__); }
#else
  #define SERVO_CONTROL_DEBUG_PRINT(...)   {}
  #define SERVO_CONTROL_DEBUG_PRINTLN(...) {}
  #define SERVO_CONTROL_DEBUG_PRINTF(...)  {}
#endif

// =============================================================
// ===================== OBJETO PCA9685 =========================
// =============================================================
static Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);

// =============================================================
// ===================== ESTADO INTERNO =========================
// =============================================================
static bool initialized = false;
static bool pcaDetected = false;
static uint32_t lastCheckMs = 0;

// =============================================================
// ===================== FUNÇÕES AUXILIARES =====================
// =============================================================

// Converte porcentagem (0–100%) para duty cycle (0–4095)
static uint16_t mapPercentToPwm(uint8_t percent)
{
    percent = constrain(percent, 0, 100);
    return (uint16_t)((percent / 100.0f) * 4095);
}

// Converte ângulo (0–180°) para pulso (205–410)
static uint16_t mapAngleToPulse(uint8_t angle)
{
    angle = constrain(angle, 0, 180);
    return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Verifica se o PCA responde no barramento I2C
static bool checkPcaConnection(uint8_t address)
{
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}

// =============================================================
// ===================== INICIALIZAÇÃO ==========================
// =============================================================
void servoControl_ini(SystemStatus* systemStatus)
{
    if (initialized)
        return;

    SERVO_CONTROL_DEBUG_PRINTLN("\n[ServoControl] Inicializando...");

    Wire.begin(IIC_SDA, IIC_SCL);
    Wire.setClock(400000);

    if (!checkPcaConnection(PCA9685_ADDR))
    {
        SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] ❌ PCA9685 não encontrado (0x%02X)\n", PCA9685_ADDR);
        initialized = false;
        pcaDetected = false;
        return;
    }

    pca.begin();
    pca.setOscillatorFrequency(27000000);
    pca.setPWMFreq(SERVO_FREQ);

    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1) == 1)
    {
        uint8_t mode1 = Wire.read();
        SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] ✅ PCA9685 detectado (MODE1=0x%02X)\n", mode1);
        pcaDetected = true;
    }
    else
    {
        SERVO_CONTROL_DEBUG_PRINTLN("[ServoControl] ⚠️ Falha ao ler MODE1");
        pcaDetected = false;
    }

    // === Configuração de canais ===
    DbServoControl* servo = &systemStatus->machine.servo;
    DbServoChannel* channels[SERVO_CHANNEL_COUNT] = {
        &servo->leftWhell,
        &servo->rightWhell,
        &servo->leftFront,
        &servo->rightFront
    };

    for (int i = 0; i < SERVO_CHANNEL_COUNT; i++)
    {
        DbServoChannel* ch = channels[i];

        if (ch->mode == SERVO_MODE_DISABLED)
        {
            ch->mode = SERVO_MODE_SERVO_ANGLE;
            ch->pcaChannel = i;
            ch->currentValue = 90;
            ch->previousValue = 255;
            ch->updated = true;
        }

        // Inicializa GPIO se o modo for digital
        if (ch->mode == SERVO_MODE_GPIO)
        {
            pinMode(ch->gpioPin, OUTPUT);
            digitalWrite(ch->gpioPin, LOW);
            SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] GPIO %d inicializado como saída.\n", ch->gpioPin);
        }

        SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] Canal %d pronto (modo=%d)\n", i, ch->mode);
    }

    initialized = true;
    SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] 🟢 Inicialização concluída (freq=%dHz)\n", SERVO_FREQ);
}

// =============================================================
// ===================== LOOP DE EXECUÇÃO =======================
// =============================================================
void servoControl_run(SystemStatus* systemStatus)
{
    // Rechecagem de comunicação I2C em caso de falha
    if (!pcaDetected && (millis() - lastCheckMs > PCA_RETRY_INTERVAL_MS))
    {
        lastCheckMs = millis();
        if (checkPcaConnection(PCA9685_ADDR))
        {
            SERVO_CONTROL_DEBUG_PRINTLN("[ServoControl] 🔄 PCA9685 reconectado!");
            pca.begin();
            pca.setOscillatorFrequency(27000000);
            pca.setPWMFreq(SERVO_FREQ);
            pcaDetected = true;
        }
    }

    if (!initialized)
        return;

    DbServoControl* servo = &systemStatus->machine.servo;
    DbServoChannel* channels[SERVO_CHANNEL_COUNT] = {
        &servo->leftWhell,
        &servo->rightWhell,
        &servo->leftFront,
        &servo->rightFront
    };

    for (int i = 0; i < SERVO_CHANNEL_COUNT; i++)
    {
        DbServoChannel* ch = channels[i];

        if (ch->mode == SERVO_MODE_DISABLED)
            continue;

        // Atualiza apenas se mudou ou sinalizado
        if ((ch->currentValue == ch->previousValue) && !ch->updated)
            continue;

        switch (ch->mode)
        {
            case SERVO_MODE_GPIO:
                if (!pcaDetected) break;

                if (ch->currentValue > 0)
                {
                    // Força saída HIGH no canal do PCA
                    pca.setPWM(ch->pcaChannel, 4096, 0);
                    SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] CH%d -> HIGH (GPIO MODE)\n", ch->pcaChannel);
                }
                else
                {
                    // Força saída LOW no canal do PCA
                    pca.setPWM(ch->pcaChannel, 0, 0);
                    SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] CH%d -> LOW (GPIO MODE)\n", ch->pcaChannel);
                }
                break;

            case SERVO_MODE_PWM_PERCENT:
            {
                if (!pcaDetected) break;
                uint16_t pwm = mapPercentToPwm(ch->currentValue);
                pca.setPWM(ch->pcaChannel, 0, pwm);
                SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] CH%d PWM = %u (%d%%)\n",
                                        ch->pcaChannel, pwm, ch->currentValue);
                break;
            }

            case SERVO_MODE_SERVO_ANGLE:
            {
                if (!pcaDetected) break;
                uint16_t pulse = mapAngleToPulse(ch->currentValue);
                pca.setPWM(ch->pcaChannel, 0, pulse);
                SERVO_CONTROL_DEBUG_PRINTF("[ServoControl] CH%d ANGLE = %d° -> pulse %u\n",
                                        ch->pcaChannel, ch->currentValue, pulse);
                break;
            }
        }


        ch->previousValue = ch->currentValue;
        ch->updated = false;
    }
}
