#include <Arduino.h>
#include <athenasObd2.h>
#include <esp32_can.h>
#include <can_common.h>
#include "Hardware/hardware.h"
#include "taskManager/Tasks/obd2Task.hpp"
#include "taskManager/taskManager.h"
#include "OBD2_KLine.h"
#include "Peripheral/ServoControl/servoControl.hpp"  // Controle via PCA/MUX

// =============================================================
// ===================== CONFIGURAÇÕES GERAIS ==================
// =============================================================

#define OBD2_TASK_DEBUG_ENABLE
#ifdef OBD2_TASK_DEBUG_ENABLE
  #define OBD2_TASK_DEBUG_PRINT(...)   { Serial.print(__VA_ARGS__); }
  #define OBD2_TASK_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
  #define OBD2_TASK_DEBUG_PRINTF(...)  { Serial.printf(__VA_ARGS__); }
#else
  #define OBD2_TASK_DEBUG_PRINT(...)
  #define OBD2_TASK_DEBUG_PRINTLN(...)
  #define OBD2_TASK_DEBUG_PRINTF(...)
#endif

// =============================================================
// ===================== CONFIG CAN / K-LINE ====================
// =============================================================

// Pinos de comunicação (K-Line compartilha pinos com CAN via MUX)
#define KLINE_RX_PIN           PIN_PCI_ATHENAS_CAN_RX   // RX ← L9637D TX
#define KLINE_TX_PIN           PIN_PCI_ATHENAS_CAN_TX   // TX → L9637D RX
#define KLINE_BAUDRATE         10400                    // Baud ISO9141/KWP2000

// Canal físico no PCA que controla o MUX
#define MUX_CHANNEL            3

// Estados lógicos do MUX
#define MUX_CAN_ACTIVE         0   // LOW → habilita transceptor CAN (SN65HVD230)
#define MUX_KLINE_ACTIVE       1   // HIGH → habilita transceptor K-Line (L9637D)

// Intervalos entre tentativas de reconexão
#define CAN_RETRY_INTERVAL_MS   5000
#define KLINE_RETRY_INTERVAL_MS 5000

// =============================================================
// ===================== INSTÂNCIAS =============================
// =============================================================

static OBD2_KLine KLine(Serial1, KLINE_BAUDRATE, KLINE_RX_PIN, KLINE_TX_PIN);

// =============================================================
// ===================== ENUM DE ESTADOS ========================
// =============================================================

typedef enum {
    OBD_MODE_TRY_CAN = 0,   // Tentando conectar via CAN
    OBD_MODE_CAN_ACTIVE,    // Comunicação ativa via CAN
    OBD_MODE_TRY_KLINE,     // Tentando conectar via K-Line
    OBD_MODE_KLINE_ACTIVE   // Comunicação ativa via K-Line
} ObdProtocolMode;

static ObdProtocolMode obdMode = OBD_MODE_TRY_CAN;

// =============================================================
// ===================== FUNÇÕES AUXILIARES ====================
// =============================================================

/**
 * @brief Controla o pino físico do MUX via PCA9685.
 * 
 * HIGH (1) → ativa K-Line
 * LOW  (0) → ativa CAN
 */
static void setMuxState(SystemStatus* ss, bool useKLine)
{
    DbServoChannel* muxCh = &ss->machine.servo.leftFront;  // canal físico no PCA
    muxCh->mode = SERVO_MODE_GPIO;     // força modo GPIO digital
    muxCh->pcaChannel = MUX_CHANNEL;   // CH4
    muxCh->currentValue = useKLine ? MUX_KLINE_ACTIVE : MUX_CAN_ACTIVE;
    muxCh->updated = true;

    OBD2_TASK_DEBUG_PRINTF("[MUX] Estado → %s (CH%d = %d)\n",
        useKLine ? "K-LINE" : "CAN",
        MUX_CHANNEL,
        muxCh->currentValue);
}

/**
 * @brief Zera todos os valores de PIDs monitorados.
 */
static void resetValuePIDs(SystemStatus *s)
{
    s->automotiveSystem.canMonitor.engineRPM                 = 0;
    s->automotiveSystem.canMonitor.vehicleSpeed              = 0;
    s->automotiveSystem.canMonitor.fuelLevel                 = 0;
    s->automotiveSystem.canMonitor.engineCoolantTemperature  = 0;
    s->automotiveSystem.canMonitor.intakeAirTemperature      = 0;
    s->automotiveSystem.canMonitor.intakeManifoldPressure    = 0;
    s->automotiveSystem.canMonitor.throttlePosition          = 0;
    s->automotiveSystem.canMonitor.ignitionTimingAdvance     = 0;
    s->automotiveSystem.canMonitor.calculatedLoadValue       = 0;
    s->automotiveSystem.canMonitor.massAirFlowRate           = 0;
}

// =============================================================
// ===================== TASK PRINCIPAL ========================
// =============================================================
void obd2Task_run(void *pvParameters)
{
    SystemStatus *systemStatus = (SystemStatus *)pvParameters;

    uint32_t lastSwitchAttempt = 0;
    uint32_t lastRequest = 0;
    uint32_t lastLog = 0;
    bool connected = false;

    OBD2_TASK_DEBUG_PRINTLN("\n=======================================");
    OBD2_TASK_DEBUG_PRINTLN("   OBD2 AUTO-SYNC TASK (CAN <-> KLINE)");
    OBD2_TASK_DEBUG_PRINTLN("=======================================\n");

    vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda inicialização completa

    // ----------------------------------------------------------
    // Configuração inicial do driver CAN
    // ----------------------------------------------------------
    CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX, (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);
    OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);
    OBD2.setHeartbeatPid(ENGINE_RPM);
    OBD2.setHeartbeatInterval(1000);
    OBD2.setBackoff(1000, 8000);
    OBD2.setReconnectInterval(20000);

    // ----------------------------------------------------------
    // Configuração inicial do driver K-Line
    // ----------------------------------------------------------
    KLine.setDebug(Serial);
    KLine.setProtocol("Automatic");
    KLine.setByteWriteInterval(5);
    KLine.setInterByteTimeout(60);
    KLine.setReadTimeout(1000);

    resetValuePIDs(systemStatus);

    // ==========================================================
    // LOOP PRINCIPAL (FSM - Máquina de Estados)
    // ==========================================================
    for (;;)
    {
        switch (obdMode)
        {
            // ==========================================================
            // 1️⃣ Tentativa de inicializar CAN
            // ==========================================================
            case OBD_MODE_TRY_CAN:
                OBD2_TASK_DEBUG_PRINTLN("[FSM] Tentando comunicação via CAN...");

                // Desliga K-Line e comuta MUX para modo CAN
                KLine.powerDownBus();
                setMuxState(systemStatus, false);      // Força MUX em modo CAN
                OBD2_TASK_DEBUG_PRINTLN("[MUX] 🕒 Aguardando estabilização (500 ms)...");
                vTaskDelay(pdMS_TO_TICKS(500));        // Aguarda troca física

                // Inicializa o driver CAN após a comutação
                OBD2.beginAuto();
                vTaskDelay(pdMS_TO_TICKS(300));        // Aguarda driver estabilizar

                if (OBD2.connected())
                {
                    OBD2_TASK_DEBUG_PRINTLN("✅ Comunicação CAN estabelecida!");
                    obdMode = OBD_MODE_CAN_ACTIVE;
                    connected = true;
                    break;
                }

                OBD2_TASK_DEBUG_PRINTLN("❌ Falha no CAN. Tentando K-Line...");
                OBD2.end();  // encerra driver CAN antes de trocar
                connected = false;
                obdMode = OBD_MODE_TRY_KLINE;
                lastSwitchAttempt = millis();
                break;

            // ==========================================================
            // 2️⃣ Comunicação CAN ativa
            // ==========================================================
            case OBD_MODE_CAN_ACTIVE:
                OBD2.tick();

                if (!OBD2.connected())
                {
                    OBD2_TASK_DEBUG_PRINTLN("⚠️ CAN desconectado. Alternando para K-Line...");
                    OBD2.end();
                    connected = false;
                    obdMode = OBD_MODE_TRY_KLINE;
                    lastSwitchAttempt = millis();
                    break;
                }

                // Leitura periódica de RPM via CAN
                if (millis() - lastRequest >= 500)
                {
                    lastRequest = millis();
                    float rpm = 0.0f;
                    if (OBD2.readPid(ENGINE_RPM, rpm))
                    {
                        systemStatus->automotiveSystem.canMonitor.engineRPM = rpm;
                        OBD2_TASK_DEBUG_PRINTF("🔥 [CAN] RPM: %.0f\n", rpm);
                    }
                }
                break;

            // ==========================================================
            // 3️⃣ Tentativa de inicializar K-Line
            // ==========================================================
            case OBD_MODE_TRY_KLINE:
                if (millis() - lastSwitchAttempt < KLINE_RETRY_INTERVAL_MS)
                    break;  // evita flood de tentativas

                OBD2_TASK_DEBUG_PRINTLN("[FSM] Tentando comunicação via K-Line...");

                // Desliga CAN e comuta MUX para modo K-Line
                OBD2.end();
                setMuxState(systemStatus, true);        // Força MUX em modo K-Line
                OBD2_TASK_DEBUG_PRINTLN("[MUX] 🕒 Aguardando estabilização (500 ms)...");
                vTaskDelay(pdMS_TO_TICKS(500));         // Aguarda troca física

                // Inicializa driver UART/K-Line após estabilização
                KLine.setSerial(true);
                connected = KLine.initOBD2();

                if (connected)
                {
                    OBD2_TASK_DEBUG_PRINTLN("✅ Comunicação K-Line estabelecida!");
                    obdMode = OBD_MODE_KLINE_ACTIVE;
                    break;
                }

                OBD2_TASK_DEBUG_PRINTLN("❌ Falha na K-Line. Voltando ao CAN...");
                KLine.powerDownBus();
                obdMode = OBD_MODE_TRY_CAN;
                lastSwitchAttempt = millis();
                break;

            // ==========================================================
            // 4️⃣ Comunicação K-Line ativa
            // ==========================================================
            case OBD_MODE_KLINE_ACTIVE:
                if (!connected)
                {
                    obdMode = OBD_MODE_TRY_CAN;
                    break;
                }

                // Leitura de RPM a cada 2 segundos
                if (millis() - lastRequest >= 2000)
                {
                    lastRequest = millis();

                    float rpm = KLine.getPID(read_LiveData, 0x0C);
                    if (rpm > 0)
                    {
                        systemStatus->automotiveSystem.canMonitor.engineRPM = rpm;
                        OBD2_TASK_DEBUG_PRINTF("🔥 [KLINE] RPM: %.0f\n", rpm);
                    }
                    else
                    {
                        OBD2_TASK_DEBUG_PRINTLN("❌ Falha ao ler PID. Voltando ao modo CAN...");
                        KLine.powerDownBus();
                        connected = false;
                        obdMode = OBD_MODE_TRY_CAN;
                        lastSwitchAttempt = millis();
                    }
                }
                break;
        }

        // ==========================================================
        // Log de status periódico (3 s)
        // ==========================================================
        if (millis() - lastLog >= 3000)
        {
            lastLog = millis();
            OBD2_TASK_DEBUG_PRINTF("[STATUS] Modo: %d | CAN: %d | KLine: %d | RPM: %.0f\n",
                                   obdMode,
                                   (OBD2.connected() ? 1 : 0),
                                   (connected ? 1 : 0),
                                   systemStatus->automotiveSystem.canMonitor.engineRPM);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
