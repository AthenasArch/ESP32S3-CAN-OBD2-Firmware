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
// =============== CONFIGURAÇÕES / CONSTANTES ==================
// =============================================================

// Debug local do task
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

// ---- Utilidades legíveis (evita 1/0 espalhado) ----
static inline uint8_t BOOL_TO_U8(bool b){ return b ? 1U : 0U; }

// ---- Pinos compartilhados (via MUX PCA) ----
#define KLINE_RX_PIN                 PIN_PCI_ATHENAS_CAN_RX   // RX ← L9637D TX
#define KLINE_TX_PIN                 PIN_PCI_ATHENAS_CAN_TX   // TX → L9637D RX
#define KLINE_BAUDRATE               10400U                   // ISO9141/KWP2000

// ---- Canal do PCA que comanda o MUX (seu hardware) ----
#define MUX_CHANNEL                  3

// ---- Estados lógicos do MUX ----
#define MUX_CAN_ACTIVE               0U   // LOW  → habilita CAN
#define MUX_KLINE_ACTIVE             1U   // HIGH → habilita K-Line

// ---- Intervalos de comutação/espera ----
#define MUX_SETTLE_MS                500U   // tempo para o MUX “assentar”
#define CAN_DRIVER_STABILIZE_MS      300U

// ---- Intervalos de retry de conexão ----
#define CAN_RETRY_INTERVAL_MS        5000U
#define KLINE_RETRY_INTERVAL_MS      5000U

// ---- Coleta via CAN / K-Line (dados) ----
#define CAN_POLL_INTERVAL_MS         500U     // exemplo: RPM via CAN
// K-Line: foco em **coleta mais rápida**
#define KLINE_POLL_INTERVAL_1PID_MS  120U     // ~120 ms para 1 PID (ajuste fino ok)
#define KLINE_POLL_INTERVAL_2PID_MS  200U     // ~200 ms para 2 PIDs (se alternar)

// ---- Timeouts K-Line (mais agressivos) ----
#define KLINE_INTERBYTE_TIMEOUT_MS   20U
#define KLINE_READ_TIMEOUT_MS        250U

// ---- PIDs que usamos (mantém nomes sem “número mágico”) ----
#define PID_ENGINE_RPM               ENGINE_RPM    // (0x0C) já vem do athenasObd2.h
#define PID_VEHICLE_SPEED            VEHICLE_SPEED // (0x0D)

// Acima, defina:
#define KLINE_MAX_FAILS 3

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
 * @brief Controla a linha do MUX via PCA9685.
 * HIGH (1) → ativa K-Line
 * LOW  (0) → ativa CAN
 */
static void setMuxState(SystemStatus* ss, bool useKLine)
{
    DbServoChannel* muxCh = &ss->machine.servo.leftFront; // use o canal reservado ao MUX
    muxCh->mode         = SERVO_MODE_GPIO;
    muxCh->pcaChannel   = MUX_CHANNEL;
    muxCh->currentValue = useKLine ? MUX_KLINE_ACTIVE : MUX_CAN_ACTIVE;
    muxCh->updated      = true;

    OBD2_TASK_DEBUG_PRINTF("[MUX] Estado → %s (CH%d = %u)\n",
        useKLine ? "K-LINE" : "CAN",
        MUX_CHANNEL,
        (unsigned)muxCh->currentValue);
}

/** Zera os valores monitorados (visual limpo quando desconecta). */
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

    static int klineFails = 0;
    uint32_t lastSwitchAttempt = 0;
    uint32_t lastRequest       = 0;
    uint32_t lastLog           = 0;
    bool     klineConnected    = false;  // estado interno para K-Line

    OBD2_TASK_DEBUG_PRINTLN("\n=======================================");
    OBD2_TASK_DEBUG_PRINTLN("   OBD2 AUTO-SYNC TASK (CAN <-> KLINE)");
    OBD2_TASK_DEBUG_PRINTLN("=======================================\n");

    vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda inicialização completa do sistema

    // ---- CAN (TWAI) ----
    CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX, (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);
    OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);
    OBD2.setHeartbeatPid(PID_ENGINE_RPM);
    OBD2.setHeartbeatInterval(1000);
    OBD2.setBackoff(1000, 8000);
    OBD2.setReconnectInterval(20000);

    // ---- K-Line (logs internos DESLIGADOS: não chamamos setDebug) ----
    KLine.setProtocol("Automatic");
    KLine.setByteWriteInterval(5);
    KLine.setInterByteTimeout(KLINE_INTERBYTE_TIMEOUT_MS);
    KLine.setReadTimeout(KLINE_READ_TIMEOUT_MS);

    resetValuePIDs(systemStatus);

    for (;;)
    {
        switch (obdMode)
        {
            // ==========================================================
            // 1) Tentativa via CAN
            // ==========================================================
            case OBD_MODE_TRY_CAN:
                OBD2_TASK_DEBUG_PRINTLN("[FSM] Tentando comunicação via CAN...");

                KLine.powerDownBus();
                setMuxState(systemStatus, false);
                OBD2_TASK_DEBUG_PRINTLN("[MUX] Aguardando estabilização...");
                vTaskDelay(pdMS_TO_TICKS(MUX_SETTLE_MS));

                OBD2.beginAuto();
                vTaskDelay(pdMS_TO_TICKS(CAN_DRIVER_STABILIZE_MS));

                if (OBD2.connected())
                {
                    OBD2_TASK_DEBUG_PRINTLN("✅ Comunicação CAN estabelecida!");
                    obdMode = OBD_MODE_CAN_ACTIVE;
                    break;
                }

                OBD2_TASK_DEBUG_PRINTLN("❌ Falha no CAN. Tentando K-Line...");
                OBD2.end();
                obdMode           = OBD_MODE_TRY_KLINE;
                lastSwitchAttempt = millis();
                break;

            // ==========================================================
            // 2) CAN ativo
            // ==========================================================
            case OBD_MODE_CAN_ACTIVE:
                OBD2.tick(); // mantém FSM da lib viva

                if (!OBD2.connected())
                {
                    OBD2_TASK_DEBUG_PRINTLN("⚠️ CAN desconectado. Alternando para K-Line...");
                    OBD2.end();
                    obdMode           = OBD_MODE_TRY_KLINE;
                    lastSwitchAttempt = millis();
                    break;
                }

                if ((millis() - lastRequest) >= CAN_POLL_INTERVAL_MS)
                {
                    lastRequest = millis();
                    float rpm = 0.0f;
                    if (OBD2.readPid(PID_ENGINE_RPM, rpm))
                    {
                        systemStatus->automotiveSystem.canMonitor.engineRPM = rpm;
                        OBD2_TASK_DEBUG_PRINTF("🔥 [CAN] RPM: %.0f\n", rpm);
                    }
                }
                break;

            // ==========================================================
            // 3) Tentativa via K-Line
            // ==========================================================
            case OBD_MODE_TRY_KLINE:
                if ((millis() - lastSwitchAttempt) < KLINE_RETRY_INTERVAL_MS)
                    break;  // evita flood

                OBD2_TASK_DEBUG_PRINTLN("[FSM] Tentando comunicação via K-Line...");

                OBD2.end();
                setMuxState(systemStatus, true);
                OBD2_TASK_DEBUG_PRINTLN("[MUX] Aguardando estabilização...");
                vTaskDelay(pdMS_TO_TICKS(MUX_SETTLE_MS));

                KLine.setSerial(true);
                klineConnected = KLine.initOBD2();

                if (klineConnected)
                {
                    OBD2_TASK_DEBUG_PRINTLN("✅ Comunicação K-Line estabelecida!");
                    obdMode = OBD_MODE_KLINE_ACTIVE;
                    break;
                }

                OBD2_TASK_DEBUG_PRINTLN("❌ Falha na K-Line. Voltando ao CAN...");
                KLine.powerDownBus();
                obdMode           = OBD_MODE_TRY_CAN;
                lastSwitchAttempt = millis();
                break;

            // ==========================================================
            // 4) K-Line ativa
            // ==========================================================
            case OBD_MODE_KLINE_ACTIVE:
            {
                if (!klineConnected)
                {
                    obdMode = OBD_MODE_TRY_CAN;
                    break;
                }

                if ((millis() - lastRequest) >= KLINE_POLL_INTERVAL_1PID_MS)
                {
                    lastRequest = millis();
                    float rpm = KLine.getPID(read_LiveData, PID_ENGINE_RPM);

                    if (rpm >= 0.0f)
                    {
                        systemStatus->automotiveSystem.canMonitor.engineRPM = rpm;
                        klineFails = 0;
                        OBD2_TASK_DEBUG_PRINTF("🔥 [KLINE] RPM: %.0f\n", rpm);
                    }
                    else
                    {
                        klineFails++;
                        OBD2_TASK_DEBUG_PRINTF("⚠️ [KLINE] Falha %d/%d ao ler RPM\n",
                                                klineFails, KLINE_MAX_FAILS);

                        if (klineFails >= KLINE_MAX_FAILS)
                        {
                            OBD2_TASK_DEBUG_PRINTLN("❌ [KLINE] Falhas consecutivas. Voltando ao CAN...");
                            KLine.powerDownBus();
                            klineConnected    = false;
                            obdMode           = OBD_MODE_TRY_CAN;
                            lastSwitchAttempt = millis();
                            klineFails        = 0;
                        }
                    }
                }

                // (Opcional) manter sessão ativa se a lib expuser tester-present:
                // if (millis() - lastKeepAlive >= 1800) { KLine.testerPresent(); lastKeepAlive = millis(); }

                break;
            }

        } // <-- Faltava essa chave de fechamento do switch!

        // ---- Log de status a cada 3 s ----
        const uint32_t STATUS_LOG_PERIOD_MS = 3000U;
        static uint32_t lastStatusLog = 0;
        if ((millis() - lastStatusLog) >= STATUS_LOG_PERIOD_MS)
        {
            lastStatusLog = millis();
            OBD2_TASK_DEBUG_PRINTF("[STATUS] Modo:%d | CAN:%u | KLine:%u | RPM:%.0f\n",
                                   (int)obdMode,
                                   BOOL_TO_U8(OBD2.connected()),
                                   BOOL_TO_U8(klineConnected),
                                   systemStatus->automotiveSystem.canMonitor.engineRPM);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
