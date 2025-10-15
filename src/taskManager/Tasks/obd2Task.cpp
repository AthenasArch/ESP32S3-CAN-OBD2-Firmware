#include <Arduino.h>
#include <athenasObd2.h>
#include <esp32_can.h>
#include <can_common.h>
#include "Hardware/hardware.h"
#include "taskManager/Tasks/obd2Task.hpp"
#include "taskManager/taskManager.h"
#include "OBD2_KLine.h"

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

// Timers
static uint32_t lastFast = 0;
static uint32_t lastMed  = 0;
static uint32_t lastSlow = 0;
static uint32_t lastLog  = 0;

static void collectSelectedGauge(SystemStatus* ss);

// ---------- Auxiliares ----------
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

// <<< PROTEÇÃO: nunca tenta ler PID se o driver/barramento não está pronto
static inline bool readPidSafe(uint8_t pid, float &out)
{
    if (!OBD2.connected()) return false;  // <<< guarda-chuva contra fila nula
    return OBD2.readPid(pid, out);
}

/**
 * Coleta os PIDs rápidos (RPM e, se for o gauge ativo, SPEED).
 * Deve ser chamado a cada ~300 ms (ou menos).
 * RPM é sempre lido; SPEED só se o gauge ativo for SPEED.
 * @param st ponteiro para SystemStatus
 */
static void collectFast(SystemStatus *st)
{
    const uint8_t idx = st->nvsAutomotiveConfig.cfgMemory.currGaugeIndex % TOTAL_AUTOMOTIVE_GAUGES;

    float v;

    if (idx == 3) {
        // Gauge 3 (0→S): ler SOMENTE velocidade, rápido
        if (readPidSafe(VEHICLE_SPEED, v))
            st->automotiveSystem.canMonitor.vehicleSpeed = (uint16_t)v;
        return; // não lê RPM aqui
    }

    // Demais gauges: RPM rápido como antes
    if (readPidSafe(ENGINE_RPM, v))
        st->automotiveSystem.canMonitor.engineRPM = (uint16_t)v;
    // (Velocidade nos gauges 0..2 é tratada em collectSelectedGauge quando o PID selecionado é SPEED)
}


static void collectMedium(SystemStatus *st)
{
    float v;
    if (readPidSafe(FUEL_TANK_LEVEL_INPUT, v))
        st->automotiveSystem.canMonitor.fuelLevel = (int)v;
    if (readPidSafe(ENGINE_COOLANT_TEMPERATURE, v))
        st->automotiveSystem.canMonitor.engineCoolantTemperature = (int)v;
    if (readPidSafe(AIR_INTAKE_TEMPERATURE, v))
        st->automotiveSystem.canMonitor.intakeAirTemperature = (int)v;
    if (readPidSafe(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE, v))
        st->automotiveSystem.canMonitor.intakeManifoldPressure = (int)v;
}

static void collectSlow(SystemStatus *st)
{
    float v;
    if (readPidSafe(THROTTLE_POSITION, v))
        st->automotiveSystem.canMonitor.throttlePosition = (int)v;
    if (readPidSafe(TIMING_ADVANCE, v))
        st->automotiveSystem.canMonitor.ignitionTimingAdvance = (int)v;
    if (readPidSafe(CALCULATED_ENGINE_LOAD, v))
        st->automotiveSystem.canMonitor.calculatedLoadValue = v;
    if (readPidSafe(MAF_AIR_FLOW_RATE, v))
        st->automotiveSystem.canMonitor.massAirFlowRate = v;
}

static void debugDump(SystemStatus *st)
{
    OBD2_TASK_DEBUG_PRINTLN("====== OBD MONITOR ======");
    OBD2_TASK_DEBUG_PRINTF("Baud: %lu | Ext: %d | Conn: %d\n",
                          (unsigned long)OBD2.currentBaud(),
                          OBD2.isExtended() ? 1 : 0,
                          OBD2.connected() ? 1 : 0);

    OBD2_TASK_DEBUG_PRINTF("RPM: %ld\n", st->automotiveSystem.canMonitor.engineRPM);
    OBD2_TASK_DEBUG_PRINTF("Velocidade: %d km/h\n", st->automotiveSystem.canMonitor.vehicleSpeed);
    OBD2_TASK_DEBUG_PRINTF("Combustível: %d\n", st->automotiveSystem.canMonitor.fuelLevel);
    OBD2_TASK_DEBUG_PRINTF("Temp. Motor: %d °C\n", st->automotiveSystem.canMonitor.engineCoolantTemperature);
    OBD2_TASK_DEBUG_PRINTF("Temp. Ar: %d °C\n", st->automotiveSystem.canMonitor.intakeAirTemperature);
    OBD2_TASK_DEBUG_PRINTF("Pressão Coletor: %d kPa\n", st->automotiveSystem.canMonitor.intakeManifoldPressure);
    OBD2_TASK_DEBUG_PRINTF("Posição Acelerador: %d %%\n", st->automotiveSystem.canMonitor.throttlePosition);
    OBD2_TASK_DEBUG_PRINTF("Avanço Ignição: %d °\n", st->automotiveSystem.canMonitor.ignitionTimingAdvance);
    OBD2_TASK_DEBUG_PRINTF("Carga Calculada: %.2f\n", st->automotiveSystem.canMonitor.calculatedLoadValue);
    OBD2_TASK_DEBUG_PRINTF("MAF: %.2f g/s\n", st->automotiveSystem.canMonitor.massAirFlowRate);
    OBD2_TASK_DEBUG_PRINTLN("==========================");
}

/**************************************************************
 * @brief  Lê o PID do gauge atualmente exibido (número central).
 *
 * Regras:
 *  - Gauge 3 (0→S):            COLETA APENAS VELOCIDADE (ritmo ~30 ms).
 *  - Demais gauges (0..2):
 *      - RPM:                  nada a fazer (já vem de collectFast()).
 *      - SPEED:                lê em toda chamada (ritmo ~30 ms).
 *      - Outros PIDs:          respeita períodos internos (mais longos).
 *
 * Observações:
 *  - Esta função pressupõe ser chamada logo após collectFast() no laço ~30 ms.
 *  - readPidSafe() já protege contra barramento/driver desconectado.
 **************************************************************/
static void collectSelectedGauge(SystemStatus* ss)
{
    // Índice do gauge atual: 0,1,2 (gauges normais) e 3 (0→S)
    const uint8_t idx = ss->nvsAutomotiveConfig.cfgMemory.currGaugeIndex % TOTAL_AUTOMOTIVE_GAUGES;

    // =========================
    // GAUGE 3: 0→S (tela especial)
    // - Não acessa slots[3] (evita out-of-bounds, pois slots tem 3 entradas).
    // - Requisito: ler SOMENTE velocidade, com a mesma cadência do loop rápido.
    // =========================
    if (idx == 3) {
        float v;
        if (readPidSafe(VEHICLE_SPEED, v)) {
            ss->automotiveSystem.canMonitor.vehicleSpeed = (uint16_t)v;
        }
        return; // nada além de velocidade nesta tela
    }

    // =========================
    // GAUGES 0..2: usam a configuração do slot correspondente
    // =========================
    const DbAutomotiveGaugePids selPid = ss->nvsAutomotiveConfig.cfgMemory.slots[idx].pid;

    // RPM já é lido por collectFast() (ritmo ~30 ms). Não faça chamada extra aqui.
    if (selPid == AUTOMOTIVE_GAUGE_PID_RPM) {
        return;
    }

    // SPEED (nos gauges 0..2): ler sempre que esta função for chamada (ritmo ~30 ms)
    if (selPid == AUTOMOTIVE_GAUGE_PID_SPEED) {
        float v;
        if (readPidSafe(VEHICLE_SPEED, v)) {
            ss->automotiveSystem.canMonitor.vehicleSpeed = (uint16_t)v;
        }
        return;
    }

    // =========================
    // Demais PIDs (períodos moderados/lentos para não sobrecarregar a ECU)
    // =========================

    // Estes estáticos implementam o rate limit por PID selecionado.
    static uint32_t lastReadMs = 0;
    static DbAutomotiveGaugePids lastPid = AUTOMOTIVE_GAUGE_PID_RPM;

    // Período padrão e ajustes por PID (em ms)
    uint32_t periodMs = 500;   // padrão: 0,5 s
    if (selPid == AUTOMOTIVE_GAUGE_PID_THROTTLE_POS)      periodMs = 100;   // mais responsivo
    if (selPid == AUTOMOTIVE_GAUGE_PID_FUEL_TANK_LEVEL)   periodMs = 1000;  // bem lento
    if (selPid == AUTOMOTIVE_GAUGE_PID_COOLANT_TEMP)      periodMs = 500;
    if (selPid == AUTOMOTIVE_GAUGE_CONTROL_MODULE_VOLTAGE)periodMs = 500;

    // Se o usuário trocou o PID do slot, forza leitura imediata no novo PID
    if (selPid != lastPid) {
        lastPid    = selPid;
        lastReadMs = 0;
    }

    const uint32_t now = millis();
    if ((now - lastReadMs) < periodMs) {
        return; // ainda não é hora de consultar de novo este PID
    }
    lastReadMs = now;

    // Leitura do PID selecionado e gravação no campo correspondente
    float v = 0.0f;
    switch (selPid) {
        case AUTOMOTIVE_GAUGE_PID_COOLANT_TEMP:
            if (readPidSafe(ENGINE_COOLANT_TEMPERATURE, v)) {
                ss->automotiveSystem.canMonitor.engineCoolantTemperature = (int)v; // signed int
            }
            break;

        case AUTOMOTIVE_GAUGE_PID_THROTTLE_POS:
            if (readPidSafe(THROTTLE_POSITION, v)) {
                ss->automotiveSystem.canMonitor.throttlePosition = (uint8_t)v; // 0..100
            }
            break;

        case AUTOMOTIVE_GAUGE_CONTROL_MODULE_VOLTAGE:
            if (readPidSafe(CONTROL_MODULE_VOLTAGE, v)) {
                ss->automotiveSystem.canMonitor.batteryVoltage = v; // float (ex.: 12.6)
            }
            break;

        case AUTOMOTIVE_GAUGE_PID_FUEL_TANK_LEVEL:
            if (readPidSafe(FUEL_TANK_LEVEL_INPUT, v)) {
                ss->automotiveSystem.canMonitor.fuelLevel = (uint8_t)v; // 0..100
            }
            break;

        default:
            // Outros PIDs não mapeados aqui
            break;
    }
}




// // ---------- Task principal ----------
// void obd2Task_run(void *pvParameters)
// {
//     SystemStatus *systemStatus = (SystemStatus *)pvParameters;

//     uint32_t lastBg = 0;

//     systemStatus->automotiveSystem.zeroToS.state     = ZTS_IDLE;
//     systemStatus->automotiveSystem.zeroToS.targetKmh =
//         systemStatus->nvsAutomotiveConfig.cfgMemory.zeroToS_target_kmh;

//     OBD2_TASK_DEBUG_PRINTLN("------------------------");
//     OBD2_TASK_DEBUG_PRINTLN("    CAN OBD2 MONITOR TASK");
//     OBD2_TASK_DEBUG_PRINTLN("------------------------");


//     vTaskDelay(pdMS_TO_TICKS(TIME_TO_INIT_TASK_CAN));

//     // Pinos do TWAI
//     CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX,
//                     (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);

//     // FSM OBD2
//     OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);
//     OBD2.setHeartbeatPid(ENGINE_RPM);
//     OBD2.setHeartbeatInterval(1000);
//     OBD2.setBackoff(1000, 8000);
//     OBD2.setReconnectInterval(20000);

//     OBD2.beginAuto(); // primeira tentativa

//     systemStatus->automotiveSystem.canConnected = OBD2.connected();
//     systemStatus->automotiveSystem.canBaud     = OBD2.currentBaud();
//     systemStatus->automotiveSystem.canExtended = OBD2.isExtended() ? 1 : 0;
//     resetValuePIDs(systemStatus);

//     unsigned long canTimer = millis();

//     for (;;)
//     {
//         task_checkUsedMem(TASK_NAME_OBD2, &canTimer);

//         const bool onAutoPages =
//             systemStatus->display.currPage == DISPLAY_PAGE_AUTOMOTIVE_GAUGES ||
//             systemStatus->display.currPage == DISPLAY_PAGE_NIKO;

//         // Mantém FSM viva sempre (mas só ler PID quando conectado)
//         OBD2.tick();

//         // Reflete status atual para o app
//         systemStatus->automotiveSystem.canConnected = OBD2.connected();
//         systemStatus->automotiveSystem.canBaud     = OBD2.currentBaud();
//         systemStatus->automotiveSystem.canExtended = OBD2.isExtended() ? 1 : 0;

//         if (!onAutoPages) {
//             // <<< MODO BACKGROUND: NÃO HÁ LEITURA DE PID AQUI >>>
//             // Antes havia collectFast() de 1 em 1s — isso causava leitura com driver fora do ar.
//             // Mantemos só o heartbeat/tick e uma coleta ocasional *apenas se* conectado.
//             if (OBD2.connected() && (millis() - lastBg > 1000)) {
//                 lastBg = millis();
//                 collectFast(systemStatus);  // ok porque está conectado (guard acima)
//             }
//             vTaskDelay(pdMS_TO_TICKS(100)); // mais responsivo que 200ms
//             continue;
//         }

//         if (systemStatus->automotiveSystem.canConnected)
//         {
//             uint32_t now = millis();

//             if (now - lastFast >= 30) {
//                 lastFast = now;
//                 collectFast(systemStatus);
//                 collectSelectedGauge(systemStatus);       // valor central do gauge atual
//             }
//             if (now - lastMed >= 600) {
//                 lastMed = now;
//                 // collectMedium(systemStatus);
//             }
//             if (now - lastSlow >= 1000) {
//                 lastSlow = now;
//                 // collectSlow(systemStatus);
//             }
//             if (now - lastLog >= 1000) {
//                 lastLog = now;
//                 debugDump(systemStatus);
//             }
//         }
//         else
//         {
//             // Não conectado: zera PIDs e loga tempo p/ próxima tentativa
//             resetValuePIDs(systemStatus);

//             static uint32_t lastDebugReconnect = 0;
//             uint32_t now = millis();

//             if (now - lastDebugReconnect >= 1000) {
//                 lastDebugReconnect = now;
//                 systemStatus->automotiveSystem.nextConnectionTimeout = OBD2.timeToNextReconnect();
//                 OBD2_TASK_DEBUG_PRINTF("Sem conexão. Próxima tentativa em %lu ms\n",
//                     (unsigned long)systemStatus->automotiveSystem.nextConnectionTimeout);
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }



void testServoControl(SystemStatus* systemStatus)
{
    static uint32_t lastToggle = 0;
    static bool state = false;

    DbServoChannel* ch = &systemStatus->machine.servo.leftFront;
    ch->mode = SERVO_MODE_GPIO;    // ativa modo GPIO no PCA
    ch->pcaChannel = 4;            // canal físico CH4 do PCA
    ch->updated = true;

    if (millis() - lastToggle >= 10000)
    {
        lastToggle = millis();
        state = !state;
        ch->currentValue = state ? 1 : 0;
        ch->updated = true;
        Serial.printf("[testServoControl] ⚡ CH%d -> %s\n", ch->pcaChannel, state ? "HIGH" : "LOW");
    }
}




// =============================================================
// === CONFIGURAÇÕES DO HARDWARE (ajuste conforme sua placa) ===
// =============================================================
#define KLINE_RX_PIN PIN_PCI_ATHENAS_CAN_RX  // ESP32 RX ← L9637D TX
#define KLINE_TX_PIN PIN_PCI_ATHENAS_CAN_TX  // ESP32 TX → L9637D RX
#define KLINE_BAUDRATE 10400                 // ISO9141/KWP2000 FAST

// =============================================================
// === INSTÂNCIA DO DRIVER K-LINE ===============================
// =============================================================
static OBD2_KLine KLine(Serial1, KLINE_BAUDRATE, KLINE_RX_PIN, KLINE_TX_PIN);

// =============================================================
// === TASK PRINCIPAL PARA TESTE DO K-LINE COM L9637D ============
// =============================================================
void obd2Task_run(void *pvParameters)
{
    SystemStatus *systemStatus = (SystemStatus *)pvParameters;

    OBD2_TASK_DEBUG_PRINTLN("\n==============================");
    OBD2_TASK_DEBUG_PRINTLN("     K-LINE TEST TASK (L9637D)");
    OBD2_TASK_DEBUG_PRINTLN("==============================\n");

    vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda o sistema estabilizar

    // Inicializa a UART (Serial1)
    Serial1.begin(KLINE_BAUDRATE, SERIAL_8N1, KLINE_RX_PIN, KLINE_TX_PIN);

    // === Configurações de protocolo e debug ===
    // KLine.setDebug(Serial);
    // KLine.setSerial(true);
    KLine.setProtocol("Automatic");   // Auto: tenta ISO9141 e ISO14230
    KLine.setByteWriteInterval(5);
    KLine.setInterByteTimeout(60);
    KLine.setReadTimeout(1000);

    unsigned long lastRequest = millis();
    unsigned long lastLog = millis();
    unsigned long taskTimer = millis();

    bool connected = false;

    for (;;)
    {
        testServoControl(systemStatus); // função de teste do servo
        task_checkUsedMem(TASK_NAME_OBD2, &taskTimer);

        // === Etapa de inicialização automática ===
        if (!connected)
        {
            OBD2_TASK_DEBUG_PRINTLN("Iniciando comunicação com ECU...");
            connected = KLine.initOBD2();  // método oficial da biblioteca

            if (connected)
            {
                OBD2_TASK_DEBUG_PRINTLN("✅ Comunicação K-Line estabelecida!");
                systemStatus->automotiveSystem.canConnected = true;
            }
            else
            {
                OBD2_TASK_DEBUG_PRINTLN("❌ Falha ao inicializar K-Line. Tentando novamente...");
                systemStatus->automotiveSystem.canConnected = false;
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
        }

        // === Leitura de PIDs suportados (modo 01) ===
        if (millis() - lastRequest >= 2000)
        {
            lastRequest = millis();
            OBD2_TASK_DEBUG_PRINTLN("🔍 Solicitando PIDs suportados (modo 0x01)");

            int liveDataLength = KLine.readSupportedLiveData();
            if (liveDataLength > 0)
            {
                OBD2_TASK_DEBUG_PRINTF("LiveData suportado (%d): ", liveDataLength);
                for (int i = 0; i < liveDataLength; i++)
                {
                    byte pid = KLine.getSupportedData(0x01, i);
                    Serial.printf("%02X ", pid);
                }
                Serial.println();
            }
            else
            {
                OBD2_TASK_DEBUG_PRINTLN("⚠️ Nenhum PID suportado retornado ou falha na leitura.");
            }

            // === Leitura de RPM (PID 0x0C) ===
            OBD2_TASK_DEBUG_PRINTLN("🔁 Solicitando PID 0x0C (Engine RPM)...");
            float rpm = KLine.getPID(read_LiveData, 0x0C);

            if (rpm > 0)
            {
                systemStatus->automotiveSystem.canMonitor.engineRPM = rpm;
                OBD2_TASK_DEBUG_PRINTF("🔥 RPM: %.0f rpm\n", rpm);
            }
            else
            {
                OBD2_TASK_DEBUG_PRINTLN("❌ Falha ao ler PID 0x0C.");
            }
        }

        // === Log periódico de status ===
        if (millis() - lastLog >= 3000)
        {
            lastLog = millis();
            OBD2_TASK_DEBUG_PRINTF("[KLINE] Conectado: %d | Baud: %lu | RPM: %.0f\n",
                                   connected,
                                   (unsigned long)KLINE_BAUDRATE,
                                   systemStatus->automotiveSystem.canMonitor.engineRPM);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
