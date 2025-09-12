/**
 * CAN OBD2 Monitor com detecção automática de protocolo
 * - Zera PIDs ao perder conexão
 * - Redetecção robusta (mutex + cooldown) sem double-init do TWAI
 * - Evita crashes por begin()/end() concorrentes.
 * 
 * 		ISO15765-4 11 bit 500 k
 * 		ISO15765-4 11 bit 250 k 
 * 		ISO15765-4 29 bit 500 k
 * 		ISO15765-4 29 bit 250 k
 * 		ISO9141-2
 * 		ISO14230-4 KWP2000 5BPS
 * 		ISO14230-4 KWP2000 FAST
 * 
 */
/**
 * CAN OBD2 Monitor com detecção automática dos 4 modos CAN ISO15765-4:
 *  - 11b/500k  (usa esp32_obd2)
 *  - 11b/250k  (usa esp32_obd2)
 *  - 29b/500k  (usa esp32_can direto - manual)
 *  - 29b/250k  (usa esp32_can direto - manual)
 *
 * Observações:
 * - Protocolos K-Line (ISO9141-2 e ISO14230/KWP2000) NÃO são suportados por hardware CAN/TWAI.
 * - Para 29-bit a esp32_obd2 não cobre o fluxo (endereçamento estendido); por isso, usamos esp32_can.
 * - Evita double-init do TWAI: quando vamos usar OBD2, paramos o CAN "manual" e vice-versa.
 */

// #include <Arduino.h>
// #include <esp32_obd2.h> // Alto nível (11-bit)
// #include <esp32_can.h>  // CAN (TWAI) baixo nível p/ 29-bit
// #include <can_common.h>
// #include "Hardware/hardware.h"
// #include "taskManager/Tasks/canTask.hpp"
// #include "taskManager/taskManager.h"
// #include "freertos/semphr.h"

// // =====================
// // DEBUG
// // =====================
// #define CAN_TASK_DEBUG_ENABLE
// #ifdef CAN_TASK_DEBUG_ENABLE
//     #define CAN_TASK_DEBUG_PRINT(...)   { Serial.print(__VA_ARGS__); }
//     #define CAN_TASK_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
//     #define CAN_TASK_DEBUG_PRINTF(...)  { Serial.printf(__VA_ARGS__); }
// #else
//     #define CAN_TASK_DEBUG_PRINT(...)
//     #define CAN_TASK_DEBUG_PRINTLN(...)
//     #define CAN_TASK_DEBUG_PRINTF(...)
// #endif

// // =====================
// // Controle de tempo
// // =====================
// static uint32_t lastFast = 0;
// static uint32_t lastMedium = 0;
// static uint32_t lastSlow = 0;
// static uint32_t lastLog = 0;

// // =====================
// // Estado global
// // =====================
// enum CanAddrMode
// {
//     ADDR_11BIT = 0,
//     ADDR_29BIT = 1
// };

// static bool g_connected = false;
// static bool g_detectInProgress = false;
// static uint32_t g_lastDetectMs = 0;
// static uint32_t g_detectBackoffMs = 1000; // 1s -> 2s -> 4s -> 8s (máx)
// static uint8_t g_failCount = 0;

// static uint32_t g_currBaud = 0;
// static CanAddrMode g_currAddrMode = ADDR_11BIT;

// // Mutex para serializar begin()/end()
// static SemaphoreHandle_t g_canStackMutex = nullptr;

// // =====================
// // Reset PIDs
// // =====================
// static void canMonitor_reset(SystemStatus *s)
// {
//     s->automotiveSystem.canMonitor.engineRPM = 0;
//     s->automotiveSystem.canMonitor.vehicleSpeed = 0;
//     s->automotiveSystem.canMonitor.fuelLevel = 0;
//     s->automotiveSystem.canMonitor.engineCoolantTemperature = 0;
//     s->automotiveSystem.canMonitor.intakeAirTemperature = 0;
//     s->automotiveSystem.canMonitor.intakeManifoldPressure = 0;
//     s->automotiveSystem.canMonitor.throttlePosition = 0;
//     s->automotiveSystem.canMonitor.ignitionTimingAdvance = 0;
//     s->automotiveSystem.canMonitor.calculatedLoadValue = 0;
//     s->automotiveSystem.canMonitor.massAirFlowRate = 0;
// }

// // =====================
// // Util | fecha tudo
// // =====================
// static void close_all_stacks()
// {
//     // Fecha OBD2 (se estiver rodando)
//     OBD2.end();
//     // Fecha driver CAN que a esp32_can abriu
//     CAN0.disable();
// }

// // =====================
// // OBD2 (11-bit) stack restart
// // =====================
// static bool start_obd2_11bit(uint32_t baud)
// {
//     if (!g_canStackMutex){
//         g_canStackMutex = xSemaphoreCreateMutex();
//     }
//     xSemaphoreTake(g_canStackMutex, portMAX_DELAY);

//     close_all_stacks();
//     vTaskDelay(pdMS_TO_TICKS(100));

//     CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX,
//                     (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);

//     // Inicia OBD2 primeiro (sempre em 500k internamente)
//     OBD2.begin();
//     OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);

//     // Força baud correto em seguida
//     CAN0.set_baudrate(baud);

//     g_currAddrMode = ADDR_11BIT;
//     xSemaphoreGive(g_canStackMutex);
//     return true;
// }

// // =====================
// // CAN manual (29-bit) stack restart
// // =====================
// static bool start_can_29bit(uint32_t baud)
// {
//     if (!g_canStackMutex){
//         g_canStackMutex = xSemaphoreCreateMutex();
//     }
        
//     xSemaphoreTake(g_canStackMutex, portMAX_DELAY);

//     close_all_stacks();
//     vTaskDelay(pdMS_TO_TICKS(200)); // cooldown

//     // Driver CAN direto (esp32_can cuida de instalar TWAI)
//     CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX, (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);
//     CAN0.set_baudrate(baud);

//     xSemaphoreGive(g_canStackMutex);
//     return true;
// }

// // =====================
// // SAFE PID via OBD2 (11-bit)
// // =====================
// static inline bool safePid11(float &out, uint8_t pid)
// {
//     float v = OBD2.pidRead(pid);
//     if (isnan(v))
//         return false;
//     out = v;
//     return true;
// }

// // =====================
// // Envio/Recebimento OBD-II 29-bit (manual)
// // - Request funcional 29-bit: 0x18DB33F1
// // - Respostas típicas: 0x18DAF1xx (qualquer ECU xx)
// // - ISO-TP Single Frame: data[0] = 0x02 (len=2), data[1]=serviço (0x01), data[2]=PID
// // =====================
// static bool send_pid_29(uint8_t service, uint8_t pid)
// {
//     CAN_FRAME tx;
//     tx.id = 0x18DB33F1; // Functional request 29-bit
//     tx.extended = true;
//     tx.rtr = 0;
//     tx.length = 8;
//     tx.data.byte[0] = 0x02;    // 2 bytes de payload (01, PID)
//     tx.data.byte[1] = service; // 0x01 = Current Data
//     tx.data.byte[2] = pid;
//     tx.data.byte[3] = 0x00;
//     tx.data.byte[4] = 0x00;
//     tx.data.byte[5] = 0x00;
//     tx.data.byte[6] = 0x00;
//     tx.data.byte[7] = 0x00;

//     return CAN0.sendFrame(tx);
// }

// static bool recv_pid_29(uint8_t expected_service_resp, uint8_t pid, uint8_t *outData, uint32_t timeout_ms)
// {
//     uint32_t start = millis();
//     while ((millis() - start) < timeout_ms)
//     {
//         if (CAN0.rx_avail())
//         {
//             CAN_FRAME rx;
//             if (CAN0.get_rx_buff(rx))
//             {
//                 if (!rx.extended)
//                     continue; // precisamos de 29-bit

//                 // Verificação básica do payload ISO-TP SF e do serviço/PIB
//                 // Para resposta, data[0] = 0x.. (len), data[1] = service|0x40, data[2] = pid ecoado
//                 if (rx.length >= 3)
//                 {
//                     uint8_t pci_len = rx.data.byte[0] & 0x0F; // nibble de len
//                     uint8_t svc = rx.data.byte[1];
//                     uint8_t pid_rx = rx.data.byte[2];

//                     if (svc == expected_service_resp && pid_rx == pid && pci_len >= 2)
//                     {
//                         // Copia o restante
//                         for (int i = 0; i < rx.length; i++)
//                             outData[i] = rx.data.byte[i];
//                         return true;
//                     }
//                 }
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(2));
//     }
//     return false;
// }

// // Decodificador básico de alguns PIDs (para 29-bit manual)
// // NOTA: mesma matemática do OBD2
// static bool pidRead29(uint8_t pid, float &outVal)
// {
//     const uint8_t SERVICE = 0x01;
//     const uint8_t SERVICE_RESP = 0x41;

//     if (!send_pid_29(SERVICE, pid))
//         return false;

//     uint8_t buf[8] = {0};
//     if (!recv_pid_29(SERVICE_RESP, pid, buf, 200))
//         return false;

//     // buf[0] = PCI (len), buf[1] = 0x41, buf[2] = pid
//     uint8_t A = buf[3];
//     uint8_t B = (buf[4]);
//     // uint8_t C = buf[5];
//     // uint8_t D = buf[6];

//     switch (pid)
//     {
//     case ENGINE_RPM:
//     { // 0x0C: RPM = (256*A + B)/4
//         outVal = (256.0f * A + B) / 4.0f;
//         return true;
//     }
//     case VEHICLE_SPEED:
//     { // 0x0D: km/h
//         outVal = A;
//         return true;
//     }
//     case ENGINE_COOLANT_TEMPERATURE: // 0x05: °C = A - 40
//     case AIR_INTAKE_TEMPERATURE:
//     { // 0x0F: °C = A - 40
//         outVal = (int)A - 40;
//         return true;
//     }
//     case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
//     { // 0x0B: kPa = A
//         outVal = A;
//         return true;
//     }
//     case THROTTLE_POSITION: // 0x11: % = A * 100 / 255
//     case CALCULATED_ENGINE_LOAD:
//     { // 0x04: % = A * 100 / 255
//         outVal = (A * 100.0f) / 255.0f;
//         return true;
//     }
//     case MAF_AIR_FLOW_RATE:
//     { // 0x10: g/s = (256*A + B)/100
//         outVal = (256.0f * A + B) / 100.0f;
//         return true;
//     }
//     case TIMING_ADVANCE:
//     { // 0x0E: ° = (A/2) - 64
//         outVal = (A / 2.0f) - 64.0f;
//         return true;
//     }
//     case FUEL_TANK_LEVEL_INPUT:
//     { // 0x2F: % = A * 100 / 255
//         outVal = (A * 100.0f) / 255.0f;
//         return true;
//     }
//     case PIDS_SUPPORT_01_20:
//     { // 0x00: apenas checagem (qualquer retorno serve)
//         outVal = 1.0f;
//         return true;
//     }
//     }
//     return false; // PID não mapeado aqui
// }

// // Wrapper de leitura que escolhe 11b/29b automaticamente
// static inline bool safePid(float &out, uint8_t pid)
// {
//     if (g_currAddrMode == ADDR_11BIT)
//     {
//         return safePid11(out, pid);
//     }
//     else
//     {
//         return pidRead29(pid, out);
//     }
// }

// // =====================
// // Coletas
// // =====================
// static void canMonitor_collectFastPIDs(SystemStatus *systemStatus)
// {
//     float v;
//     if (safePid(v, ENGINE_RPM))
//         systemStatus->automotiveSystem.canMonitor.engineRPM = (int)v;
//     if (safePid(v, VEHICLE_SPEED))
//         systemStatus->automotiveSystem.canMonitor.vehicleSpeed = (int)v;
// }

// static void canMonitor_collectMediumPIDs(SystemStatus *systemStatus)
// {
//     float v;
//     if (safePid(v, FUEL_TANK_LEVEL_INPUT))
//         systemStatus->automotiveSystem.canMonitor.fuelLevel = (int)v;
//     if (safePid(v, ENGINE_COOLANT_TEMPERATURE))
//         systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature = (int)v;
//     if (safePid(v, AIR_INTAKE_TEMPERATURE))
//         systemStatus->automotiveSystem.canMonitor.intakeAirTemperature = (int)v;
//     if (safePid(v, INTAKE_MANIFOLD_ABSOLUTE_PRESSURE))
//         systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure = (int)v;
// }

// static void canMonitor_collectSlowPIDs(SystemStatus *systemStatus)
// {
//     float v;
//     if (safePid(v, THROTTLE_POSITION))
//         systemStatus->automotiveSystem.canMonitor.throttlePosition = (int)v;
//     if (safePid(v, TIMING_ADVANCE))
//         systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance = (int)v;
//     if (safePid(v, CALCULATED_ENGINE_LOAD))
//         systemStatus->automotiveSystem.canMonitor.calculatedLoadValue = v;
//     if (safePid(v, MAF_AIR_FLOW_RATE))
//         systemStatus->automotiveSystem.canMonitor.massAirFlowRate = v;
// }

// // =====================
// // Debug
// // =====================
// static void canMonitor_debug(SystemStatus *systemStatus)
// {
//     CAN_TASK_DEBUG_PRINTLN("====== OBD MONITOR ======");
//     CAN_TASK_DEBUG_PRINTF("RPM: %ld\n", systemStatus->automotiveSystem.canMonitor.engineRPM);
//     CAN_TASK_DEBUG_PRINTF("Velocidade: %d km/h\n", systemStatus->automotiveSystem.canMonitor.vehicleSpeed);
//     CAN_TASK_DEBUG_PRINTF("Combustível: %d\n", systemStatus->automotiveSystem.canMonitor.fuelLevel);
//     CAN_TASK_DEBUG_PRINTF("Temp. Motor: %d °C\n", systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature);
//     CAN_TASK_DEBUG_PRINTF("Temp. Ar: %d °C\n", systemStatus->automotiveSystem.canMonitor.intakeAirTemperature);
//     CAN_TASK_DEBUG_PRINTF("Pressão Coletor: %d kPa\n", systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure);
//     CAN_TASK_DEBUG_PRINTF("Posição Acelerador: %d %%\n", systemStatus->automotiveSystem.canMonitor.throttlePosition);
//     CAN_TASK_DEBUG_PRINTF("Avanço Ignição: %d °\n", systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance);
//     CAN_TASK_DEBUG_PRINTF("Carga Calculada: %.2f\n", systemStatus->automotiveSystem.canMonitor.calculatedLoadValue);
//     CAN_TASK_DEBUG_PRINTF("MAF: %.2f g/s\n", systemStatus->automotiveSystem.canMonitor.massAirFlowRate);
//     CAN_TASK_DEBUG_PRINTLN("==========================");
// }

// // =====================
// // Detecção de protocolo CAN OBD-II
// // Testa em ordem:
// //   11b/500k → 11b/250k → 29b/500k → 29b/250k
// //
// // Critério de sucesso:
// //   - conseguir ler PIDS_SUPPORT_01_20 (PID 0x00) OU ENGINE_RPM (PID 0x0C)
// //   - e confirmar com uma segunda leitura válida de RPM
// //
// // Proteções adicionadas:
// //   - cooldown entre tentativas (g_detectBackoffMs)
// //   - safePid11() com proteção contra ponteiros inválidos (OBD2 sem resposta)
// //   - se não houver ECU, aborta aquele baud ao invés de crashar
// // =====================
// static bool detect_protocol(SystemStatus *sys)
// {
//     if (g_detectInProgress){
//         return false; // já rodando? aborta
//     }
    
//     // Lista de combinações a testar
//     const struct
//     {
//         uint32_t baud;
//         CanAddrMode mode;
//         const char *label;
//     } tries[] = {
//         {CAN_BPS_500K, ADDR_11BIT, "11b/500k"},
//         {CAN_BPS_250K, ADDR_11BIT, "11b/250k"}, // aqui dava crash → agora protegido
//         {CAN_BPS_500K, ADDR_29BIT, "29b/500k"},
//         {CAN_BPS_250K, ADDR_29BIT, "29b/250k"},
//     };

//     // aplica cooldown entre varreduras
//     const uint32_t now = millis();
//     if ((now - g_lastDetectMs) < g_detectBackoffMs){
//         return false;
//     }
        
//     g_detectInProgress = true;
//     bool detected = false;

//     for (auto &t : tries)
//     {
//         CAN_TASK_DEBUG_PRINTF("[CAN]\tTestando %s...\n", t.label);

//         // inicia stack correspondente (11b usa esp32_obd2, 29b é manual)
//         bool started = (t.mode == ADDR_11BIT) ? start_obd2_11bit(t.baud)
//                                               : start_can_29bit(t.baud);
//         if (!started){
//             continue;
//         }

//         vTaskDelay(pdMS_TO_TICKS(300)); // tempo pro barramento estabilizar

//         bool ok1 = false, ok2 = false;
//         float v;

//         // 1ª checagem: PID 0x00 (supported) ou RPM
//         if (t.mode == ADDR_11BIT)
//         {
//             ok1 = safePid11(v, PIDS_SUPPORT_01_20);
//             if (!ok1)
//                 ok1 = safePid11(v, ENGINE_RPM);
//         }
//         else
//         {
//             ok1 = pidRead29(PIDS_SUPPORT_01_20, v);
//             if (!ok1)
//                 ok1 = pidRead29(ENGINE_RPM, v);
//         }

//         // 2ª checagem: confirma RPM de novo
//         if (ok1)
//         {
//             if (t.mode == ADDR_11BIT)
//                 ok2 = safePid11(v, ENGINE_RPM);
//             else
//                 ok2 = pidRead29(ENGINE_RPM, v);
//         }

//         // Se as duas leituras foram válidas → protocolo confirmado
//         if (ok1 && ok2)
//         {
//             g_currBaud = t.baud;
//             g_currAddrMode = t.mode;

//             sys->automotiveSystem.canBaud = t.baud;
//             sys->automotiveSystem.canExtended = (t.mode == ADDR_29BIT) ? 1 : 0;
//             sys->automotiveSystem.canConnected = 1;

//             g_connected = true;
//             g_failCount = 0;

//             CAN_TASK_DEBUG_PRINTF("[CAN]\tECU detectada @ %s\n", t.label);
//             detected = true;
//             break; // sai do loop
//         }
//     }

//     if (!detected)
//     {
//         // não achou nenhum protocolo → limpa tudo
//         close_all_stacks();
//         g_connected = false;
//         sys->automotiveSystem.canConnected = 0;

//         // aumenta cooldown exponencialmente até 8s
//         g_detectBackoffMs = min<uint32_t>(g_detectBackoffMs << 1, 8000);
//         CAN_TASK_DEBUG_PRINTLN("[CAN]\tNenhum protocolo detectado");
//     }
//     else
//     {
//         // se detectou, reseta cooldown p/ resposta rápida
//         g_detectBackoffMs = 1000;
//     }

//     g_lastDetectMs = millis();
//     g_detectInProgress = false;
//     return detected;
// }

// // =====================
// // Task Principal
// // =====================
// void canTask_run(void *pvParameters)
// {
//     SystemStatus *systemStatus = (SystemStatus *)pvParameters;

//     CAN_TASK_DEBUG_PRINTLN("------------------------");
//     CAN_TASK_DEBUG_PRINTLN("    CAN OBD2 MONITOR TASK");
//     CAN_TASK_DEBUG_PRINTLN("------------------------");

//     delay(TIME_TO_INIT_TASK_CAN);

//     // Estado inicial
//     g_connected = false;
//     g_detectInProgress = false;
//     g_lastDetectMs = 0;
//     g_detectBackoffMs = 1000;
//     systemStatus->automotiveSystem.canConnected = false;
//     systemStatus->automotiveSystem.canBaud = 0;
//     systemStatus->automotiveSystem.canExtended = false; // = 29 bits
    
//     canMonitor_reset(systemStatus);

//     if (!g_canStackMutex){
//         g_canStackMutex = xSemaphoreCreateMutex();
//     }

//     // Primeira detecção
//     detect_protocol(systemStatus);

//     unsigned long canTimer = millis();

//     for (;;)
//     {
//         task_checkUsedMem(TASK_NAME_CAN, &canTimer);

//         if (g_connected && systemStatus->automotiveSystem.canConnected)
//         {
//             uint32_t now = millis();

//             if (now - lastFast >= 30)
//             {
//                 lastFast = now;
//                 canMonitor_collectFastPIDs(systemStatus);
//             }
//             if (now - lastMedium >= 600)
//             {
//                 lastMedium = now;
//                 canMonitor_collectMediumPIDs(systemStatus);
//             }
//             if (now - lastSlow >= 1000)
//             {
//                 lastSlow = now;
//                 canMonitor_collectSlowPIDs(systemStatus);
//             }
//             if (now - lastLog >= 1000)
//             {
//                 lastLog = now;
//                 canMonitor_debug(systemStatus);
//             }

//             // Heartbeat: duas verificações para “não confundir” ruído com perda real
//             float tmp;
//             bool okHeartbeat = safePid(tmp, ENGINE_RPM);
//             if (!okHeartbeat)
//             {
//                 float dummy;
//                 okHeartbeat = safePid(dummy, PIDS_SUPPORT_01_20);
//             }

//             if (!okHeartbeat)
//             {
//                 if (++g_failCount > 5)
//                 {
//                     CAN_TASK_DEBUG_PRINTLN("[CAN]\tConexão perdida, zerando e redetectando...");
//                     g_connected = false;
//                     systemStatus->automotiveSystem.canConnected = 0;
//                     canMonitor_reset(systemStatus);

//                     close_all_stacks();
//                     g_lastDetectMs = millis();
//                     g_detectBackoffMs = 1000;
//                     g_failCount = 0;
//                 }
//             }
//             else
//             {
//                 g_failCount = 0;
//             }
//         }
//         else
//         {
//             // Não conectado: zera e tenta (respeitando cooldown)
//             canMonitor_reset(systemStatus);
//             detect_protocol(systemStatus);
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }



















#include <Arduino.h>
#include <esp32_obd2.h>
#include <esp32_can.h>
#include <can_common.h>
#include "Hardware/hardware.h"
#include "taskManager/Tasks/canTask.hpp"
#include "taskManager/taskManager.h"

// =====================
// DEBUG
// =====================
#define CAN_TASK_DEBUG_ENABLE
#ifdef CAN_TASK_DEBUG_ENABLE
  #define CAN_TASK_DEBUG_PRINT(...)   { Serial.print(__VA_ARGS__); }
  #define CAN_TASK_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
  #define CAN_TASK_DEBUG_PRINTF(...)  { Serial.printf(__VA_ARGS__); }
#else
  #define CAN_TASK_DEBUG_PRINT(...)
  #define CAN_TASK_DEBUG_PRINTLN(...)
  #define CAN_TASK_DEBUG_PRINTF(...)
#endif

// =====================
// Controle de tempo de coleta/log
// =====================
static uint32_t lastFast  = 0;
static uint32_t lastMed   = 0;
static uint32_t lastSlow  = 0;
static uint32_t lastLog   = 0;

// =====================
// Auxiliares
// =====================
static void resetPIDs(SystemStatus *s) {
  s->automotiveSystem.canMonitor.engineRPM = 0;
  s->automotiveSystem.canMonitor.vehicleSpeed = 0;
  s->automotiveSystem.canMonitor.fuelLevel = 0;
  s->automotiveSystem.canMonitor.engineCoolantTemperature = 0;
  s->automotiveSystem.canMonitor.intakeAirTemperature = 0;
  s->automotiveSystem.canMonitor.intakeManifoldPressure = 0;
  s->automotiveSystem.canMonitor.throttlePosition = 0;
  s->automotiveSystem.canMonitor.ignitionTimingAdvance = 0;
  s->automotiveSystem.canMonitor.calculatedLoadValue = 0;
  s->automotiveSystem.canMonitor.massAirFlowRate = 0;
}

static inline bool readPidSafe(uint8_t pid, float &out) {
  return OBD2.readPid(pid, out);
}

// =====================
// Coletas
// =====================
static void collectFast(SystemStatus *st) {
  float v;
  if (readPidSafe(ENGINE_RPM, v))        st->automotiveSystem.canMonitor.engineRPM = (int)v;
  if (readPidSafe(VEHICLE_SPEED, v))     st->automotiveSystem.canMonitor.vehicleSpeed = (int)v;
}

static void collectMedium(SystemStatus *st) {
  float v;
  if (readPidSafe(FUEL_TANK_LEVEL_INPUT, v))           st->automotiveSystem.canMonitor.fuelLevel = (int)v;
  if (readPidSafe(ENGINE_COOLANT_TEMPERATURE, v))      st->automotiveSystem.canMonitor.engineCoolantTemperature = (int)v;
  if (readPidSafe(AIR_INTAKE_TEMPERATURE, v))          st->automotiveSystem.canMonitor.intakeAirTemperature = (int)v;
  if (readPidSafe(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE, v)) st->automotiveSystem.canMonitor.intakeManifoldPressure = (int)v;
}

static void collectSlow(SystemStatus *st) {
  float v;
  if (readPidSafe(THROTTLE_POSITION, v))       st->automotiveSystem.canMonitor.throttlePosition = (int)v;
  if (readPidSafe(TIMING_ADVANCE, v))          st->automotiveSystem.canMonitor.ignitionTimingAdvance = (int)v;
  if (readPidSafe(CALCULATED_ENGINE_LOAD, v))  st->automotiveSystem.canMonitor.calculatedLoadValue = v;
  if (readPidSafe(MAF_AIR_FLOW_RATE, v))       st->automotiveSystem.canMonitor.massAirFlowRate = v;
}

static void debugDump(SystemStatus *st) {
  CAN_TASK_DEBUG_PRINTLN("====== OBD MONITOR ======");
  CAN_TASK_DEBUG_PRINTF("Baud: %lu | Ext: %d | Conn: %d\n",
    (unsigned long)OBD2.currentBaud(),
    OBD2.isExtended() ? 1 : 0,
    OBD2.connected() ? 1 : 0);

  CAN_TASK_DEBUG_PRINTF("RPM: %ld\n", st->automotiveSystem.canMonitor.engineRPM);
  CAN_TASK_DEBUG_PRINTF("Velocidade: %d km/h\n", st->automotiveSystem.canMonitor.vehicleSpeed);
  CAN_TASK_DEBUG_PRINTF("Combustível: %d\n", st->automotiveSystem.canMonitor.fuelLevel);
  CAN_TASK_DEBUG_PRINTF("Temp. Motor: %d °C\n", st->automotiveSystem.canMonitor.engineCoolantTemperature);
  CAN_TASK_DEBUG_PRINTF("Temp. Ar: %d °C\n", st->automotiveSystem.canMonitor.intakeAirTemperature);
  CAN_TASK_DEBUG_PRINTF("Pressão Coletor: %d kPa\n", st->automotiveSystem.canMonitor.intakeManifoldPressure);
  CAN_TASK_DEBUG_PRINTF("Posição Acelerador: %d %%\n", st->automotiveSystem.canMonitor.throttlePosition);
  CAN_TASK_DEBUG_PRINTF("Avanço Ignição: %d °\n", st->automotiveSystem.canMonitor.ignitionTimingAdvance);
  CAN_TASK_DEBUG_PRINTF("Carga Calculada: %.2f\n", st->automotiveSystem.canMonitor.calculatedLoadValue);
  CAN_TASK_DEBUG_PRINTF("MAF: %.2f g/s\n", st->automotiveSystem.canMonitor.massAirFlowRate);
  CAN_TASK_DEBUG_PRINTLN("==========================");
}

// =====================
// Task principal
// =====================
void canTask_run(void *pvParameters) {
  SystemStatus *systemStatus = (SystemStatus *)pvParameters;

  CAN_TASK_DEBUG_PRINTLN("------------------------");
  CAN_TASK_DEBUG_PRINTLN("    CAN OBD2 MONITOR TASK");
  CAN_TASK_DEBUG_PRINTLN("------------------------");

  delay(TIME_TO_INIT_TASK_CAN);

  // Pinos do TWAI
  CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX,
                  (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);

  // Ajustes da FSM da lib
  OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);
  OBD2.setHeartbeatPid(ENGINE_RPM);
  OBD2.setHeartbeatInterval(1000); // 1s
  OBD2.setBackoff(1000, 8000);     // 1s..8s

  // Primeira varredura automática
  OBD2.beginAuto();

  // Estado inicial do app
  systemStatus->automotiveSystem.canConnected = OBD2.connected();
  systemStatus->automotiveSystem.canBaud     = OBD2.currentBaud();
  systemStatus->automotiveSystem.canExtended = OBD2.isExtended() ? 1 : 0;
  resetPIDs(systemStatus);

  unsigned long canTimer = millis();

  for (;;) {
    task_checkUsedMem(TASK_NAME_CAN, &canTimer);

    // Deixa a biblioteca cuidar de heartbeat/redetecção
    OBD2.tick();

    // Reflete status atual no SystemStatus
    systemStatus->automotiveSystem.canConnected = OBD2.connected();
    systemStatus->automotiveSystem.canBaud     = OBD2.currentBaud();
    systemStatus->automotiveSystem.canExtended = OBD2.isExtended() ? 1 : 0;

    if (systemStatus->automotiveSystem.canConnected) {
      uint32_t now = millis();

      if (now - lastFast >= 30)   { lastFast = now;  collectFast(systemStatus);   }
      if (now - lastMed  >= 600)  { lastMed  = now;  collectMedium(systemStatus); }
      if (now - lastSlow >= 1000) { lastSlow = now;  collectSlow(systemStatus);   }
      if (now - lastLog  >= 1000) { lastLog  = now;  debugDump(systemStatus);     }
    } else {
      resetPIDs(systemStatus);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
