/**
 * CAN OBD2 Monitor com detecção automática de protocolo
 * - Zera PIDs ao perder conexão
 * - Redetecção robusta (mutex + cooldown) sem double-init do TWAI
 * - Evita crashes por begin()/end() concorrentes
 */
// #include <Arduino.h>
// #include <esp32_obd2.h>
// #include <esp32_can.h>
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
// 	#define CAN_TASK_DEBUG_PRINT(...)	{ Serial.print(__VA_ARGS__); }
// 	#define CAN_TASK_DEBUG_PRINTLN(...)	{ Serial.println(__VA_ARGS__); }
// 	#define CAN_TASK_DEBUG_PRINTF(...)	{ Serial.printf(__VA_ARGS__); }
// #else
// 	#define CAN_TASK_DEBUG_PRINT(...)	{}
// 	#define CAN_TASK_DEBUG_PRINTLN(...)	{}
// 	#define CAN_TASK_DEBUG_PRINTF(...)	{}
// #endif

// // =====================
// // Controle de tempo
// // =====================
// static uint32_t lastFast	= 0;
// static uint32_t lastMedium	= 0;
// static uint32_t lastSlow	= 0;
// static uint32_t lastLog		= 0;

// // =====================
// // Estado global
// // =====================
// static bool			g_connected			= false;
// static bool			g_detectInProgress	= false;
// static uint32_t		g_lastDetectMs		= 0;
// static uint32_t		g_detectBackoffMs	= 1000;	// backoff inicial 1s
// static uint8_t		g_failCount			= 0;

// // Serializa qualquer begin()/end() da pilha OBD/CAN
// static SemaphoreHandle_t g_canStackMutex = nullptr;

// // =====================
// // Util: reset nos PIDs (zera tudo)
// // =====================
// static void canMonitor_reset(SystemStatus *s) {
// 	s->automotiveSystem.canMonitor.engineRPM = 0;
// 	s->automotiveSystem.canMonitor.vehicleSpeed = 0;
// 	s->automotiveSystem.canMonitor.fuelLevel = 0;
// 	s->automotiveSystem.canMonitor.engineCoolantTemperature = 0;
// 	s->automotiveSystem.canMonitor.intakeAirTemperature = 0;
// 	s->automotiveSystem.canMonitor.intakeManifoldPressure = 0;
// 	s->automotiveSystem.canMonitor.throttlePosition = 0;
// 	s->automotiveSystem.canMonitor.ignitionTimingAdvance = 0;
// 	s->automotiveSystem.canMonitor.calculatedLoadValue = 0;
// 	s->automotiveSystem.canMonitor.massAirFlowRate = 0;
// }

// // =====================
// // Safe PID Read (com validação de NaN)
// // =====================
// static inline bool safePid(float &out, uint8_t pid) {
// 	float v = OBD2.pidRead(pid);
// 	if (isnan(v)) return false;
// 	out = v;
// 	return true;
// }

// // =====================
// // Coletas
// // =====================
// static void canMonitor_collectFastPIDs(SystemStatus *systemStatus) {
// 	float val;
// 	if (safePid(val, ENGINE_RPM))
// 		systemStatus->automotiveSystem.canMonitor.engineRPM = (int)val;
// 	if (safePid(val, VEHICLE_SPEED))
// 		systemStatus->automotiveSystem.canMonitor.vehicleSpeed = (int)val;
// }

// static void canMonitor_collectMediumPIDs(SystemStatus *systemStatus) {
// 	float val;
// 	if (safePid(val, FUEL_TANK_LEVEL_INPUT))
// 		systemStatus->automotiveSystem.canMonitor.fuelLevel = (int)val;
// 	if (safePid(val, ENGINE_COOLANT_TEMPERATURE))
// 		systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature = (int)val;
// 	if (safePid(val, AIR_INTAKE_TEMPERATURE))
// 		systemStatus->automotiveSystem.canMonitor.intakeAirTemperature = (int)val;
// 	if (safePid(val, INTAKE_MANIFOLD_ABSOLUTE_PRESSURE))
// 		systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure = (int)val;
// }

// static void canMonitor_collectSlowPIDs(SystemStatus *systemStatus) {
// 	float val;
// 	if (safePid(val, THROTTLE_POSITION))
// 		systemStatus->automotiveSystem.canMonitor.throttlePosition = (int)val;
// 	if (safePid(val, TIMING_ADVANCE))
// 		systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance = (int)val;
// 	if (safePid(val, CALCULATED_ENGINE_LOAD))
// 		systemStatus->automotiveSystem.canMonitor.calculatedLoadValue = val;
// 	if (safePid(val, MAF_AIR_FLOW_RATE))
// 		systemStatus->automotiveSystem.canMonitor.massAirFlowRate = val;
// }

// // =====================
// // Debug
// // =====================
// static void canMonitor_debug(SystemStatus *systemStatus) {
// 	CAN_TASK_DEBUG_PRINTLN("====== OBD MONITOR ======");
// 	CAN_TASK_DEBUG_PRINTF("RPM: %ld\n", systemStatus->automotiveSystem.canMonitor.engineRPM);
// 	CAN_TASK_DEBUG_PRINTF("Velocidade: %d km/h\n", systemStatus->automotiveSystem.canMonitor.vehicleSpeed);
// 	CAN_TASK_DEBUG_PRINTF("Combustível: %d\n", systemStatus->automotiveSystem.canMonitor.fuelLevel);
// 	CAN_TASK_DEBUG_PRINTF("Temp. Motor: %d °C\n", systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature);
// 	CAN_TASK_DEBUG_PRINTF("Temp. Ar: %d °C\n", systemStatus->automotiveSystem.canMonitor.intakeAirTemperature);
// 	CAN_TASK_DEBUG_PRINTF("Pressão Coletor: %d kPa\n", systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure);
// 	CAN_TASK_DEBUG_PRINTF("Posição Acelerador: %d %%\n", systemStatus->automotiveSystem.canMonitor.throttlePosition);
// 	CAN_TASK_DEBUG_PRINTF("Avanço Ignição: %d °\n", systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance);
// 	CAN_TASK_DEBUG_PRINTF("Carga Calculada: %.2f\n", systemStatus->automotiveSystem.canMonitor.calculatedLoadValue);
// 	CAN_TASK_DEBUG_PRINTF("MAF: %.2f g/s\n", systemStatus->automotiveSystem.canMonitor.massAirFlowRate);
// 	CAN_TASK_DEBUG_PRINTLN("==========================");
// }

// // =====================
// // Restart limpo da stack OBD/CAN
// // - Serializa com mutex
// // - OBD2.end() -> CAN0.disable() -> delay -> set pins/baud -> OBD2.begin()
// // =====================
// static bool obd_stack_restart(uint32_t baud) {
// 	if (!g_canStackMutex) g_canStackMutex = xSemaphoreCreateMutex();
// 	xSemaphoreTake(g_canStackMutex, portMAX_DELAY);

// 	// Fecha qualquer sessão anterior
// 	OBD2.end();
// 	// Garante que o driver TWAI foi parado pela camada de baixo
// 	CAN0.disable();
// 	vTaskDelay(pdMS_TO_TICKS(200)); // cooldown mínimo pro driver finalizar

// 	// Configura hardware base para a lib usar
// 	CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX, (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);
// 	CAN0.set_baudrate(baud);

// 	// Sobe OBD2 novamente
// 	OBD2.begin();
// 	OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);

// 	xSemaphoreGive(g_canStackMutex);
// 	return true;
// }

// // =====================
// // Scanner de protocolos (baud)
// // - Tenta 500k e 250k
// // - Confirma com duas leituras válidas
// // - Respeita cooldown/backoff para evitar tempestade de inits
// // =====================
// static bool can_detectProtocol(SystemStatus *systemStatus) {
// 	const uint32_t baudList[] = { CAN_BPS_500K, CAN_BPS_250K };

// 	if (g_detectInProgress) return false;

// 	const uint32_t now = millis();
// 	if ((now - g_lastDetectMs) < g_detectBackoffMs) {
// 		return false; // ainda no cooldown
// 	}

// 	g_detectInProgress = true;
// 	bool detected = false;

// 	for (int i = 0; i < 2 && !detected; i++) {
// 		const uint32_t baud = baudList[i];
// 		CAN_TASK_DEBUG_PRINTF("[CAN]\tTestando baud %lu...\n", baud);

// 		obd_stack_restart(baud);

// 		// Confirma com duas leituras (PIDs suportados e/ou RPM)
// 		bool ok1 = false, ok2 = false;

// 		float v = OBD2.pidRead(PIDS_SUPPORT_01_20);
// 		ok1 = !isnan(v);
// 		if (!ok1) {
// 			float rpm = OBD2.pidRead(ENGINE_RPM);
// 			ok1 = !isnan(rpm);
// 		}

// 		if (ok1) {
// 			float rpm2 = OBD2.pidRead(ENGINE_RPM);
// 			ok2 = !isnan(rpm2);
// 		}

// 		if (ok1 && ok2) {
// 			systemStatus->automotiveSystem.canBaud = baud;
// 			systemStatus->automotiveSystem.canConnected = 1;
// 			systemStatus->automotiveSystem.canExtended = 0; // OBD-II por CAN geralmente 11-bit
// 			g_connected = true;
// 			g_failCount = 0;

// 			CAN_TASK_DEBUG_PRINTF("[CAN]\tECU detectada @ %lu bps\n", baud);
// 			detected = true;
// 			break;
// 		}
// 	}

// 	if (!detected) {
// 		// nada detectado: desliga limpo e aumenta backoff
// 		OBD2.end();
// 		CAN0.disable();
// 		g_connected = false;
// 		systemStatus->automotiveSystem.canConnected = 0;

// 		g_detectBackoffMs = min<uint32_t>(g_detectBackoffMs << 1, 8000); // até 8s
// 		CAN_TASK_DEBUG_PRINTLN("[CAN]\tNenhum protocolo detectado");
// 	} else {
// 		// sucesso: reduz backoff para resposta rápida na próxima perda
// 		g_detectBackoffMs = 1000;
// 	}
// 	g_lastDetectMs = millis();
// 	g_detectInProgress = false;

// 	return detected;
// }

// // =====================
// // Task Principal
// // =====================
// void canTask_run(void* pvParameters) {
// 	SystemStatus *systemStatus = (SystemStatus *)pvParameters;

// 	CAN_TASK_DEBUG_PRINTLN("------------------------");
// 	CAN_TASK_DEBUG_PRINTLN("    CAN OBD2 MONITOR TASK");
// 	CAN_TASK_DEBUG_PRINTLN("------------------------");

// 	delay(TIME_TO_INIT_TASK_CAN);

// 	// Estado inicial
// 	g_connected = false;
// 	g_detectInProgress = false;
// 	g_lastDetectMs = 0;
// 	g_detectBackoffMs = 1000;
// 	systemStatus->automotiveSystem.canConnected = 0;
// 	systemStatus->automotiveSystem.canBaud = 0;
// 	systemStatus->automotiveSystem.canExtended = 0;
// 	canMonitor_reset(systemStatus);

// 	// Mutex global
// 	if (!g_canStackMutex) g_canStackMutex = xSemaphoreCreateMutex();

// 	// Primeira detecção (sem pressa)
// 	can_detectProtocol(systemStatus);

// 	unsigned long canTimer = millis();

// 	for (;;) {
// 		task_checkUsedMem(TASK_NAME_CAN, &canTimer);

// 		if (g_connected && systemStatus->automotiveSystem.canConnected) {
// 			// Coletas cronometradas
// 			uint32_t now = millis();

// 			if (now - lastFast >= 30)	{ lastFast = now;		canMonitor_collectFastPIDs(systemStatus); }
// 			if (now - lastMedium >= 600){ lastMedium = now;		canMonitor_collectMediumPIDs(systemStatus); }
// 			if (now - lastSlow >= 1000)	{ lastSlow = now;		canMonitor_collectSlowPIDs(systemStatus); }
// 			if (now - lastLog >= 1000) {
// 				lastLog = now;
// 				canMonitor_debug(systemStatus);
// 			}

// 			// Heartbeat: se falhar várias leituras seguidas, considera desconectado
// 			float tmp;
// 			if (!safePid(tmp, ENGINE_RPM)) {
// 				if (++g_failCount > 5) {
// 					CAN_TASK_DEBUG_PRINTLN("[CAN]\tConexão perdida, zerando valores e iniciando redetecção...");
// 					g_connected = false;
// 					systemStatus->automotiveSystem.canConnected = 0;
// 					canMonitor_reset(systemStatus);

// 					// Fecha limpo e inicia cooldown curto antes de detectar
// 					OBD2.end();
// 					CAN0.disable();
// 					g_lastDetectMs = millis();
// 					g_detectBackoffMs = 1000; // começa com 1s
// 					g_failCount = 0;
// 				}
// 			} else {
// 				g_failCount = 0;
// 			}

// 			// Se foi marcado desconectado acima, não tenta detectar imediatamente aqui;
// 			// deixa para o ramo "else" abaixo, que respeita cooldown.
// 		} else {
// 			// Desconectado: zera por garantia e tenta detectar respeitando backoff
// 			canMonitor_reset(systemStatus);
// 			can_detectProtocol(systemStatus);
// 		}

// 		vTaskDelay(pdMS_TO_TICKS(10));
// 	}
// }













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

#include <Arduino.h>
#include <esp32_obd2.h>   // Alto nível (11-bit)
#include <esp32_can.h>    // CAN (TWAI) baixo nível p/ 29-bit
#include <can_common.h>
#include "Hardware/hardware.h"
#include "taskManager/Tasks/canTask.hpp"
#include "taskManager/taskManager.h"
#include "freertos/semphr.h"

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
// Controle de tempo
// =====================
static uint32_t lastFast   = 0;
static uint32_t lastMedium = 0;
static uint32_t lastSlow   = 0;
static uint32_t lastLog    = 0;

// =====================
// Estado global
// =====================
enum CanAddrMode { ADDR_11BIT = 0, ADDR_29BIT = 1 };

static bool         g_connected        = false;
static bool         g_detectInProgress = false;
static uint32_t     g_lastDetectMs     = 0;
static uint32_t     g_detectBackoffMs  = 1000; // 1s -> 2s -> 4s -> 8s (máx)
static uint8_t      g_failCount        = 0;

static uint32_t     g_currBaud         = 0;
static CanAddrMode  g_currAddrMode     = ADDR_11BIT;

// Mutex para serializar begin()/end()
static SemaphoreHandle_t g_canStackMutex = nullptr;

// =====================
// Reset PIDs
// =====================
static void canMonitor_reset(SystemStatus *s) {
  s->automotiveSystem.canMonitor.engineRPM                = 0;
  s->automotiveSystem.canMonitor.vehicleSpeed             = 0;
  s->automotiveSystem.canMonitor.fuelLevel                = 0;
  s->automotiveSystem.canMonitor.engineCoolantTemperature = 0;
  s->automotiveSystem.canMonitor.intakeAirTemperature     = 0;
  s->automotiveSystem.canMonitor.intakeManifoldPressure   = 0;
  s->automotiveSystem.canMonitor.throttlePosition         = 0;
  s->automotiveSystem.canMonitor.ignitionTimingAdvance    = 0;
  s->automotiveSystem.canMonitor.calculatedLoadValue      = 0;
  s->automotiveSystem.canMonitor.massAirFlowRate          = 0;
}

// =====================
// Util | fecha tudo
// =====================
static void close_all_stacks() {
  // Fecha OBD2 (se estiver rodando)
  OBD2.end();
  // Fecha driver CAN que a esp32_can abriu
  CAN0.disable();
}

// =====================
// OBD2 (11-bit) stack restart
// =====================
static bool start_obd2_11bit(uint32_t baud) {
  if (!g_canStackMutex) g_canStackMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(g_canStackMutex, portMAX_DELAY);

  close_all_stacks();
  vTaskDelay(pdMS_TO_TICKS(200)); // cooldown

  // Pinos e baud para a camada OBD usar por baixo
  CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX, (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);
  CAN0.set_baudrate(baud);

  // Inicia biblioteca OBD2 (11-bit)
  OBD2.begin();
  OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);

  xSemaphoreGive(g_canStackMutex);
  return true;
}

// =====================
// CAN manual (29-bit) stack restart
// =====================
static bool start_can_29bit(uint32_t baud) {
  if (!g_canStackMutex) g_canStackMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(g_canStackMutex, portMAX_DELAY);

  close_all_stacks();
  vTaskDelay(pdMS_TO_TICKS(200)); // cooldown

  // Driver CAN direto (esp32_can cuida de instalar TWAI)
  CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX, (gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);
  CAN0.set_baudrate(baud);

  xSemaphoreGive(g_canStackMutex);
  return true;
}

// =====================
// SAFE PID via OBD2 (11-bit)
// =====================
static inline bool safePid11(float &out, uint8_t pid) {
  float v = OBD2.pidRead(pid);
  if (isnan(v)) return false;
  out = v;
  return true;
}

// =====================
// Envio/Recebimento OBD-II 29-bit (manual)
// - Request funcional 29-bit: 0x18DB33F1
// - Respostas típicas: 0x18DAF1xx (qualquer ECU xx)
// - ISO-TP Single Frame: data[0] = 0x02 (len=2), data[1]=serviço (0x01), data[2]=PID
// =====================
static bool send_pid_29(uint8_t service, uint8_t pid) {
  CAN_FRAME tx;
  tx.id       = 0x18DB33F1;   // Functional request 29-bit
  tx.extended = true;
  tx.rtr      = 0;
  tx.length   = 8;
  tx.data.byte[0] = 0x02;     // 2 bytes de payload (01, PID)
  tx.data.byte[1] = service;  // 0x01 = Current Data
  tx.data.byte[2] = pid;
  tx.data.byte[3] = 0x00;
  tx.data.byte[4] = 0x00;
  tx.data.byte[5] = 0x00;
  tx.data.byte[6] = 0x00;
  tx.data.byte[7] = 0x00;

  return CAN0.sendFrame(tx);
}

static bool recv_pid_29(uint8_t expected_service_resp, uint8_t pid, uint8_t *outData, uint32_t timeout_ms) {
  uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    if (CAN0.rx_avail()) {
      CAN_FRAME rx;
      if (CAN0.get_rx_buff(rx)) {
        if (!rx.extended) continue; // precisamos de 29-bit

        // Verificação básica do payload ISO-TP SF e do serviço/PIB
        // Para resposta, data[0] = 0x.. (len), data[1] = service|0x40, data[2] = pid ecoado
        if (rx.length >= 3) {
          uint8_t pci_len = rx.data.byte[0] & 0x0F; // nibble de len
          uint8_t svc     = rx.data.byte[1];
          uint8_t pid_rx  = rx.data.byte[2];

          if (svc == expected_service_resp && pid_rx == pid && pci_len >= 2) {
            // Copia o restante
            for (int i = 0; i < rx.length; i++) outData[i] = rx.data.byte[i];
            return true;
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

// Decodificador básico de alguns PIDs (para 29-bit manual)
// NOTA: mesma matemática do OBD2
static bool pidRead29(uint8_t pid, float &outVal) {
  const uint8_t SERVICE = 0x01;
  const uint8_t SERVICE_RESP = 0x41;

  if (!send_pid_29(SERVICE, pid)) return false;

  uint8_t buf[8] = {0};
  if (!recv_pid_29(SERVICE_RESP, pid, buf, 200)) return false;

  // buf[0] = PCI (len), buf[1] = 0x41, buf[2] = pid
  uint8_t A = buf[3];
  uint8_t B = (buf[4]);
  // uint8_t C = buf[5];
  // uint8_t D = buf[6];

  switch (pid) {
    case ENGINE_RPM: {               // 0x0C: RPM = (256*A + B)/4
      outVal = (256.0f * A + B) / 4.0f;
      return true;
    }
    case VEHICLE_SPEED: {            // 0x0D: km/h
      outVal = A;
      return true;
    }
    case ENGINE_COOLANT_TEMPERATURE: // 0x05: °C = A - 40
    case AIR_INTAKE_TEMPERATURE: {   // 0x0F: °C = A - 40
      outVal = (int)A - 40;
      return true;
    }
    case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE: { // 0x0B: kPa = A
      outVal = A;
      return true;
    }
    case THROTTLE_POSITION:          // 0x11: % = A * 100 / 255
    case CALCULATED_ENGINE_LOAD: {   // 0x04: % = A * 100 / 255
      outVal = (A * 100.0f) / 255.0f;
      return true;
    }
    case MAF_AIR_FLOW_RATE: {        // 0x10: g/s = (256*A + B)/100
      outVal = (256.0f * A + B) / 100.0f;
      return true;
    }
    case TIMING_ADVANCE: {           // 0x0E: ° = (A/2) - 64
      outVal = (A / 2.0f) - 64.0f;
      return true;
    }
    case FUEL_TANK_LEVEL_INPUT: {    // 0x2F: % = A * 100 / 255
      outVal = (A * 100.0f) / 255.0f;
      return true;
    }
    case PIDS_SUPPORT_01_20: {       // 0x00: apenas checagem (qualquer retorno serve)
      outVal = 1.0f;
      return true;
    }
  }
  return false; // PID não mapeado aqui
}

// Wrapper de leitura que escolhe 11b/29b automaticamente
static inline bool safePid(float &out, uint8_t pid) {
  if (g_currAddrMode == ADDR_11BIT) {
    return safePid11(out, pid);
  } else {
    return pidRead29(pid, out);
  }
}

// =====================
// Coletas
// =====================
static void canMonitor_collectFastPIDs(SystemStatus *systemStatus) {
  float v;
  if (safePid(v, ENGINE_RPM))    systemStatus->automotiveSystem.canMonitor.engineRPM = (int)v;
  if (safePid(v, VEHICLE_SPEED)) systemStatus->automotiveSystem.canMonitor.vehicleSpeed = (int)v;
}

static void canMonitor_collectMediumPIDs(SystemStatus *systemStatus) {
  float v;
  if (safePid(v, FUEL_TANK_LEVEL_INPUT))           systemStatus->automotiveSystem.canMonitor.fuelLevel = (int)v;
  if (safePid(v, ENGINE_COOLANT_TEMPERATURE))      systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature = (int)v;
  if (safePid(v, AIR_INTAKE_TEMPERATURE))          systemStatus->automotiveSystem.canMonitor.intakeAirTemperature = (int)v;
  if (safePid(v, INTAKE_MANIFOLD_ABSOLUTE_PRESSURE)) systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure = (int)v;
}

static void canMonitor_collectSlowPIDs(SystemStatus *systemStatus) {
  float v;
  if (safePid(v, THROTTLE_POSITION))       systemStatus->automotiveSystem.canMonitor.throttlePosition = (int)v;
  if (safePid(v, TIMING_ADVANCE))          systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance = (int)v;
  if (safePid(v, CALCULATED_ENGINE_LOAD))  systemStatus->automotiveSystem.canMonitor.calculatedLoadValue = v;
  if (safePid(v, MAF_AIR_FLOW_RATE))       systemStatus->automotiveSystem.canMonitor.massAirFlowRate = v;
}

// =====================
// Debug
// =====================
static void canMonitor_debug(SystemStatus *systemStatus) {
  CAN_TASK_DEBUG_PRINTLN("====== OBD MONITOR ======");
  CAN_TASK_DEBUG_PRINTF("RPM: %ld\n", systemStatus->automotiveSystem.canMonitor.engineRPM);
  CAN_TASK_DEBUG_PRINTF("Velocidade: %d km/h\n", systemStatus->automotiveSystem.canMonitor.vehicleSpeed);
  CAN_TASK_DEBUG_PRINTF("Combustível: %d\n", systemStatus->automotiveSystem.canMonitor.fuelLevel);
  CAN_TASK_DEBUG_PRINTF("Temp. Motor: %d °C\n", systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature);
  CAN_TASK_DEBUG_PRINTF("Temp. Ar: %d °C\n", systemStatus->automotiveSystem.canMonitor.intakeAirTemperature);
  CAN_TASK_DEBUG_PRINTF("Pressão Coletor: %d kPa\n", systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure);
  CAN_TASK_DEBUG_PRINTF("Posição Acelerador: %d %%\n", systemStatus->automotiveSystem.canMonitor.throttlePosition);
  CAN_TASK_DEBUG_PRINTF("Avanço Ignição: %d °\n", systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance);
  CAN_TASK_DEBUG_PRINTF("Carga Calculada: %.2f\n", systemStatus->automotiveSystem.canMonitor.calculatedLoadValue);
  CAN_TASK_DEBUG_PRINTF("MAF: %.2f g/s\n", systemStatus->automotiveSystem.canMonitor.massAirFlowRate);
  CAN_TASK_DEBUG_PRINTLN("==========================");
}

// =====================
// Detecção (baud × addr mode)
// Ordem: 11b/500k → 11b/250k → 29b/500k → 29b/250k
// Confirma com: PIDS_SUPPORT_01_20 OU RPM
// =====================
static bool detect_protocol(SystemStatus *sys) {
  if (g_detectInProgress) return false;

  const struct {
    uint32_t    baud;
    CanAddrMode mode;
    const char* label;
  } tries[] = {
    { CAN_BPS_500K, ADDR_11BIT, "11b/500k" },
    { CAN_BPS_250K, ADDR_11BIT, "11b/250k" },
    { CAN_BPS_500K, ADDR_29BIT, "29b/500k" },
    { CAN_BPS_250K, ADDR_29BIT, "29b/250k" },
  };

  const uint32_t now = millis();
  if ((now - g_lastDetectMs) < g_detectBackoffMs) return false;

  g_detectInProgress = true;
  bool detected = false;

  for (auto &t : tries) {
    CAN_TASK_DEBUG_PRINTF("[CAN]\tTestando %s...\n", t.label);

    bool started = (t.mode == ADDR_11BIT) ? start_obd2_11bit(t.baud)
                                          : start_can_29bit(t.baud);
    if (!started) continue;

    // dá tempo pro barramento estabilizar
    vTaskDelay(pdMS_TO_TICKS(250));

    bool ok1=false, ok2=false;
    float v;

    // Teste de suporte
    if (t.mode == ADDR_11BIT) {
      ok1 = safePid11(v, PIDS_SUPPORT_01_20);
      if (!ok1) ok1 = safePid11(v, ENGINE_RPM);
    } else {
      ok1 = pidRead29(PIDS_SUPPORT_01_20, v);
      if (!ok1) ok1 = pidRead29(ENGINE_RPM, v);
    }

    if (ok1) {
      // segunda leitura de RPM pra confirmar
      if (t.mode == ADDR_11BIT) ok2 = safePid11(v, ENGINE_RPM);
      else                      ok2 = pidRead29(ENGINE_RPM, v);
    }

    if (ok1 && ok2) {
      g_currBaud     = t.baud;
      g_currAddrMode = t.mode;

      sys->automotiveSystem.canBaud      = t.baud;
      sys->automotiveSystem.canExtended  = (t.mode == ADDR_29BIT) ? 1 : 0;
      sys->automotiveSystem.canConnected = 1;

      g_connected  = true;
      g_failCount  = 0;

      CAN_TASK_DEBUG_PRINTF("[CAN]\tECU detectada @ %s\n", t.label);
      detected = true;
      break;
    }
  }

  if (!detected) {
    close_all_stacks();
    g_connected = false;
    sys->automotiveSystem.canConnected = 0;

    g_detectBackoffMs = min<uint32_t>(g_detectBackoffMs << 1, 8000);
    CAN_TASK_DEBUG_PRINTLN("[CAN]\tNenhum protocolo detectado");
  } else {
    g_detectBackoffMs = 1000;
  }

  g_lastDetectMs = millis();
  g_detectInProgress = false;
  return detected;
}

// =====================
// Task Principal
// =====================
void canTask_run(void* pvParameters) {
  SystemStatus *sys = (SystemStatus *)pvParameters;

  CAN_TASK_DEBUG_PRINTLN("------------------------");
  CAN_TASK_DEBUG_PRINTLN("    CAN OBD2 MONITOR TASK");
  CAN_TASK_DEBUG_PRINTLN("------------------------");

  delay(TIME_TO_INIT_TASK_CAN);

  // Estado inicial
  g_connected = false;
  g_detectInProgress = false;
  g_lastDetectMs = 0;
  g_detectBackoffMs = 1000;
  sys->automotiveSystem.canConnected = 0;
  sys->automotiveSystem.canBaud = 0;
  sys->automotiveSystem.canExtended = 0;
  canMonitor_reset(sys);

  if (!g_canStackMutex) g_canStackMutex = xSemaphoreCreateMutex();

  // Primeira detecção
  detect_protocol(sys);

  unsigned long canTimer = millis();

  for (;;) {
    task_checkUsedMem(TASK_NAME_CAN, &canTimer);

    if (g_connected && sys->automotiveSystem.canConnected) {
      uint32_t now = millis();

      if (now - lastFast   >= 30)   { lastFast   = now; canMonitor_collectFastPIDs(sys); }
      if (now - lastMedium >= 600)  { lastMedium = now; canMonitor_collectMediumPIDs(sys); }
      if (now - lastSlow   >= 1000) { lastSlow   = now; canMonitor_collectSlowPIDs(sys); }
      if (now - lastLog    >= 1000) { lastLog    = now; canMonitor_debug(sys); }

      // Heartbeat: duas verificações para “não confundir” ruído com perda real
      float tmp;
      bool okHeartbeat = safePid(tmp, ENGINE_RPM);
      if (!okHeartbeat) {
        float dummy;
        okHeartbeat = safePid(dummy, PIDS_SUPPORT_01_20);
      }

      if (!okHeartbeat) {
        if (++g_failCount > 5) {
          CAN_TASK_DEBUG_PRINTLN("[CAN]\tConexão perdida, zerando e redetectando...");
          g_connected = false;
          sys->automotiveSystem.canConnected = 0;
          canMonitor_reset(sys);

          close_all_stacks();
          g_lastDetectMs = millis();
          g_detectBackoffMs = 1000;
          g_failCount = 0;
        }
      } else {
        g_failCount = 0;
      }

    } else {
      // Não conectado: zera e tenta (respeitando cooldown)
      canMonitor_reset(sys);
      detect_protocol(sys);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


























// /**
//  * CAN OBD2 Monitor com detecção automática de protocolo
//  * - Usa ESP32CAN::init() corretamente
//  * - Confirma protocolo enviando queries de PID
//  * - Zera PIDs ao perder conexão
//  * - Redetecção robusta com mutex + cooldown
//  */
// #include <Arduino.h>
// #include <esp32_obd2.h>
// #include <esp32_can.h>
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
//   #define CAN_TASK_DEBUG_PRINT(...)   { Serial.print(__VA_ARGS__); }
//   #define CAN_TASK_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
//   #define CAN_TASK_DEBUG_PRINTF(...)  { Serial.printf(__VA_ARGS__); }
// #else
//   #define CAN_TASK_DEBUG_PRINT(...)
//   #define CAN_TASK_DEBUG_PRINTLN(...)
//   #define CAN_TASK_DEBUG_PRINTF(...)
// #endif

// // =====================
// // Controle de tempo
// // =====================
// static uint32_t lastFast   = 0;
// static uint32_t lastMedium = 0;
// static uint32_t lastSlow   = 0;
// static uint32_t lastLog    = 0;

// // =====================
// // Estado global
// // =====================
// static bool         g_connected        = false;
// static bool         g_detectInProgress = false;
// static uint32_t     g_lastDetectMs     = 0;
// static uint32_t     g_detectBackoffMs  = 1000; // backoff inicial 1s
// static uint8_t      g_failCount        = 0;

// // Mutex global para serializar init/disable
// static SemaphoreHandle_t g_canStackMutex = nullptr;

// // =====================
// // Reset de PIDs
// // =====================
// static void canMonitor_reset(SystemStatus *s) {
//   s->automotiveSystem.canMonitor.engineRPM                = 0;
//   s->automotiveSystem.canMonitor.vehicleSpeed             = 0;
//   s->automotiveSystem.canMonitor.fuelLevel                = 0;
//   s->automotiveSystem.canMonitor.engineCoolantTemperature = 0;
//   s->automotiveSystem.canMonitor.intakeAirTemperature     = 0;
//   s->automotiveSystem.canMonitor.intakeManifoldPressure   = 0;
//   s->automotiveSystem.canMonitor.throttlePosition         = 0;
//   s->automotiveSystem.canMonitor.ignitionTimingAdvance    = 0;
//   s->automotiveSystem.canMonitor.calculatedLoadValue      = 0;
//   s->automotiveSystem.canMonitor.massAirFlowRate          = 0;
// }

// // =====================
// // Safe PID Read
// // =====================
// static inline bool safePid(float &out, uint8_t pid) {
//   float v = OBD2.pidRead(pid);
//   if (isnan(v)) return false;
//   out = v;
//   return true;
// }

// // =====================
// // Coletas
// // =====================
// static void canMonitor_collectFastPIDs(SystemStatus *systemStatus) {
//   float val;
//   if (safePid(val, ENGINE_RPM))
//     systemStatus->automotiveSystem.canMonitor.engineRPM = (int)val;
//   if (safePid(val, VEHICLE_SPEED))
//     systemStatus->automotiveSystem.canMonitor.vehicleSpeed = (int)val;
// }

// static void canMonitor_collectMediumPIDs(SystemStatus *systemStatus) {
//   float val;
//   if (safePid(val, FUEL_TANK_LEVEL_INPUT))
//     systemStatus->automotiveSystem.canMonitor.fuelLevel = (int)val;
//   if (safePid(val, ENGINE_COOLANT_TEMPERATURE))
//     systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature = (int)val;
//   if (safePid(val, AIR_INTAKE_TEMPERATURE))
//     systemStatus->automotiveSystem.canMonitor.intakeAirTemperature = (int)val;
//   if (safePid(val, INTAKE_MANIFOLD_ABSOLUTE_PRESSURE))
//     systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure = (int)val;
// }

// static void canMonitor_collectSlowPIDs(SystemStatus *systemStatus) {
//   float val;
//   if (safePid(val, THROTTLE_POSITION))
//     systemStatus->automotiveSystem.canMonitor.throttlePosition = (int)val;
//   if (safePid(val, TIMING_ADVANCE))
//     systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance = (int)val;
//   if (safePid(val, CALCULATED_ENGINE_LOAD))
//     systemStatus->automotiveSystem.canMonitor.calculatedLoadValue = val;
//   if (safePid(val, MAF_AIR_FLOW_RATE))
//     systemStatus->automotiveSystem.canMonitor.massAirFlowRate = val;
// }

// // =====================
// // Debug
// // =====================
// static void canMonitor_debug(SystemStatus *systemStatus) {
//   CAN_TASK_DEBUG_PRINTLN("====== OBD MONITOR ======");
//   CAN_TASK_DEBUG_PRINTF("RPM: %ld\n", systemStatus->automotiveSystem.canMonitor.engineRPM);
//   CAN_TASK_DEBUG_PRINTF("Velocidade: %d km/h\n", systemStatus->automotiveSystem.canMonitor.vehicleSpeed);
//   CAN_TASK_DEBUG_PRINTF("Combustível: %d\n", systemStatus->automotiveSystem.canMonitor.fuelLevel);
//   CAN_TASK_DEBUG_PRINTF("Temp. Motor: %d °C\n", systemStatus->automotiveSystem.canMonitor.engineCoolantTemperature);
//   CAN_TASK_DEBUG_PRINTF("Temp. Ar: %d °C\n", systemStatus->automotiveSystem.canMonitor.intakeAirTemperature);
//   CAN_TASK_DEBUG_PRINTF("Pressão Coletor: %d kPa\n", systemStatus->automotiveSystem.canMonitor.intakeManifoldPressure);
//   CAN_TASK_DEBUG_PRINTF("Posição Acelerador: %d %%\n", systemStatus->automotiveSystem.canMonitor.throttlePosition);
//   CAN_TASK_DEBUG_PRINTF("Avanço Ignição: %d °\n", systemStatus->automotiveSystem.canMonitor.ignitionTimingAdvance);
//   CAN_TASK_DEBUG_PRINTF("Carga Calculada: %.2f\n", systemStatus->automotiveSystem.canMonitor.calculatedLoadValue);
//   CAN_TASK_DEBUG_PRINTF("MAF: %.2f g/s\n", systemStatus->automotiveSystem.canMonitor.massAirFlowRate);
//   CAN_TASK_DEBUG_PRINTLN("==========================");
// }

// // =====================
// // Restart limpo da stack
// // =====================
// static bool obd_stack_restart(uint32_t baud) {
// 	if (!g_canStackMutex) g_canStackMutex = xSemaphoreCreateMutex();
// 	xSemaphoreTake(g_canStackMutex, portMAX_DELAY);

// 	// Fecha sessão anterior
// 	OBD2.end();
// 	CAN0.disable();
// 	vTaskDelay(pdMS_TO_TICKS(800)); // mais tempo para tasks internas morrerem

// 	CAN0.setCANPins((gpio_num_t)PIN_PCI_ATHENAS_CAN_RX,
// 					(gpio_num_t)PIN_PCI_ATHENAS_CAN_TX);

// 	// inicia o CAN de novo com segurança
// 	if (CAN0.init(baud) == 0) {
// 		CAN_TASK_DEBUG_PRINTF("[CAN]\tErro ao iniciar @ %lu bps\n", baud);
// 		xSemaphoreGive(g_canStackMutex);
// 		return false;
// 	}

// 	OBD2.begin();
// 	OBD2.setTimeout(TIME_TO_REQUEST_CAN_PID);


// 	xSemaphoreGive(g_canStackMutex);
// 	return true;
// }

// // =====================
// // Scanner de protocolos
// // =====================
// static bool can_detectProtocol(SystemStatus *systemStatus) {
//   const uint32_t baudList[] = { CAN_BPS_500K, CAN_BPS_250K };

//   if (g_detectInProgress) return false;
//   if ((millis() - g_lastDetectMs) < g_detectBackoffMs) return false;

//   g_detectInProgress = true;
//   bool detected = false;

//   for (int i = 0; i < 2 && !detected; i++) {
//     const uint32_t baud = baudList[i];
//     CAN_TASK_DEBUG_PRINTF("[CAN]\tTestando baud %lu...\n", baud);

//     obd_stack_restart(baud);
//     delay(50); // tempo para estabilizar

//     bool ok1 = false, ok2 = false;
//     float v = OBD2.pidRead(PIDS_SUPPORT_01_20);
//     ok1 = !isnan(v);
//     if (!ok1) {
//       float rpm = OBD2.pidRead(ENGINE_RPM);
//       ok1 = !isnan(rpm);
//     }
//     if (ok1) {
//       float rpm2 = OBD2.pidRead(ENGINE_RPM);
//       ok2 = !isnan(rpm2);
//     }

//     if (ok1 && ok2) {
//       systemStatus->automotiveSystem.canBaud = baud;
//       systemStatus->automotiveSystem.canConnected = 1;
//       g_connected = true;
//       g_failCount = 0;
//       CAN_TASK_DEBUG_PRINTF("[CAN]\tECU detectada @ %lu bps\n", baud);
//       detected = true;
//       break;
//     }
//   }

//   if (!detected) {
//     OBD2.end();
//     CAN0.disable();
//     g_connected = false;
//     systemStatus->automotiveSystem.canConnected = 0;
//     g_detectBackoffMs = min<uint32_t>(g_detectBackoffMs << 1, 8000);
//     CAN_TASK_DEBUG_PRINTLN("[CAN]\tNenhum protocolo detectado");
//   } else {
//     g_detectBackoffMs = 1000;
//   }
//   g_lastDetectMs = millis();
//   g_detectInProgress = false;

//   return detected;
// }

// // =====================
// // Task Principal
// // =====================
// void canTask_run(void* pvParameters) {
//   SystemStatus *systemStatus = (SystemStatus *)pvParameters;

//   CAN_TASK_DEBUG_PRINTLN("------------------------");
//   CAN_TASK_DEBUG_PRINTLN("    CAN OBD2 MONITOR TASK");
//   CAN_TASK_DEBUG_PRINTLN("------------------------");

//   delay(TIME_TO_INIT_TASK_CAN);

//   g_connected = false;
//   g_detectInProgress = false;
//   g_lastDetectMs = 0;
//   g_detectBackoffMs = 1000;
//   systemStatus->automotiveSystem.canConnected = 0;
//   systemStatus->automotiveSystem.canBaud = 0;
//   canMonitor_reset(systemStatus);

//   if (!g_canStackMutex) g_canStackMutex = xSemaphoreCreateMutex();

//   // Primeira detecção
//   can_detectProtocol(systemStatus);

//   unsigned long canTimer = millis();

//   for (;;) {
//     task_checkUsedMem(TASK_NAME_CAN, &canTimer);

//     if (g_connected && systemStatus->automotiveSystem.canConnected) {
//       uint32_t now = millis();

//       if (now - lastFast   >= 30)   { lastFast   = now; canMonitor_collectFastPIDs(systemStatus); }
//       if (now - lastMedium >= 600)  { lastMedium = now; canMonitor_collectMediumPIDs(systemStatus); }
//       if (now - lastSlow   >= 1000) { lastSlow   = now; canMonitor_collectSlowPIDs(systemStatus); }
//       if (now - lastLog    >= 1000) { lastLog    = now; canMonitor_debug(systemStatus); }

//       float tmp;
//       if (!safePid(tmp, ENGINE_RPM)) {
//         if (++g_failCount > 5) {
//           CAN_TASK_DEBUG_PRINTLN("[CAN]\tConexão perdida, redetectando...");
//           g_connected = false;
//           systemStatus->automotiveSystem.canConnected = 0;
//           canMonitor_reset(systemStatus);
//           OBD2.end();
//           CAN0.disable();
//           g_lastDetectMs = millis();
//           g_detectBackoffMs = 1000;
//           g_failCount = 0;
//         }
//       } else {
//         g_failCount = 0;
//       }
//     } else {
//       canMonitor_reset(systemStatus);
//       can_detectProtocol(systemStatus);
//     }

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }
