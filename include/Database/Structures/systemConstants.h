#ifndef SYSTEM_CONSTANTS_H_
#define SYSTEM_CONSTANTS_H_

#define RST_QTTY_TRY_CONNECT_SERVER 3 /* Se passar desse contador, reseta por nao conseguir comunicar no servidor*/ 

#define CONST_YOUTUBE_CHANNEL_ID_MAX_LEN  32
#define CONST_YOUTUBE_API_KEY_MAX_LEN     64

#define CONST_SSID_MAX_LEN 30
#define CONST_PASS_MAX_LEN 30

#define CONST_TAGO_TOKEN_MAX_LEN 40
#define CONST_TAGO_PROFILE_ID_MAX_LEN 30

#define CONST_IP_LEN 16
#define CONST_PASSWORD_LEN 16

#define CONST_STR_MSG_STATUS 16

#define TIME_TO_INIT_TASK_WIFI 500
#define TIME_TO_INIT_TASK_WEB_SERVER 1500
#define TIME_TO_INIT_TASK_SERVERS 2000
#define TIME_TO_INIT_TASK_SENSORS 3200
#define TIME_TO_INIT_TASK_TIMER 500
#define TIME_TO_INIT_TASK_OUTPUT 2500
#define TIME_TO_INIT_TASK_DISPLAY 1000
#define TIME_TO_INIT_TASK_CAN 3000
#define TIME_TO_INIT_TASK_GYROSCOPE 3000


#define TIME_TO_REQUEST_CAN_PID 150

#define SENSOR_NO_RESPONSE_VALUE 000
#define VALUE_TO_RESET_TIMER_INTERVAL 99999999 // valor enorme para resetar o contador

#define GENERIC_BUFF_IRRIGAPLAY_ADDR_TOTAL 65 // ENDERECO AUTOMATION + MONITOR 
#define GENERIC_BUFF_AUTOMATION_ADDR_MAX 33 // utilizado nos enderecos de 1 ao 32.
#define GENERIC_BUFF_MONITOR_ADDR_MAX  (GENERIC_BUFF_IRRIGAPLAY_ADDR_TOTAL - GENERIC_BUFF_AUTOMATION_ADDR_MAX) // utilizado nos enderecos de 33 ao 64.

#define START_VALID_ADDR 1 // pega o primeiro valor de endereco valido
#define QTTY_ONBOARD_RELAYS 9
#define RELAY_OFFBOARD_START_ADDR QTTY_ONBOARD_RELAYS

#define TOTAL_AUTOMOTIVE_GAUGES 4

#define SENSOR_BOIA_ENABLED 0
#define SENSOR_BOIA_HAS_WATER 0
#define SENSOR_BOIA_STATUS_HAS_WATER true
#define SENSOR_BOIA_STATUS_NO_WATER false

#define MEMORY_BUFF_LENGTH 256 //Definição do tamanho da alocação de memoria flash
#define MEMORY_LONG_BUFF_LENGTH 512 //Definição do tamanho da alocação de memoria flash
#define MEMORY_BUFF_LENGTH_10_BIT 1024 //Definição do tamanho da alocação de memoria flash
#define MEMORY_TIMER_BUFF_LENGTH 2048 //Definição do tamanho da alocação de memoria flash

#define CONST_STR_PRODUCT_NAME "ATHENAS-"

#define CONST_STR_MDNS_NAME "athenas"

#define WIFI_MAC_SIZE 6
#define SERIAL_NUMBER_LEN ((WIFI_MAC_SIZE * 2) + 1)

#define REMINDER_MAX 7 // Maximo de lembretes que o sistema suporta
#define REMINDER_DATE_MAX_LEN 11 // Formato DD/MM/AAAA
#define REMINDER_TIME_MAX_LEN 6 // Formato HH:MM
#define REMINDER_MESSAGE_MAX_LEN 50 // Tamanho máximo da mensagem do lembrete
/**
 * @brief Flag de status, utilizada principalmente no display.
 * 
 * Caso precise de uma flag de status para set e reset,
 * utilize essa estrutura.
 * 
 *  se == FLAG_STATUS_RST, nao tem nenhuma flag ativa.
 * 
 *  se == FLAG_STATUS_SET, tem uma nova configuracao 
 *  de flag que deve ser verificada e resetada.
*/
typedef enum GENERIC_FLAG_STATUS{
    FLAG_STATUS_RST = 0,
    FLAG_STATUS_SET
}GenericFlagStatus;

#endif /* SYSTEM_CONSTANTS_H_ */