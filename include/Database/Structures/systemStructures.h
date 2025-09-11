#ifndef SYSTEM_STRUCTURES_H_
#define SYSTEM_STRUCTURES_H_

#include <Arduino.h>
#include "Database/Structures/systemInformation.h"
#include "Database/Structures/systemConstants.h"
#include "modules/eeprom/MAP_FLASH.h"

/////////////////////////////////////////////////////////////////////
///
///                 @note ESTRUTURAS DE ALARME
///
/////////////////////////////////////////////////////////////////////
typedef enum DB_ALARM{
    ERROR_NONE_MACHINE_OK = 0,
    ERROR_GENERIC = 1
} DbAlarm;

/////////////////////////////////////////////////////////////////////
///
///                 @note ESTRUTURAS DO MICROCONTROLADOR
///
/////////////////////////////////////////////////////////////////////
typedef struct DB_UC_STATUS{
    uint8_t cntTimeMillis;
} DbUcStatus;

/////////////////////////////////////////////////////////////////////
///
///                 @note ESTRUTURAS DO WIFI - AP & STA
///
/////////////////////////////////////////////////////////////////////
typedef enum NET_MDOE{
    NET_AP      = 0, // Outros dispositivos podem se conectar ao ESP
    NET_STATION, // ESP se conecta a outros dispositivos
    NET_APSTA // Pode se conectar e receber conexao
}NetMode;

typedef enum CONNECTED_TO_MODEM_STATUS{
    NO_CONNECTED_TO_MODEM = 0,
    CONNECTED_TO_MODEM  = 1
}ConnModemStatus;

/** @brief STATUS DO MODO STATION */
typedef enum DB_STATION_STATUS {
    STATION_IDLE                    = 0,  // Estado inicial, não tentando conectar
    STATION_CONNECTING_TO_AP        = 1,  // Tentando conectar ao ponto de acesso
    STATION_CONNECTED_TO_AP         = 2,  // Conectado ao ponto de acesso
    STATION_FAILED_CONNECT_TO_AP    = 3,  // Falha ao conectar ao ponto de acesso
    STATION_DISCONNECTED_FROM_AP    = 4,  // Desconectado do ponto de acesso
    STATION_MODE_NOT_SET            = 5   // Modo station não configurado
} DbStationStatus;

/** @brief STATUS DA CONEXAO COM O SERVIDOR */
typedef enum SERVER_COMMUNICATION_STATUS {
    SERVER_IDLE                 = 0,  // Esperando para iniciar comunicação
    SERVER_CONNECTING           = 1,  // Tentando conectar ao servidor
    SERVER_CONNECTED            = 2,  // Conectado ao servidor
    SERVER_FAILED_CONNECT       = 3,  // Falha ao conectar ao servidor
    SERVER_SENDING_DATA         = 4,  // Enviando dados para o servidor
    SERVER_DATA_SENT_SUCCESS    = 5,  // Dados enviados com sucesso
    SERVER_ERROR_SENDING_DATA   = 6   // Erro ao enviar dados
} ServerCommunicationStatus;

/** @brief STATUS DO MODO ACCESS POINT */
typedef enum DB_AP_STATUS {
    AP_IDLE                        = 0,  // Estado inicial, AP não iniciado
    AP_STARTING                    = 1,  // AP está inicializando
    AP_RUNNING                     = 2,  // AP em funcionamento e disponível para conexões
    AP_CLIENT_CONNECTED            = 3,  // Um dispositivo cliente se conectou ao AP
    AP_CLIENT_DISCONNECTED         = 4,  // Um dispositivo cliente se desconectou do AP
    AP_STOPPING                    = 5,  // AP está sendo desativado
    AP_STOPPED                     = 6,  // AP desativado
    AP_ERROR                       = 7,  // Erro ao iniciar ou manter o AP
    AP_MAX_CLIENTS_REACHED         = 8,  // Número máximo de clientes conectados alcançado
    AP_DHCP_FAIL                   = 9   // Falha ao atribuir IP via DHCP aos clientes
} DbAaccessPointStatus;

/////////////////////////////////////////////////////////////////////
///
///         @note NVS - WiFi
///
/////////////////////////////////////////////////////////////////////
// status da conexao com algum modem
typedef struct DB_CONNECTION_CREDENTIALS{
    char ssid[CONST_SSID_MAX_LEN];
    char password[CONST_PASS_MAX_LEN];
    bool hasCredentials;
}DbConnectionCredentials;

/** @brief Station - STA */
typedef struct DB_STATION {
    uint8_t cntTryConnectToAp;
    char IP[CONST_IP_LEN];
    int32_t RSSI;
    DbStationStatus status;
    ConnModemStatus connectedToModem;
    bool communication;
    uint8_t signal; // qualidade do sinal do wifi percentual.
    char strMsgStatus[CONST_STR_MSG_STATUS];
}DbStation;

/** @brief Access Point - AP */
typedef struct DB_ACCESS_POINT {
    char webserverIp[CONST_IP_LEN];
    bool communication;
    ConnModemStatus connectedToModem; // 1 = sim | 0 = nao
    char strMsgStatus[CONST_STR_MSG_STATUS];
    DbAaccessPointStatus status;
    uint8_t numActiveConnections;
}DbAccessPoint;

typedef struct DB_WIFI_STATUS{
    NetMode mode;
    bool flagListingWiFiNetworks;
    DbStation station;
    DbAccessPoint accessPoint;
    char macAddr[6];
    unsigned long TIME_SEND_PACK;
}DbWifiStatus;

typedef struct DB_WIFI_MEMORY{
    DbConnectionCredentials credentialsSta;
    DbConnectionCredentials credentialsAp;
    char localhost[30];     //ip do Servidor de DNS
    uint8_t ipAddress[4];   //Endereço IP em modo dhcp off
    uint8_t netmask[4];     //Máscara de subrede
    uint8_t gateway[4];     //Gateway
    uint8_t dns[4];         //ip do Servidor de DNS
    uint16_t tcpPort;       //porta de comunicação
    uint8_t dhcp:1;         //Flag de DHCP
} DbWifiMemory;


/////////////////////////////////////////////////////////////////////
///
///                 @note ESTRUTURAS OPERACIONAIS DA MAQUINA 
///
/////////////////////////////////////////////////////////////////////
/** @brief STATUS DE CONFIGURACAO DE CADA ITEM DA MAQUINA */
typedef enum DB_LANGUAGE{
    LANGUAGE_EN=0,
    LANGUAGE_ES,
    LANGUAGE_PT_BR,
} DbLanguage;

typedef enum DB_UPDATE_STATUS{
    UPDATE_IDLE = 0,
    START_UPDATE,
    LOAD_UPDATING,
    UPDATE_FAIL,
    UPDATE_SUCCESS,
} DbUpdateStatus;

typedef struct DB_FILE_UPDATE{
    DbUpdateStatus status;
    unsigned long lastUpdateTimer;
    uint8_t lastPercent;
    uint8_t currPercent;
    uint8_t idx;
} DbUpdate;

/////////////////////////////////////////////////////////////////////
///
///                 @note ESTRUTURAS DE COMUNICACAO COM 
///                 SERVIDORES E CLIENTS EXTERNOS
///
/////////////////////////////////////////////////////////////////////
/** @brief Todos os softwares tem DB machine, que cuida de todos os status da maquina
 *  a maquina é a aplicação do produto, esse caso, a maquina é a irrigaplay
 */

/** @brief Detalhes da requisição ao servidor externo */
// typedef struct SERVER_REQUEST_DETAILS {
//     char lastEndpoint[128];          // Último endpoint acessado
//     uint16_t lastHttpStatusCode;     // Código de status da última resposta HTTP
//     char lastRequestTime[20];        // Timestamp da última requisição
//     char lastResponseTime[20];       // Timestamp da última resposta
// } ServerRequestDetails;

typedef enum DB_SERVERS_STATUS{
    IDLE=0,               // Nenhuma comunicação em andamento
    REQUESTING_YOUTUBE, // Requisitando dados do YouTube
    REQUESTING_TAGOIO,  // Requisitando dados do TagoIO
    REQUESTING_NTP     // Sincronizando hora via NTP
} DbServersStatus;

typedef struct DB_SERVER_COMMUNICATION{
    DbServersStatus status;
    // ServerRequestDetails lastRequest;  // Detalhes da última requisição
    uint32_t successfulTransactions;   // Contagem de transações bem-sucedidas
    uint32_t failedTransactions;       // Contagem de transações falhas
    // char lastErrorMessage[256];        // Mensagem de erro da última falha
} DbServerCommunication;



typedef struct DB_GOOGLE_ADDR_STATUS {
    std::string street_address;
}DbGoogleAddrStatus;
/** @brief Tamanho máximo das strings de informações */
#define CHANNEL_USER_INFO_MAX_LEN 100

/**
 * @brief Cuida das informações internas do usuário 
 *        (como chaves, senhas, IDs, tokens, etc.).
 */
typedef struct DB_CHANNEL_INTERNAL_INFO {
    char id[CHANNEL_USER_INFO_MAX_LEN];
    char key[CHANNEL_USER_INFO_MAX_LEN];
    char token[CHANNEL_USER_INFO_MAX_LEN];  // Token de acesso ou outros dados confidenciais
} DbChannelPrivateInfo;

/**
 * @brief Cuida das informações públicas básicas do usuário 
 *        (como nome de usuário e nome do canal/perfil).
 */
typedef struct DB_CHANNEL_PUBLIC_INFO {
    char userName[CHANNEL_USER_INFO_MAX_LEN];   // Nome de usuário (login)
    char channelName[CHANNEL_USER_INFO_MAX_LEN];  // Nome do canal ou perfil
    // Métricas comuns a várias redes
    unsigned long followers;   // Número de seguidores
    unsigned long following;   // Número de contas seguidas

    unsigned long posts;       // Número de publicações (posts, tweets, etc.)
    
    // Métricas específicas para YouTube
    unsigned long videos;      // Quantidade de vídeos postados
    unsigned long views;       // Total de visualizações
    
    // Métricas específicas para TikTok
    unsigned long likes;       // Total de curtidas recebidas
} DbChannelPublicInfo;








/**
 * @brief Estrutura genérica para armazenar as métricas e informações de um canal.
 *
 * Essa estrutura comporta dados que podem ser comuns (como seguidores e publicações)
 * e também campos específicos para determinadas redes (como inscritos, vídeos, views, curtidas, tweets, etc.).
 * Mesmo que alguma rede não utilize todos os campos, o espaço extra é ignorado.
 */
/**
 * @brief Estrutura agregadora para redes sociais.
 *
 * Cada membro representa uma rede social específica, utilizando a estrutura
 * genérica DbChannelStatus para armazenar todas as informações e métricas.
 */
/**
 * @brief Estrutura agregadora para redes sociais.
 *
 * Cada membro representa uma rede social específica, utilizando a estrutura
 * genérica DbChannelStatus para armazenar todas as informações e métricas.
 */
typedef struct DB_SOCIAL_NETWORK_INFO {
    DbChannelPublicInfo instagram;
    DbChannelPublicInfo youtube;
    // DbChannelPublicInfo tiktok;
    // DbChannelPublicInfo threads;
    // DbChannelPublicInfo x;  // Representa o antigo Twitter (atual X)
} DbSocialNetworkInfo;



typedef enum SD_CARD_COMMUNICATION {
    SD_CARD_OK = 0,
    SD_CARD_FAIL_OPEN,
    SD_CARD_FAIL_JSON
} SdCardCommunication;


typedef enum DB_GIF_MODE {
    GIF_RANDOM  = 0,
    GIF_STATIC,
    GIF_BAD,
    GIF_SLEEP,
    GIF_MUSIC,
    GIF_YOUTUBE,
    GIF_WEB_SERVER
} DbGifMode;


typedef enum DB_REQUESTING_DATA {
    FREE = 0, // Esta livre para outras tarefas.
    REQUESTING // Vai solicitar dados na Web...
} DbRequestingData;

/*flags de status de operaçao do sistema*/
typedef struct FLAGS {
    bool wifi_connected;
    // std::map<int, int> servo_pos;
    DbRequestingData requestingWebData;
}Flags_t;

/*flags de status de operaçao do sistema*/
typedef struct DB_SD_CARD_STATUS {
    SdCardCommunication status; // qualidade do sinal do wifi
}DbSdCardStatus;

/////////////////////////////////////////////////////////////////////
///
///                 @note ESTRUTURAS DO DISPLAY
///
/////////////////////////////////////////////////////////////////////

typedef enum DB_DISPLAY_PAGE{
    
    
    DISPLAY_PAGE_GIF = 0, // Nenhuma comunicação em andamento
    DISPLAY_PAGE_NIKO,
    DISPLAY_PAGE_PIXEL_ART,
    
    DISPLAY_PAGE_GYROSCOPE,
    DISPLAY_PAGE_DICE,
    DISPLAY_PAGE_SLOT_MACHINE,
    DISPLAY_PAGE_WIFI,
    // DISPLAY_PAGE_SENSORS,
    DISPLAY_PAGE_ANALOG_CLOCK,               
    // DISPLAY_PAGE_SHOW_IMG,
    DISPLAY_PAGE_SHOW_IMG_FROM_FILE,
    // DISPLAY_PAGE_YOUTUBE,
    // DISPLAY_PAGE_DEBUG,
    // DISPLAY_PAGE_INSTAGRAM,
    DISPLAY_PAGE_LEMBRETES,
    DISPLAY_PAGE_NAMORADA,
    DISPLAY_PAGE_CLOCK,
    // DISPLAY_PAGE_INFO,
    DISPLAY_PAGE_AUTOMOTIVE_PID,
    // DISPLAY_PAGE_ANIMATION,
    DISPLAY_PAGE_CONFIG,
    DISPLAY_PAGE_TOTAL
} DbDisplayPage;

// typedef struct DB_NIKO_STATUS{
//     EyesMood currEyes;
// } DbNikoStatus;

// /*flags de status de operaçao do sistema*/
// typedef struct DB_DISPLAY_STATUS {
//     char msgBuff[50];
//     DbGifMode gifMode;
//     uint8_t currStaticGif;
//     bool gifsDelay;
//     DbNikoStatus niko;
//     DbDisplayPage currPage;
//     DbDisplayPage lastPage;
//     bool btnEnterStatus;
//     bool configuration;
//     bool configurationChange;
//     int configurationIndex;
//     bool reminderTriggered;
// }DbDisplayStatus;

typedef struct DB_PIXEL_ART{
    bool activityViaWebServer;
    bool enable;
    uint16_t color[13][16];
}DbPixelArt;


typedef enum DB_WEB_SERVERS_PAGE{
    WEB_SERVER_PAGE_WIFI=0,               // Nenhuma comunicação em andamento
    WEB_SERVER_PAGE_TIMERS, // Requisitando dados do YouTube
    WEB_SERVER_PAGE_TOKENS,  // Requisitando dados do TagoIO
    WEB_SERVER_PAGE_ABOUT     // Sincronizando hora via NTP
} DbWebServersPage;

/**
 * @brief
 */
typedef struct DB_WEB_SERVER{

    DbWebServersPage webPage; // pagina que esta utilizando atualmente
    // verifica se alguem requisitou alguma pagina do webserver
    // essa variavel é utilizada para direcionar recursos de outras 
    // tarefas para o webserver, indicando que se o usuário está tentando
    // acessar o webserver, devemos deixar recursos para ele.
    bool someoneIsTryingToAccess; 

    bool userIsLoggedIn; // status, se o usuario estiver logado, retorna true
    unsigned long timerlastUserActivity; // timer para verificar a atividade no web server
    unsigned long timerUserLogged; // timer permissao de login do usuario
} DbWebServer;


typedef struct DB_SERVO_CONTROL{
    uint8_t leftWhell;
    uint8_t rightWhell;
    uint8_t leftFront;
    uint8_t rightFront;
} DbServoControl;

typedef enum MOTION {
    MO_NONE,
    MO_STOPPED,
    MO_SINGLE_TAP
} Motion;

// Estrutura para armazenar os dados do QMI8658C
typedef struct DB_GYROSCOPE_CONTROL {

    bool error;

    struct {
        float x; // Aceleração no eixo X (g)
        float y; // Aceleração no eixo Y (g)
        float z; // Aceleração no eixo Z (g)
    } accelerometer;

    struct {
        float x; // Velocidade angular no eixo X (°/s)
        float y; // Velocidade angular no eixo Y (°/s)
        float z; // Velocidade angular no eixo Z (°/s)
    } gyroscope;

    struct {
        float x; // Velocidade angular no eixo X (°/s)
        float y; // Velocidade angular no eixo Y (°/s)
        float z; // Velocidade angular no eixo Z (°/s)
    } angle;

    float temperature; // Temperatura interna (°C)
    
    struct {
        float roll;  // Inclinação em torno do eixo X (°)
        float pitch; // Inclinação em torno do eixo Y (°)
        float yaw;   // Inclinação em torno do eixo Z (°)
    } orientation;
  
    Motion motion;

} DbGyroscopeControl;



typedef enum DB_BATTERY_STATUS {
    BATTERY_CHARGING = 0,      // Bateria carregando
    BATTERY_DISCHARGING,       // Bateria descarregando
    BATTERY_FULL,              // Bateria carregada completamente
    BATTERY_LOW,               // Bateria com carga baixa
    BATTERY_CRITICAL,          // Bateria em estado crítico
    BATTERY_CUTOFF             // Energia cortada para evitar descarga profunda
} DbBatteryStatus;

typedef struct DB_BATTERY_CONTROL {
    uint8_t percentage;  // Percentual estimado de carga
    float voltage;       // Tensão atual da bateria
    DbBatteryStatus status; // Status da bateria
    char strMsgStatus[CONST_STR_MSG_STATUS];
} DbBatteryControl;


typedef struct DB_SPIFFS_STATUS {
    bool clearingMemory;        // true se estiver formatando
    uint8_t clearingProgress;   // 0-100%
    bool memoryError;           // true se houve falha de montagem
} DbSpiffsStatus;


typedef enum DB_BUZZER_SONG_ID{
    BUZZER_SONG_STARWARS=0,
    BUZZER_SONG_PIRATES,
    BUZZER_SONG_GOT,
    BUZZER_SONG_HARRYPOTER,
    BUZZER_SONG_NOKIA,
    BUZZER_SONG_MARIO,
    BUZZER_SONG_IMPERIAL,
    BUZZER_SONG_TETRIS,
    BUZZER_SONG_HAPPYBDAY,
    BUZZER_SONG_NG_GIVEUP,
    BUZZER_SONG_KEYBOARD_CAT,
    BUZZER_SONG_CHRISTMAS,
    BUZZER_SONG_MINUET,
    BUZZER_SONG_PINKPANTHER,
    BUZZER_SONG_LIONSLEEPS,
    BUZZER_SONG_GANGSTASPARADISE,
    BUZZER_SONG_DRAGONBALLGT,
    BUZZER_SONG_DRAGONBALLZ,
    BUZZER_SONG_SPIDERMAN,
    BUZZER_SONG_TITANIC,
    BUZZER_SONG_SIMPSONS,
    BUZZER_SONG_MISSIONIMPOSSIBLE,
    BUZZER_SONG_BARBIGIRL,


    BUZZER_SONG_NONE  // sempre o último
} DbBuzzerSongID;

typedef struct {
    DbBuzzerSongID id;
    const char* name;
    const int* notes;
    int length;
    int songSpeed;
} BuzzerSong;

#define MAX_SONG_NAME_LEN 8

typedef struct DB_BUZZER_SONG_PLAYER{
    bool isPlaying;
    const int* notes;
    int length;
    int songSpeed;
    float speedFactor;
    int minNoteDuration;
    int index;
    bool noteOn;
    int noteDuration;
    int pauseBetweenNotes;
    unsigned long lastMillis;
    bool isActive;
    bool isPaused;
    DbBuzzerSongID currentSongID;
    char currentSongName[MAX_SONG_NAME_LEN];  ///< nome da música tocando
    const BuzzerSong* availableSongs;
} DbBuzzerSongPlayer;

/** @brief Todos os softwares tem DB machine, que cuida de todos os status da maquina
 *  a maquina é a aplicação do produto, esse caso, a maquina é a irrigaplay
 */
typedef struct DB_MACHINE{
    DbBuzzerSongPlayer buzzerSong;
    DbBatteryControl battery;
    DbGyroscopeControl gyroscope;
    DbServoControl servo;
    tm ntpTimeDate;
    tm rtcTimeDate;
    DbLanguage language;
    uint8_t cnt;
    // DbWifiStatus wifi;
    // DbSdCardStatus sdCard;
    bool relay[2];
    bool allTasksInitialized;
    DbSpiffsStatus spiffsStatus;
    DbUpdate imgUpdate;
    DbUpdate fwUpdate;
} DbMachine;

typedef struct DB_CAN_MONITOR {
    float batteryVoltage;                        // PID 0x42 - Tensão da ECU (Ex: 12.0–14.8 V)
    uint8_t intakeManifoldPressure;              // PID 0x0B - Pressão do coletor de admissão (0–255 kPa)
    uint8_t throttlePosition;                    // PID 0x11 - Posição da borboleta (0–100 %)
    float ignitionTimingAdvance;                // PID 0x0E - Avanço de ignição (-64 a +63.5 °)
    uint8_t calculatedLoadValue;                 // PID 0x04 - Carga calculada do motor (0–100 %)
    uint8_t fuelLevel;                           // PID 0x2F - Nível de combustível (0–100 %)
    uint16_t vehicleSpeed;                       // PID 0x0D - Velocidade do veículo (0–255 km/h)
    uint16_t engineRPM;                          // PID 0x0C - Rotação do motor (0–16383.75 RPM)
    signed int engineCoolantTemperature;            // PID 0x05 - Temperatura do líquido de arrefecimento (-40 a +215 °C)
    signed int intakeAirTemperature;                 // PID 0x0F - Temperatura do ar admitido (-40 a +215 °C)
    float massAirFlowRate;                       // PID 0x10 - Vazão de ar da MAF (0–655.35 g/s)
    float fuelPressure;                          // PID 0x0A - Pressão do combustível (0–765 kPa)
    float engineFuelRate;                        // PID 0x5E - Vazão de combustível (0–3212.75 L/h)
    uint8_t ethanolFuelPercentage;               // PID 0x52 - Porcentagem de etanol (0–100 %)
    uint16_t runTimeSinceEngineStart;            // PID 0x1F - Tempo de motor ligado (0–65535 s)
    uint16_t distanceWithMILOn;                  // PID 0x21 - Distância com luz de falha acesa (0–65535 km)
    uint16_t distanceSinceCodesCleared;          // PID 0x31 - Distância desde a limpeza dos códigos (0–65535 km)
    uint16_t barometricPressure;                 // PID 0x33 - Pressão atmosférica (0–255 kPa)
} DbCanMonitor;

typedef struct DB_AUTOMOTIVE_SYSTEM {
    DbCanMonitor canMonitor;
    unsigned long timerToRequestPIDs;
    bool     canExtended;   // true = 29 bits, false = 11 bits
    uint32_t canBaud;
    bool     canConnected;  // true = ECU respondeu, false = desconectado
} DbAutomotiveSystem;

/////////////////////////////////////////////////////////////////////
///
///                 @note ESTRUTURAS DA MEMORIA NVS
///
/////////////////////////////////////////////////////////////////////
/**
 * @brief NVS - CONFIGURACOES DE REDES SOCIAIS / SOCIAL NETOWORK
 *              YOUTUBE
 *              INSTAGRAM
 *              ... 
 */
typedef struct DB_TOKENS_MEMORY{
    DbChannelPrivateInfo youtube;
    DbChannelPrivateInfo instagram;
} DbTokensMemory;

typedef union DB_NVS_TOKENS {
    DbTokensMemory tokensMemory;
    uint8_t bufferMemory[MEMORY_BUFF_LENGTH];
} DbNvsTokens;

/**
 * @brief NVS - LEMBRETES / REMINDERS
 */
typedef struct DB_REMINDERS {
    struct {
        uint8_t enabled; // Se o lembrete está ativado
        uint8_t repeat; // Frequência de repetição (0-6, onde 0 é nunca 1 é diário, 2 é semanal, 3 mensal, 4 anual etc.)
        uint8_t weekDay; // Dia da semana (cada bit representa um dia, 0 = domingo, 1 = segunda, etc.)
        uint8_t imageId; // ID da imagem associada ao lembrete
        char date[REMINDER_DATE_MAX_LEN]; // Data do lembrete no formato "DD/MM"
        char time[REMINDER_TIME_MAX_LEN]; // Horário do lembrete no formato "HH:MM"
        char message[REMINDER_MESSAGE_MAX_LEN]; // Mensagem do lembrete
    } reminders[REMINDER_MAX]; // Suporta até 6 lembretes
} DbLembretesMemory;

typedef union DB_NVS_LEMBRETES {
    DbLembretesMemory lembretesMemory;
    uint8_t bufferMemory[MEMORY_LONG_BUFF_LENGTH];
} DbNvsLembretes;


/**
 * @brief NVS - CONFIGURACOES DE REDE / NETOWORK / WIFI 
 */
typedef union DB_NVS_NETWORK {
    DbWifiMemory wifiMemory;
    uint8_t bufferMemory[MEMORY_BUFF_LENGTH];
} DbNvsNetwork;


typedef struct DB_PERIODIC_MAINTENANCE {
    bool enable;
    uint8_t hour;      
    uint8_t weekDay;    
 } DbPeriodicMaintenance;


typedef enum DB_IMG_SLOT {
    IMG_SLOT_1 = 0,
    IMG_SLOT_2,    
    IMG_SLOT_3,    
    IMG_SLOT_4,
    IMG_SLOT_5,
    IMG_SLOT_6,    
    IMG_SLOT_7,    
    IMG_SLOT_8,
    IMG_SLOT_9,    
    IMG_SLOT_10,
    IMG_SLOT_TOTAL
} DbImgSlot;

 
typedef struct DB_IMG_MANAGER {
    bool isSaved;                // true se a imagem foi salva com sucesso
    char name[10];              // nome do arquivo original (com extensão .jpg)
    uint32_t sizeBytes;         // tamanho em bytes da imagem
    uint8_t slot;               // slot da imagem (0 a 3 por exemplo)
    uint32_t timestamp;         // timestamp Unix do momento de gravação
} DbImgManager;

/**
 * @brief NVS - CONFIGURACOES GERAIS 
 */
typedef struct DB_NIKO_PARAMETERS {
    DbImgManager imgManager[IMG_SLOT_TOTAL];
    char SerialNumber[SERIAL_NUMBER_LEN];
    char webServerPass[CONST_PASSWORD_LEN]; //Senha de acesso a configurações
    unsigned long timeSendDataToHost; // tempo de envio de dados para a algum endpoint.
    unsigned long timeGetDataFromHost; // tempo de requisicao de dados de algum host.
    DbPeriodicMaintenance maintence;
    bool buzzerEnabled;
    uint8_t muscInit;
    uint8_t telaInit;
    uint8_t imgClock;
    uint16_t ledColor;
    DbBuzzerSongID musicAlarm;
}DbNikoConfig;

typedef union DB_NVS_CONFIG {
    DbNikoConfig cfgMemory;
    uint8_t bufferMemory[MEMORY_BUFF_LENGTH_10_BIT];
} DbNvsNikoConfig;





#endif /* SYSTEM_STRUCTURES_H_ */