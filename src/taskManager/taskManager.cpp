#include "taskManager/taskManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/** @brief Tasks do sistema*/
#include "taskManager/Tasks/canTask.hpp"

/**
 * @brief Converte o valor em kilobytes para o valor equivalente em palavras de stack (StackType_t).
 *
 * No ESP32 (FreeRTOS), cada palavra da stack tem 4 bytes (32 bits).
 * As funções de criação de task (`xTaskCreate` e `xTaskCreatePinnedToCore`) esperam o tamanho da stack
 * em palavras, e não em bytes.
 *
 * Exemplo:
 * - SIZE_TASK_STACK(4) → aloca 4 KB de stack = 1024 palavras (4096 bytes).
 * - SIZE_TASK_STACK(8) → aloca 8 KB de stack = 2048 palavras.
 */
// #define SIZE_TASK_STACK(KB) ((KB * 1024) / 4) // 4 bytes por palavra
//  #define SIZE_TASK_STACK(KB) ((KB * 1024) / sizeof(StackType_t))
#define SIZE_TASK_STACK(Size) ((Size) * 1024)

#define WIFI_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define WEBSERVER_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define SENSORS_TASK_PRIORITY 10
#define SERVERS_TASK_PRIORITY 9
#define DISPLAY_TASK_PRIORITY 8
#define BUZZER_TASK_PRIORITY 7
#define LED_TASK_PRIORITY 6
#define CAN_TASK_PRIORITY 5
#define GYROSCOPE_TASK_PRIORITY 4
#define LEMBRETES_TASK_PRIORITY 3


// Mutex para garantir acesso prioritário
SemaphoreHandle_t xMutex;

void taskManager_checkWebServerAccess(SystemStatus* systemStatus);

/**
 * @brief Inicializa e cria todas as tasks do sistema com seus respectivos tamanhos de stack e prioridades.
 *
 * Essa função utiliza a macro SIZE_TASK_STACK para alocar a quantidade de memória apropriada 
 * para cada task, com base no histórico de uso e na complexidade esperada de cada função.
 * 
 * As tasks são divididas entre os dois núcleos do ESP32 (PRO_CPU e APP_CPU).
 *
 * @param systemStatus Ponteiro para a estrutura principal de status do sistema, passada como parâmetro para cada task.
 */
void taskMAnager_init(SystemStatus *systemStatus) {

    // Inicializa o mutex
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        printf("Erro ao criar o mutex\n");
        return;
    }

    // /**
    //  * @brief Tasks para o Core PRO_CPU_NUM
    //  */
    // xTaskCreatePinnedToCore(wifi_task, TASK_NAME_WIFI, SIZE_TASK_STACK(5), systemStatus, WIFI_TASK_PRIORITY, NULL, PRO_CPU_NUM);
    // xTaskCreatePinnedToCore(webServer_task, TASK_NAME_WEB_SERVER, SIZE_TASK_STACK(10), systemStatus, WEBSERVER_TASK_PRIORITY, NULL, PRO_CPU_NUM);
    // xTaskCreatePinnedToCore(sensors_Task, TASK_NAME_SENSORS, SIZE_TASK_STACK(3), systemStatus, SENSORS_TASK_PRIORITY, NULL, PRO_CPU_NUM);
    // /**
    //  * @brief Tasks para o Core APP_CPU_NUM
    //  */
    // xTaskCreatePinnedToCore(servers_task, TASK_NAME_SERVERS, SIZE_TASK_STACK(5), systemStatus, SERVERS_TASK_PRIORITY, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(displayTask_run, TASK_NAME_DISPLAY, SIZE_TASK_STACK(18), systemStatus, DISPLAY_TASK_PRIORITY, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(buzzerTask_run, TASK_NAME_BUZZER, SIZE_TASK_STACK(4), systemStatus, BUZZER_TASK_PRIORITY, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(ledTask_run, TASK_NAME_LED, SIZE_TASK_STACK(3), systemStatus, LED_TASK_PRIORITY, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(canTask_run, TASK_NAME_CAN, SIZE_TASK_STACK(6), systemStatus, CAN_TASK_PRIORITY, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(gyroscopeTask_run, TASK_NAME_GYROSCOPE, SIZE_TASK_STACK(4), systemStatus, GYROSCOPE_TASK_PRIORITY, NULL, APP_CPU_NUM);

    // xTaskCreate(displayTask_run, TASK_NAME_DISPLAY, SIZE_TASK_STACK(10), systemStatus, DISPLAY_TASK_PRIORITY, NULL); // atualizada
    // // Todas as tasks criadas com os tamanhos de stack já definidos
    // fileSystem_init(systemStatus);
    // xTaskCreate(wifi_task, TASK_NAME_WIFI, SIZE_TASK_STACK(4), systemStatus, WIFI_TASK_PRIORITY, NULL); // atualizada
    // xTaskCreate(webServer_task, TASK_NAME_WEB_SERVER, SIZE_TASK_STACK(6), systemStatus, WEBSERVER_TASK_PRIORITY, NULL); // atualizada
    // xTaskCreate(sensors_Task, TASK_NAME_SENSORS, SIZE_TASK_STACK(2), systemStatus, SENSORS_TASK_PRIORITY, NULL); // atualizada
    // xTaskCreate(servers_task, TASK_NAME_SERVERS, SIZE_TASK_STACK(4), systemStatus, SERVERS_TASK_PRIORITY, NULL); // Sem Youtube
    // xTaskCreate(buzzerTask_run, TASK_NAME_BUZZER, SIZE_TASK_STACK(4), systemStatus, BUZZER_TASK_PRIORITY, NULL);
    // xTaskCreate(ledTask_run, TASK_NAME_LED, SIZE_TASK_STACK(3), systemStatus, LED_TASK_PRIORITY, NULL);
    xTaskCreate(canTask_run, TASK_NAME_CAN, SIZE_TASK_STACK(6), systemStatus, CAN_TASK_PRIORITY, NULL);
    // xTaskCreate(gyroscopeTask_run, TASK_NAME_GYROSCOPE, SIZE_TASK_STACK(3), systemStatus, GYROSCOPE_TASK_PRIORITY, NULL);
    // xTaskCreate(lembretes_task, TASK_NAME_LEMBRETES, SIZE_TASK_STACK(2), systemStatus, LEMBRETES_TASK_PRIORITY, NULL);
    systemStatus->machine.allTasksInitialized = true;
}

void taskManager_run(SystemStatus* systemStatus) {
    // taskManager_checkWebServerAccess(systemStatus);
    vTaskDelay(pdMS_TO_TICKS(10)); // Adiciona um pequeno delay para evitar loop intenso
}

/**
 * @brief Verifica o uso mínimo histórico da stack (pilha) da task atual.
 *
 * IMPORTANTE: Este projeto utiliza a macro SIZE_TASK_STACK(Size) definida como ((Size) * 1024),
 * assumindo que a stack está sendo passada diretamente em BYTES para a função de criação da task.
 * Apesar da API do FreeRTOS esperar o valor em PALAVRAS (StackType_t), esta convenção é usada
 * intencionalmente e todo o sistema é padronizado em bytes.
 *
 * Esta função converte o valor de palavras livres (retornado por uxTaskGetStackHighWaterMark)
 * em bytes e imprime em uma única linha para facilitar a análise e manter o padrão de logs.
 *
 * @param strName Nome da task, usado apenas para exibição no log.
 * @param timerTask Ponteiro para um contador `millis()` usado para limitar a frequência da checagem.
 */
// #define DEBUG_STACK_TASKS
void task_checkUsedMem(const char *strName, unsigned long *timerTask) {
#ifdef DEBUG_STACK_TASKS
    const unsigned long TIME_CHECK_USED_MEM = 500;

    if (millis() - *timerTask > TIME_CHECK_USED_MEM) {
        *timerTask = millis();

        UBaseType_t palavrasLivres = uxTaskGetStackHighWaterMark(NULL);
        size_t bytesLivres = palavrasLivres * sizeof(StackType_t);

        Serial.printf("[STACK] Task: %s | Bytes livres: %u\n", strName, bytesLivres);
    }
#endif
}
