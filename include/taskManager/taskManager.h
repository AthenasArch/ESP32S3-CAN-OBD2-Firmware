#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <Arduino.h>
#include "Database/Structures/systemStatus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Mutex para garantir acesso prioritário
extern SemaphoreHandle_t xMutex;

void task_checkUsedMem(const char *strName, unsigned long *timerTask);
void taskMAnager_init(SystemStatus* systemStatus);
void taskManager_run(SystemStatus* systemStatus);

#endif /* TASK_MANAGER_H_ */