#ifndef SENSOR_TASK_H_
#define SENSOR_TASK_H_

#include <Arduino.h>
#include "Database/Structures/systemStatus.h"

#define TASK_NAME_SENSORS "Sensors Task"

void sensors_Task(void* pvParameters);

#endif /* SENSOR_TASK_H_ */