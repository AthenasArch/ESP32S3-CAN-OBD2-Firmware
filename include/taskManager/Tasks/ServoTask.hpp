#ifndef SERVO_TASK_H_
#define SERVO_TASK_H_

#include <Arduino.h>
#include "Database/Structures/systemStatus.h"

#define TASK_NAME_SERVO "Servo Task"

void servo_Task(void* pvParameters);

#endif /* SERVO_TASK_H_ */