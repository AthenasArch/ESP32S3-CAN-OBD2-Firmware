#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#include <Arduino.h>
#include "Database/Structures/systemStatus.h"

void servoControl_ini(SystemStatus* systemStatus);
void servoControl_run(SystemStatus* systemStatus);

#endif /* SERVO_CONTROL_H_ */ 