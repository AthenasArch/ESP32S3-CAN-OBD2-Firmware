#ifndef MACHINE_MANAGER_H_
#define MACHINE_MANAGER_H_

#include <Arduino.h>
#include "Database/Structures/systemStatus.h"

void machineManager_ini(SystemStatus *systemStatus);
void machineManager_run(SystemStatus *systemStatus);

#endif /* SYSTEM_MANAGER_H_ */