#include "MachineManager/machineManager.h"
#include "Hardware/hardware.h"
#include "taskManager/taskManager.h"

#define MACHINE_MANAGER_DEBUG_ENABLE // descomente esta linha para habilitar o debug. 
#ifdef MACHINE_MANAGER_DEBUG_ENABLE
  #define MACHINE_MANAGER_DEBUG_PRINT(...) { Serial.print(__VA_ARGS__); }
  #define MACHINE_MANAGER_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
#else
  #define MACHINE_MANAGER_DEBUG_PRINT(...) {}
  #define MACHINE_MANAGER_DEBUG_PRINTLN(...) {}
#endif

/**
 * 
*/
void machineManager_ini(SystemStatus *systemStatus){

    MACHINE_MANAGER_DEBUG_PRINTLN("machineManager_ini(INI)");

    taskMAnager_init(systemStatus);
    
    MACHINE_MANAGER_DEBUG_PRINTLN("machineManager_ini(END)");
}

/**
 * 
*/
void machineManager_run(SystemStatus *systemStatus){

    taskManager_run(systemStatus);
}
