#include "taskManager/Tasks/ServoTask.hpp"
#include "taskManager/taskManager.h"
#include "Hardware/hardware.h"
#include "Peripheral/ServoControl/servoControl.hpp"

void servo_Task(void* pvParameters){
    SystemStatus *systemStatus = (SystemStatus *)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda o sistema estabilizar

    servoControl_ini(systemStatus);

    for (;;) {
        servoControl_run(systemStatus);
        vTaskDelay(pdMS_TO_TICKS(10)); // ~10 ms → bom compromisso entre latência e CPU
    }
}