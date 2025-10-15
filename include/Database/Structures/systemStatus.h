#ifndef SYSTEM_STATUS_H_
#define SYSTEM_STATUS_H_

#include <Arduino.h>
#include "systemStructures.h"
#include <esp_task_wdt.h>

typedef struct SYSTEM_STATUS {
    DbUcStatus ucStatus; // status do micro controlador
    DbAlarm currAlarm; // status gerais de alarme
    DbWifiStatus wifi; // status do wifi
    DbDisplayStatus display; // status do display
    DbNvsNikoConfig nvsNikoConfig;
    DbNvsNetwork nvsNetworkMemory;
    DbNvsAutomotiveConfig nvsAutomotiveConfig;
    // DbNvsNikoConfig nvsNikoConfig;
    DbNvsTokens nvsTokens;
    DbNvsLembretes nvsLembretes;
    DbServerCommunication servers;
    DbSocialNetworkInfo socialNetoworkInfo;
    DbPixelArt pixelArt;
    Flags_t flags;
    DbMachine machine;
    DbAutomotiveSystem automotiveSystem;
    DbWebServer webServers;
} SystemStatus;



void systemStatus_initialize(SystemStatus* systemStatus);
SystemStatus* systemSystem_getSystemStatus();

#endif /* SYSTEM_STATUS_H_ */