#include "Hardware/hardware.h"
#include <Arduino.h>

#define HARDWARE_DEBUG_ENABLE // descomente esta linha para habilitar o debug. 
#ifdef HARDWARE_DEBUG_ENABLE
  #define HARDWARE_DEBUG_PRINT(...) { Serial.print(__VA_ARGS__); }
  #define HARDWARE_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
#else
  #define HARDWARE_DEBUG_PRINT(...) {}
  #define HARDWARE_DEBUG_PRINTLN(...) {}
#endif

void ini_digitalOutputs(void);
void ini_digitalInputs(void);
void ini_analog(void);
void ini_pwm(void);

void hardware_initialize(void){
    HARDWARE_DEBUG_PRINTLN("hardware_initialize");
    ini_digitalOutputs();
    ini_digitalInputs();
    ini_analog();
    ini_pwm();
    HARDWARE_DEBUG_PRINTLN("hardware_initialize END");
}

void ini_digitalOutputs(void){

    pinMode(PIN_SYSTEM_POWER_ENABLE, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);


    digitalWrite(PIN_SYSTEM_POWER_ENABLE, 1);

    // pinMode(PIN_BUZZER, OUTPUT);

    // digitalWrite(PIN_BUZZER, 0);
}

void ini_digitalInputs(void){

    // pinMode(, INPUT_PULLUP);
    // btn de controle da eeprom...
    // pinMode(PIN_BUTTON_CLR_EEPROM, INPUT);
}

void ini_analog(void){
    // pinMode(PIN_POT_V_IN, INPUT);
}

void ini_pwm(void){
    // Configura os canais PWM
    // ledcSetup(PWM_CHANNEL_1, PWM_FREQ_1, PWM_RESOLUTION);
}