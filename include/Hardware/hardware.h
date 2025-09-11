#ifndef HARDWARE_DEFINITION_H_
#define HARDWARE_DEFINITION_H_

// #include "Hardware/hardware.h"

#include <Arduino.h>
#include <stdio.h>


// #define ELM_PORT Serial1
// #define PIN_ELM_RXD2 17
// #define PIN_ELM_TXD2 18


#define SPI_SD_CARD_SPEED 60000000 /* frequencia do SPI */
// #define SPI_SD_CARD_SPEED 40000000 /* frequencia do SPI */
//  #define SPI_SD_CARD_SPEED 20000000 /* frequencia do SPI */


#pragma once

#define XPOWERS_CHIP_AXP2101


#define LCD_DC 4
#define LCD_CS 5
#define LCD_SCK 6
#define LCD_MOSI 7
#define LCD_RST 8
#define LCD_BL 15
#define LCD_WIDTH 240
#define LCD_HEIGHT 280

#define IIC_SDA 11
#define IIC_SCL 10

#define TP_RST 13
#define TP_INT 14

#define PIN_SYSTEM_POWER_ENABLE 41
#define PIN_BUZZER 42

// Definir o pino do botão
#define PIN_BNT_START_SYSTEM 36 // Substitua pelo pino conectado ao botão
#define PIN_BNT_BOOT 0 // Substitua pelo pino conectado ao botão

#define PIN_PCI_ATHENAS_WS2812B 43 // U0TXD - GPIO43
#define PIN_PCI_ATHENAS_CAN_RX  18
#define PIN_PCI_ATHENAS_CAN_TX  17
#define PIN_PCI_ATHENAS_CAN_LBK 16
#define PIN_PCI_ATHENAS_SW_LEFT  3
#define PIN_PCI_ATHENAS_SW_RIGHT 2
#define PIN_PCI_ATHENAS_SW_ENTER 44 // U0RXD - GPIO44


#define PIN_ADC_BATTERY_VOLTAGE_READER 1

void hardware_initialize(void);

#endif /* HARDWARE_DEFINITION_H_ */
