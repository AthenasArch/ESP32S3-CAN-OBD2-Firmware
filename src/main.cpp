#include <Arduino.h>
#include <SPI.h>
#include <esp_idf_version.h>
#include <main.h>
#include "Database/Structures/systemStatus.h"
#include "Hardware/hardware.h"
#include "MachineManager/machineManager.h"

/**
 * @brief Arquivo main do firmware.
 * @name 
 * @version | 4.0
 * @date    | 22/11/2022     
 * @github  | First update: 20/06/2020.
 * @author  | leonardohilgemberg@gmail.com  | laonardo Hilgemberg Lopes |
 * @file    | main.h
 * @note    | fone WPP (41)99661-6761
 * @warning | 
 **/

#define MAIN_DEBUG_ENABLE // descomente esta linha para habilitar o debug. 
#ifdef MAIN_DEBUG_ENABLE
    #define MAIN_DEBUG_PRINT(...) { Serial.print(__VA_ARGS__); }
    #define MAIN_DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
    #define MAIN_DEBUG_PRINTF(...)  { Serial.printf(__VA_ARGS__); }
    #define DEBUG_SERIAL_BEGIN(a)    Serial.begin(a)
#else
    #define MAIN_DEBUG_PRINT(...) {}
    #define MAIN_DEBUG_PRINTLN(...) {}
    #define MAIN_DEBUG_PRINTF(...)  {}
    #define DEBUG_SERIAL_BEGIN(a)    do{}while(0)
#endif

SystemStatus systemStatus;

void wdt_init(uint32_t wdtTimeoutSeconds);
void spentResourcesEvaluator(void);

void setup() {
    
    DEBUG_SERIAL_BEGIN(115200); // ok
    MAIN_DEBUG_PRINTLN("Serial Comm Started\r\n");
    MAIN_DEBUG_PRINT("Versão de Software: ");
    MAIN_DEBUG_PRINTLN(FIRMWARE_VERSION);
    MAIN_DEBUG_PRINTLN("CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
    MAIN_DEBUG_PRINTF("ESP-IDF Version: %d.%d.%d\n\n\r\r", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
    // Imprime a versão do compilador GCC
    // MAIN_DEBUG_PRINT("GCC version: ");
    // MAIN_DEBUG_PRINTLN(__VERSION__);

    delay(3000);

    hardware_initialize();

    systemStatus_initialize(&systemStatus);

    machineManager_ini(&systemStatus);

    wdt_init(10); // primeira coisa a ser feita
}

void loop(){
    // system_wdtReset(); // alimenta wdt
    esp_task_wdt_reset();
    machineManager_run(&systemStatus);
}

/**
 * 
*/
void wdt_init(const uint32_t wdtTimeoutSeconds){
    // Configura o WDT com um timeout de 5 segundos
    esp_task_wdt_init(wdtTimeoutSeconds, true); // ativa a flag de panico ao dar o timeout do WDT
    // Adiciona o loop() ao WDT
    esp_task_wdt_add(NULL);
}


/**
 * @warning Abaixo estao exemplos funcionais que o fabricante dexou como exemplo:
 * 
 *          - Vai precisar das bibliotecas, que estao em
 *          Exemplos-ESP32-S3-LCD-1.69_DemoCode\Arduino-v3.0.5\libraries
 *              : Mylibrary
 *              : GFX_Library_for_Arduino
 */























/**
 * @brief FUNCIONA:
 */
// #include <lvgl.h>
// #include "Hardware/hardware.h"
// #include "Arduino_GFX_Library.h"
// #include "Arduino_DriveBus_Library.h"
// #include "lv_conf.h"
// #include <demos/lv_demos.h>
// #include "HWCDC.h"
// #include "Interface/MMI/images/image.h"

// // HWCDC USBSerial;

// Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

// Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
//                                       0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);


// std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
//   std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

// void Arduino_IIC_Touch_Interrupt(void);

// std::unique_ptr<Arduino_IIC> CST816T(new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS,
//                                                          TP_RST, TP_INT, Arduino_IIC_Touch_Interrupt));

// void Arduino_IIC_Touch_Interrupt(void) {
//   CST816T->IIC_Interrupt_Flag = true;
// }

// #define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// uint32_t screenWidth;
// uint32_t screenHeight;

// static lv_disp_draw_buf_t draw_buf;
// // static lv_color_t buf[screenWidth * screenHeight / 10];


// #if LV_USE_LOG != 0
// /* Serial debugging */
// void my_print(const char *buf) {
//   Serial.printf(buf);
//   Serial.flush();
// }
// #endif

// /* Display flushing */
// void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
//   uint32_t w = (area->x2 - area->x1 + 1);
//   uint32_t h = (area->y2 - area->y1 + 1);

// #if (LV_COLOR_16_SWAP != 0)
//   gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #else
//   gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #endif

//   lv_disp_flush_ready(disp);
// }

// void example_increase_lvgl_tick(void *arg) {
//   /* Tell LVGL how many milliseconds has elapsed */
//   lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
// }

// static uint8_t count = 0;
// void example_increase_reboot(void *arg) {
//   count++;
//   if (count == 30) {
//     esp_restart();
//   }
// }

// /*Read the touchpad*/
// void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
//   int32_t touchX = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
//   int32_t touchY = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

//   if (CST816T->IIC_Interrupt_Flag == true) {
//     CST816T->IIC_Interrupt_Flag = false;
//     data->state = LV_INDEV_STATE_PR;

//     /* Set the coordinates with some debounce */
//     if (touchX >= 0 && touchY >= 0) {
//       data->point.x = touchX;
//       data->point.y = touchY;

//       USBSerial.printf("Data x: %d, Data y: %d\n", touchX, touchY);
//     }
//   } else {
//     data->state = LV_INDEV_STATE_REL;
//   }
// }


// void setup() {
//   USBSerial.begin(115200); /* prepare for possible serial debug */
//     hardware_initialize();

//   while (CST816T->begin() == false) {
//     USBSerial.println("CST816T initialization fail");
//     delay(2000);
//   }
//   USBSerial.println("CST816T initialization successfully");

//   CST816T->IIC_Write_Device_State(CST816T->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
//                                   CST816T->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);

//   gfx->begin();
//   pinMode(LCD_BL, OUTPUT);
//   digitalWrite(LCD_BL, HIGH);

//   screenWidth = gfx->width();
//   screenHeight = gfx->height();

//   lv_init();

//   lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);

//   lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);

//   String LVGL_Arduino = "Hello Arduino! ";
//   LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

//   USBSerial.println(LVGL_Arduino);
//   USBSerial.println("I am LVGL_Arduino");



// #if LV_USE_LOG != 0
//   lv_log_register_print_cb(my_print); /* register print function for debugging */
// #endif

//   lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * screenHeight / 4);

//   /*Initialize the display*/
//   static lv_disp_drv_t disp_drv;
//   lv_disp_drv_init(&disp_drv);
//   /*Change the following line to your display resolution*/
//   disp_drv.hor_res = screenWidth;
//   disp_drv.ver_res = screenHeight;
//   disp_drv.flush_cb = my_disp_flush;
//   disp_drv.draw_buf = &draw_buf;
//   lv_disp_drv_register(&disp_drv);

//   /*Initialize the (dummy) input device driver*/
//   static lv_indev_drv_t indev_drv;
//   lv_indev_drv_init(&indev_drv);
//   indev_drv.type = LV_INDEV_TYPE_POINTER;
//   indev_drv.read_cb = my_touchpad_read;
//   lv_indev_drv_register(&indev_drv);

//   lv_obj_t *label = lv_label_create(lv_scr_act());
//   lv_label_set_text(label, "Hello Ardino and LVGL!");
//   lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

//   const esp_timer_create_args_t lvgl_tick_timer_args = {
//     .callback = &example_increase_lvgl_tick,
//     .name = "lvgl_tick"
//   };

//   const esp_timer_create_args_t reboot_timer_args = {
//     .callback = &example_increase_reboot,
//     .name = "reboot"
//   };

//   esp_timer_handle_t lvgl_tick_timer = NULL;
//   esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
//   esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

//   lv_demo_widgets();
//   // lv_demo_benchmark();
//   // lv_demo_keypad_encoder();
//   // lv_demo_music();
//   // lv_demo_stress();

//   // lv_obj_t *img_obj = lv_img_create(lv_scr_act());
//   // lv_img_set_src(img_obj, &img_test3);  // Set the image source to img_test3
//   // lv_obj_align(img_obj, LV_ALIGN_CENTER, 0, 0);
//   // USBSerial.println("Setup done");
// }

// void loop() {
//   lv_timer_handler(); /* let the GUI do its work */
//   delay(5);
// }





/**
 * @brief FUNCIONA: Giroscopio
 */
// #include <lvgl.h>
// #include "Arduino_GFX_Library.h"
// #include "lv_conf.h"
// #include <Arduino.h>
// #include <Wire.h>
// #include "SensorQMI8658.hpp"
// #include "HWCDC.h"

// // HWCDC USBSerial;

// #define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// static lv_disp_draw_buf_t draw_buf;
// static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

// SensorQMI8658 qmi;

// IMUdata acc;
// IMUdata gyr;

// lv_obj_t *label;                  // Global label object
// lv_obj_t *chart;                  // Global chart object
// lv_chart_series_t *acc_series_x;  // Acceleration X series
// lv_chart_series_t *acc_series_y;  // Acceleration Y series
// lv_chart_series_t *acc_series_z;  // Acceleration Z series

// Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

// Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
//                                       0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

// #if LV_USE_LOG != 0
// /* Serial debugging */
// void my_print(const char *buf) {
//   USBSerial.printf(buf);
//   USBSerial.flush();
// }
// #endif

// /* Display flushing */
// void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
// {
//     uint32_t w = (area->x2 - area->x1 + 1);
//     uint32_t h = (area->y2 - area->y1 + 1);

// #if (LV_COLOR_16_SWAP != 0)
//     gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #else
//     gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #endif

//     lv_disp_flush_ready(disp);
// }

// void example_increase_lvgl_tick(void *arg)
// {
//     /* Tell LVGL how many milliseconds has elapsed */
//     lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
// }

// static uint8_t count = 0;
// void example_increase_reboot(void *arg)
// {
//     count++;
//     if (count == 30)
//     {
//         esp_restart();
//     }
// }

// void setup()
// {
//     USBSerial.begin(115200); /* prepare for possible serial debug */
//     hardware_initialize();

//     // pinMode(LCD_EN, OUTPUT);
//     // digitalWrite(LCD_EN, HIGH);

//     gfx->begin();
//     pinMode(LCD_BL, OUTPUT);
//     pinMode(38, OUTPUT);
//     digitalWrite(LCD_BL, HIGH);
//     digitalWrite(38, HIGH);
//     String LVGL_Arduino = "Hello Arduino! ";
//     LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

//     USBSerial.println(LVGL_Arduino);
//     USBSerial.println("I am LVGL_Arduino");

//     lv_init();

// #if LV_USE_LOG != 0
//     lv_log_register_print_cb(my_print); /* register print function for debugging */
// #endif

//     lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

//     /*Initialize the display*/
//     static lv_disp_drv_t disp_drv;
//     lv_disp_drv_init(&disp_drv);
//     /*Change the following line to your display resolution*/
//     disp_drv.hor_res = LCD_WIDTH;
//     disp_drv.ver_res = LCD_HEIGHT;
//     disp_drv.flush_cb = my_disp_flush;
//     disp_drv.draw_buf = &draw_buf;
//     lv_disp_drv_register(&disp_drv);

//     /*Initialize the (dummy) input device driver*/
//     static lv_indev_drv_t indev_drv;
//     lv_indev_drv_init(&indev_drv);
//     indev_drv.type = LV_INDEV_TYPE_POINTER;
//     lv_indev_drv_register(&indev_drv);

//     lv_obj_t *label = lv_label_create(lv_scr_act());
//     lv_label_set_text(label, "Hello Ardino and LVGL!");
//     lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

//     const esp_timer_create_args_t lvgl_tick_timer_args = {
//         .callback = &example_increase_lvgl_tick,
//         .name = "lvgl_tick"};

//     const esp_timer_create_args_t reboot_timer_args = {
//         .callback = &example_increase_reboot,
//         .name = "reboot"};

//     esp_timer_handle_t lvgl_tick_timer = NULL;
//     esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
//     esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

//     label = lv_label_create(lv_scr_act());
//     lv_label_set_text(label, "Initializing...");
//     lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

//     /* Create chart */
//     chart = lv_chart_create(lv_scr_act());
//     lv_obj_set_size(chart, 240, 280);
//     lv_obj_align(chart, LV_ALIGN_CENTER, 0, 0);
//     lv_chart_set_type(chart, LV_CHART_TYPE_LINE);              /* Set the type to line */
//     lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -3, 3); /* Set the range of y axis */
//     lv_chart_set_point_count(chart, 20);                       /* Set the number of data points */
//     acc_series_x = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
//     acc_series_y = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
//     acc_series_z = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

//     USBSerial.println("Setup done");

//     if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL))
//     {

//         while (1)
//         {
//             delay(1000);
//         }
//     }

//     /* Get chip id */
//     USBSerial.println(qmi.getChipID());

//     qmi.configAccelerometer(
//         SensorQMI8658::ACC_RANGE_4G,
//         SensorQMI8658::ACC_ODR_1000Hz,
//         SensorQMI8658::LPF_MODE_0,
//         true);

//     qmi.configGyroscope(
//         SensorQMI8658::GYR_RANGE_64DPS,
//         SensorQMI8658::GYR_ODR_896_8Hz,
//         SensorQMI8658::LPF_MODE_3,
//         true);

//     qmi.enableGyroscope();
//     qmi.enableAccelerometer();

//     qmi.dumpCtrlRegister();

//     USBSerial.println("Read data now...");
// }

// void loop()
// {
//     lv_timer_handler(); /* let the GUI do its work */
//     delay(5);

//     if (qmi.getDataReady())
//     {
//         if (qmi.getAccelerometer(acc.x, acc.y, acc.z))
//         {
//             USBSerial.print("{ACCEL: ");
//             USBSerial.print(acc.x);
//             USBSerial.print(",");
//             USBSerial.print(acc.y);
//             USBSerial.print(",");
//             USBSerial.print(acc.z);
//             USBSerial.println("}");

//             // Update chart with new accelerometer data
//             lv_chart_set_next_value(chart, acc_series_x, acc.x);
//             lv_chart_set_next_value(chart, acc_series_y, acc.y);
//             lv_chart_set_next_value(chart, acc_series_z, acc.z);
//         }

//         if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z))
//         {
//             USBSerial.print("{GYRO: ");
//             USBSerial.print(gyr.x);
//             USBSerial.print(",");
//             USBSerial.print(gyr.y);
//             USBSerial.print(",");
//             USBSerial.print(gyr.z);
//             USBSerial.println("}");
//         }
//     }
//     delay(20); // Increase the frequency of data polling
// }





/**
 * @brief FUNCIONA: Medidor de tensao de bateria.
 */
// #include <Arduino.h>
// #include <lvgl.h>
// #include "Arduino_GFX_Library.h"
// #include "lv_conf.h"
// #include "demos/lv_demos.h"
// #include "pin_config.h"

// /* Using LVGL with Arduino requires some extra steps:
//  * Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

// #define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// /* Change to your screen resolution */
// static const uint16_t screenWidth = 240;
// static const uint16_t screenHeight = 280;

// static lv_disp_draw_buf_t draw_buf;
// static lv_color_t buf[screenWidth * screenHeight / 10];


// const int voltageDividerPin = 1;  // GPIO1 pin
// float vRef = 3.3;                 // ESP32-S3的供电电压（单位：伏特）
// float R1 = 200000.0;              // 第一个电阻的阻值（单位：欧姆）
// float R2 = 100000.0;              // 第二个电阻的阻值（单位：欧姆）

// lv_obj_t *label;  // Global label object

// Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

// Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
//                                       0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

// #if LV_USE_LOG != 0
// /* Serial debugging */
// void my_print(const char *buf) {
//   Serial.printf(buf);
//   Serial.flush();
// }
// #endif

// /* Display flushing */
// void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
//   uint32_t w = (area->x2 - area->x1 + 1);
//   uint32_t h = (area->y2 - area->y1 + 1);

// #if (LV_COLOR_16_SWAP != 0)
//   gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #else
//   gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #endif

//   lv_disp_flush_ready(disp);
// }

// void example_increase_lvgl_tick(void *arg) {
//   /* Tell LVGL how many milliseconds has elapsed */
//   lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
// }

// static uint8_t count = 0;
// void example_increase_reboot(void *arg) {
//   count++;
//   if (count == 30) {
//     esp_restart();
//   }
// }

// void setup() {
//   Serial.begin(115200); /* prepare for possible serial debug */
//   pinMode(voltageDividerPin, INPUT);

//   String LVGL_Arduino = "Hello Arduino! ";
//   LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

//   Serial.println(LVGL_Arduino);
//   Serial.println("I am LVGL_Arduino");

//   lv_init();

// #if LV_USE_LOG != 0
//   lv_log_register_print_cb(my_print); /* register print function for debugging */
// #endif

//   gfx->begin();
//   pinMode(LCD_BL, OUTPUT);
//   digitalWrite(LCD_BL, HIGH);

//   lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

//   /* Initialize the display */
//   static lv_disp_drv_t disp_drv;
//   lv_disp_drv_init(&disp_drv);
//   /* Change the following line to your display resolution */
//   disp_drv.hor_res = screenWidth;
//   disp_drv.ver_res = screenHeight;
//   disp_drv.flush_cb = my_disp_flush;
//   disp_drv.draw_buf = &draw_buf;
//   lv_disp_drv_register(&disp_drv);

//   const esp_timer_create_args_t lvgl_tick_timer_args = {
//     .callback = &example_increase_lvgl_tick,
//     .name = "lvgl_tick"
//   };

//   const esp_timer_create_args_t reboot_timer_args = {
//     .callback = &example_increase_reboot,
//     .name = "reboot"
//   };

//   esp_timer_handle_t lvgl_tick_timer = NULL;
//   esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
//   esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

//   // esp_timer_handle_t reboot_timer = NULL;
//   // esp_timer_create(&reboot_timer_args, &reboot_timer);
//   // esp_timer_start_periodic(reboot_timer, 2000 * 1000);

//   /* Create label */
//   label = lv_label_create(lv_scr_act());
//   lv_label_set_text(label, "Initializing...");
//   lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

//   Serial.println("Setup done");
// }

// void loop() {
//   lv_timer_handler(); /* let the GUI do its work */
//   delay(5);

//   // 读取ADC值
//   int adcValue = analogRead(voltageDividerPin);

//   // 转换为电压
//   float voltage = (float)adcValue * (vRef / 4095.0);

//   // 应用分压公式来计算实际电压
//   float actualVoltage = voltage * ((R1 + R2) / R2);

//   // 打印实际电压
//   Serial.print("Actual Voltage: ");
//   Serial.print(actualVoltage);
//   Serial.println(" V");

//   // 更新标签内容
//   String voltageStr = "Actual Voltage: " + String(actualVoltage) + " V";
//   lv_label_set_text(label, voltageStr.c_str());
// }









/**
 * @brief FUNCIONA: Relogio.
 */
// #include <Arduino.h>
// #include "Arduino_GFX_Library.h"
// #include "pin_config.h"
// #include <Wire.h>
// #include "HWCDC.h"

// HWCDC USBSerial;
// // SensorPCF85063 rtc;

// Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

// Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
//                                       0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

// #define BACKGROUND BLACK
// #define MARK_COLOR WHITE
// #define SUBMARK_COLOR DARKGREY // LIGHTGREY
// #define HOUR_COLOR WHITE
// #define MINUTE_COLOR BLUE // LIGHTGREY
// #define SECOND_COLOR RED

// #define SIXTIETH 0.016666667
// #define TWELFTH 0.08333333
// #define SIXTIETH_RADIAN 0.10471976
// #define TWELFTH_RADIAN 0.52359878
// #define RIGHT_ANGLE_RADIAN 1.5707963

// static uint8_t conv2d(const char *p)
// {
//   uint8_t v = 0;
//   return (10 * (*p - '0')) + (*++p - '0');
// }

// static int16_t w, h, center;
// static int16_t hHandLen, mHandLen, sHandLen, markLen;
// static float sdeg, mdeg, hdeg;
// static int16_t osx = 0, osy = 0, omx = 0, omy = 0, ohx = 0, ohy = 0; // Saved H, M, S x & y coords
// static int16_t nsx, nsy, nmx, nmy, nhx, nhy;                         // H, M, S x & y coords
// static int16_t xMin, yMin, xMax, yMax;                               // redraw range
// static int16_t hh, mm, ss;
// static unsigned long targetTime; // next action time

// static int16_t *cached_points;
// static uint16_t cached_points_idx = 0;
// static int16_t *last_cached_point;

// void draw_round_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3);
// void draw_square_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3);
// void redraw_hands_cached_draw_and_erase();
// void draw_and_erase_cached_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t color, int16_t *cache, int16_t cache_len, bool cross_check_second, bool cross_check_hour);
// void write_cache_pixel(int16_t x, int16_t y, int16_t color, bool cross_check_second, bool cross_check_hour);

// void setup(void)
// {
//   USBSerial.begin(115200);
//   USBSerial.println("Arduino_GFX Clock example");

//   // Init Display
//   if (!gfx->begin())
//   {
//     USBSerial.println("gfx->begin() failed!");
//   }
//   gfx->fillScreen(BACKGROUND);

//   pinMode(LCD_BL, OUTPUT);
//   digitalWrite(LCD_BL, HIGH);

//   // init LCD constant
//   w = gfx->width();
//   h = gfx->height();
//   if (w < h)
//   {
//     center = w / 2;
//   }
//   else
//   {
//     center = h / 2;
//   }
//   hHandLen = center * 3 / 8;
//   mHandLen = center * 2 / 3;
//   sHandLen = center * 5 / 6;
//   markLen = sHandLen / 6;
//   cached_points = (int16_t *)malloc((hHandLen + 1 + mHandLen + 1 + sHandLen + 1) * 2 * 2);

//   // Draw 60 clock marks
//   draw_round_clock_mark(
//       // draw_square_clock_mark(
//       center - markLen, center,
//       center - (markLen * 2 / 3), center,
//       center - (markLen / 2), center);

//   hh = conv2d(__TIME__);
//   mm = conv2d(__TIME__ + 3);
//   ss = conv2d(__TIME__ + 6);

//   targetTime = ((millis() / 1000) + 1) * 1000;
// }

// void loop()
// {
//   unsigned long cur_millis = millis();
//   if (cur_millis >= targetTime)
//   {
//     targetTime += 1000;
//     ss++; // Advance second
//     if (ss == 60)
//     {
//       ss = 0;
//       mm++; // Advance minute
//       if (mm > 59)
//       {
//         mm = 0;
//         hh++; // Advance hour
//         if (hh > 23)
//         {
//           hh = 0;
//         }
//       }
//     }
//   }

//   // Pre-compute hand degrees, x & y coords for a fast screen update
//   sdeg = SIXTIETH_RADIAN * ((0.001 * (cur_millis % 1000)) + ss); // 0-59 (includes millis)
//   nsx = cos(sdeg - RIGHT_ANGLE_RADIAN) * sHandLen + center;
//   nsy = sin(sdeg - RIGHT_ANGLE_RADIAN) * sHandLen + center;
//   if ((nsx != osx) || (nsy != osy))
//   {
//     mdeg = (SIXTIETH * sdeg) + (SIXTIETH_RADIAN * mm); // 0-59 (includes seconds)
//     hdeg = (TWELFTH * mdeg) + (TWELFTH_RADIAN * hh);   // 0-11 (includes minutes)
//     mdeg -= RIGHT_ANGLE_RADIAN;
//     hdeg -= RIGHT_ANGLE_RADIAN;
//     nmx = cos(mdeg) * mHandLen + center;
//     nmy = sin(mdeg) * mHandLen + center;
//     nhx = cos(hdeg) * hHandLen + center;
//     nhy = sin(hdeg) * hHandLen + center;

//     // redraw hands
//     redraw_hands_cached_draw_and_erase();

//     ohx = nhx;
//     ohy = nhy;
//     omx = nmx;
//     omy = nmy;
//     osx = nsx;
//     osy = nsy;

//     delay(1);
//   }
// }

// void draw_round_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3)
// {
//   float x, y;
//   int16_t x0, x1, y0, y1, innerR, outerR;
//   uint16_t c;

//   for (uint8_t i = 0; i < 60; i++)
//   {
//     if ((i % 15) == 0)
//     {
//       innerR = innerR1;
//       outerR = outerR1;
//       c = MARK_COLOR;
//     }
//     else if ((i % 5) == 0)
//     {
//       innerR = innerR2;
//       outerR = outerR2;
//       c = MARK_COLOR;
//     }
//     else
//     {
//       innerR = innerR3;
//       outerR = outerR3;
//       c = SUBMARK_COLOR;
//     }

//     mdeg = (SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN;
//     x = cos(mdeg);
//     y = sin(mdeg);
//     x0 = x * outerR + center;
//     y0 = y * outerR + center;
//     x1 = x * innerR + center;
//     y1 = y * innerR + center;

//     gfx->drawLine(x0, y0, x1, y1, c);
//   }
// }


// void draw_square_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3)
// {
//   float x, y;
//   int16_t x0, x1, y0, y1, innerR, outerR;
//   uint16_t c;

//   for (uint8_t i = 0; i < 60; i++)
//   {
//     if ((i % 15) == 0)
//     {
//       innerR = innerR1;
//       outerR = outerR1;
//       c = MARK_COLOR;
//     }
//     else if ((i % 5) == 0)
//     {
//       innerR = innerR2;
//       outerR = outerR2;
//       c = MARK_COLOR;
//     }
//     else
//     {
//       innerR = innerR3;
//       outerR = outerR3;
//       c = SUBMARK_COLOR;
//     }

//     if ((i >= 53) || (i < 8))
//     {
//       x = tan(SIXTIETH_RADIAN * i);
//       x0 = center + (x * outerR);
//       y0 = center + (1 - outerR);
//       x1 = center + (x * innerR);
//       y1 = center + (1 - innerR);
//     }
//     else if (i < 23)
//     {
//       y = tan((SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN);
//       x0 = center + (outerR);
//       y0 = center + (y * outerR);
//       x1 = center + (innerR);
//       y1 = center + (y * innerR);
//     }
//     else if (i < 38)
//     {
//       x = tan(SIXTIETH_RADIAN * i);
//       x0 = center - (x * outerR);
//       y0 = center + (outerR);
//       x1 = center - (x * innerR);
//       y1 = center + (innerR);
//     }
//     else if (i < 53)
//     {
//       y = tan((SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN);
//       x0 = center + (1 - outerR);
//       y0 = center - (y * outerR);
//       x1 = center + (1 - innerR);
//       y1 = center - (y * innerR);
//     }
//     gfx->drawLine(x0, y0, x1, y1, c);
//   }
// }


// void redraw_hands_cached_draw_and_erase()
// {
//   gfx->startWrite();
//   draw_and_erase_cached_line(center, center, nsx, nsy, SECOND_COLOR, cached_points, sHandLen + 1, false, false);
//   draw_and_erase_cached_line(center, center, nhx, nhy, HOUR_COLOR, cached_points + ((sHandLen + 1) * 2), hHandLen + 1, true, false);
//   draw_and_erase_cached_line(center, center, nmx, nmy, MINUTE_COLOR, cached_points + ((sHandLen + 1 + hHandLen + 1) * 2), mHandLen + 1, true, true);
//   gfx->endWrite();
// }

// void draw_and_erase_cached_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t color, int16_t *cache, int16_t cache_len, bool cross_check_second, bool cross_check_hour)
// {
// #if defined(ESP8266)
//   yield();
// #endif
//   bool steep = _diff(y1, y0) > _diff(x1, x0);
//   if (steep)
//   {
//     _swap_int16_t(x0, y0);
//     _swap_int16_t(x1, y1);
//   }

//   int16_t dx, dy;
//   dx = _diff(x1, x0);
//   dy = _diff(y1, y0);

//   int16_t err = dx / 2;
//   int8_t xstep = (x0 < x1) ? 1 : -1;
//   int8_t ystep = (y0 < y1) ? 1 : -1;
//   x1 += xstep;
//   int16_t x, y, ox, oy;
//   for (uint16_t i = 0; i <= dx; i++)
//   {
//     if (steep)
//     {
//       x = y0;
//       y = x0;
//     }
//     else
//     {
//       x = x0;
//       y = y0;
//     }
//     ox = *(cache + (i * 2));
//     oy = *(cache + (i * 2) + 1);
//     if ((x == ox) && (y == oy))
//     {
//       if (cross_check_second || cross_check_hour)
//       {
//         write_cache_pixel(x, y, color, cross_check_second, cross_check_hour);
//       }
//     }
//     else
//     {
//       write_cache_pixel(x, y, color, cross_check_second, cross_check_hour);
//       if ((ox > 0) || (oy > 0))
//       {
//         write_cache_pixel(ox, oy, BACKGROUND, cross_check_second, cross_check_hour);
//       }
//       *(cache + (i * 2)) = x;
//       *(cache + (i * 2) + 1) = y;
//     }
//     if (err < dy)
//     {
//       y0 += ystep;
//       err += dx;
//     }
//     err -= dy;
//     x0 += xstep;
//   }
//   for (uint16_t i = dx + 1; i < cache_len; i++)
//   {
//     ox = *(cache + (i * 2));
//     oy = *(cache + (i * 2) + 1);
//     if ((ox > 0) || (oy > 0))
//     {
//       write_cache_pixel(ox, oy, BACKGROUND, cross_check_second, cross_check_hour);
//     }
//     *(cache + (i * 2)) = 0;
//     *(cache + (i * 2) + 1) = 0;
//   }
// }



// void write_cache_pixel(int16_t x, int16_t y, int16_t color, bool cross_check_second, bool cross_check_hour)
// {
//   int16_t *cache = cached_points;
//   if (cross_check_second)
//   {
//     for (uint16_t i = 0; i <= sHandLen; i++)
//     {
//       if ((x == *(cache++)) && (y == *(cache)))
//       {
//         return;
//       }
//       cache++;
//     }
//   }
//   if (cross_check_hour)
//   {
//     cache = cached_points + ((sHandLen + 1) * 2);
//     for (uint16_t i = 0; i <= hHandLen; i++)
//     {
//       if ((x == *(cache++)) && (y == *(cache)))
//       {
//         return;
//       }
//       cache++;
//     }
//   }
//   gfx->writePixel(x, y, color);
// }





/**
 * @brief FUNCIONA: RTC.
 */
// #include <lvgl.h>
// #include "Arduino_GFX_Library.h"
// #include "pin_config.h"
// #include "lv_conf.h"
// #include <Wire.h>
// #include <SPI.h>
// #include <Arduino.h>
// #include "SensorPCF85063.hpp"
// #include "HWCDC.h"

// HWCDC USBSerial;
// #define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// static lv_disp_draw_buf_t draw_buf;
// static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

// lv_obj_t *label;  // Global label object
// SensorPCF85063 rtc;
// uint32_t lastMillis;

// Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

// Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
//                                       0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);
// #if LV_USE_LOG != 0
// /* Serial debugging */
// void my_print(const char *buf) {
//   USBSerial.printf(buf);
//   USBSerial.flush();
// }
// #endif


// void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
// void example_increase_lvgl_tick(void *arg);
// void example_increase_reboot(void *arg);

// /* Display flushing */
// void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
//   uint32_t w = (area->x2 - area->x1 + 1);
//   uint32_t h = (area->y2 - area->y1 + 1);

// #if (LV_COLOR_16_SWAP != 0)
//   gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #else
//   gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
// #endif

//   lv_disp_flush_ready(disp);
// }

// void example_increase_lvgl_tick(void *arg) {
//   /* Tell LVGL how many milliseconds has elapsed */
//   lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
// }

// static uint8_t count = 0;
// void example_increase_reboot(void *arg) {
//   count++;
//   if (count == 30) {
//     esp_restart();
//   }
// }

// void setup() {
//   USBSerial.begin(115200); /* prepare for possible serial debug */
//   if (!rtc.begin(Wire, PCF85063_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
//     USBSerial.println("Failed to find PCF8563 - check your wiring!");
//     while (1) {
//       delay(1000);
//     }
//   }

//   uint16_t year = 2024;
//   uint8_t month = 9;
//   uint8_t day = 24;
//   uint8_t hour = 11;
//   uint8_t minute = 9;
//   uint8_t second = 41;

//   rtc.setDateTime(year, month, day, hour, minute, second);


//   gfx->begin();
//   pinMode(LCD_BL, OUTPUT);
//   digitalWrite(LCD_BL, HIGH);

//   String LVGL_Arduino = "Hello Arduino! ";
//   LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

//   USBSerial.println(LVGL_Arduino);
//   USBSerial.println("I am LVGL_Arduino");

//   lv_init();

// #if LV_USE_LOG != 0
//   lv_log_register_print_cb(my_print); /* register print function for debugging */
// #endif


//   lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

//   /*Initialize the display*/
//   static lv_disp_drv_t disp_drv;
//   lv_disp_drv_init(&disp_drv);
//   /*Change the following line to your display resolution*/
//   disp_drv.hor_res = LCD_WIDTH;
//   disp_drv.ver_res = LCD_HEIGHT;
//   disp_drv.flush_cb = my_disp_flush;
//   disp_drv.draw_buf = &draw_buf;
//   lv_disp_drv_register(&disp_drv);

//   /*Initialize the (dummy) input device driver*/
//   static lv_indev_drv_t indev_drv;
//   lv_indev_drv_init(&indev_drv);
//   indev_drv.type = LV_INDEV_TYPE_POINTER;
//   lv_indev_drv_register(&indev_drv);

//   // lv_obj_t *label = lv_label_create(lv_scr_act());
//   // lv_label_set_text(label, "Hello Ardino and LVGL!");
//   // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

//   const esp_timer_create_args_t lvgl_tick_timer_args = {
//     .callback = &example_increase_lvgl_tick,
//     .name = "lvgl_tick"
//   };

//   const esp_timer_create_args_t reboot_timer_args = {
//     .callback = &example_increase_reboot,
//     .name = "reboot"
//   };

//   esp_timer_handle_t lvgl_tick_timer = NULL;
//   esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
//   esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

//   label = lv_label_create(lv_scr_act());
//   lv_label_set_text(label, "Initializing...");
//   lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
// }

// void loop() {
//   lv_timer_handler(); /* let the GUI do its work */
//   delay(5);

//   if (millis() - lastMillis > 1000) {
//     lastMillis = millis();
//     RTC_DateTime datetime = rtc.getDateTime();
//     USBSerial.printf(" Year :");
//     USBSerial.print(datetime.year);
//     USBSerial.printf(" Month:");
//     USBSerial.print(datetime.month);
//     USBSerial.printf(" Day :");
//     USBSerial.print(datetime.day);
//     USBSerial.printf(" Hour:");
//     USBSerial.print(datetime.hour);
//     USBSerial.printf(" Minute:");
//     USBSerial.print(datetime.minute);
//     USBSerial.printf(" Sec :");
//     USBSerial.println(datetime.second);

//     char buf[32];
//     snprintf(buf, sizeof(buf), "%02d:%02d:%02d\n%02d-%02d-%04d",
//              datetime.hour, datetime.minute, datetime.second,
//              datetime.day, datetime.month, datetime.year);

//     // Update label with current time
//     lv_label_set_text(label, buf);
//     lv_obj_set_style_text_font(label, &lv_font_montserrat_40, LV_PART_MAIN);
//   }
//   delay(20);
// }
