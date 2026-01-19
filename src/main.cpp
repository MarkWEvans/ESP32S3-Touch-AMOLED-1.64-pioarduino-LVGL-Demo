#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "pin_config.h"
#include "lcd_bsp.h"
#include "FT3168.h"
#include "lcd_config.h"
#include "WifiManager.h"
#include "lvgl.h"
#include "ui.h"
#include "ServoManager.h"
#include "CalibrationManager.h"
#include "BatteryManager.h"
#include "esp_log.h"
#include "gui_updates.h"
#include "esp_task_wdt.h"
#include "esp_err.h"

// Global managers and state (accessible from gui_updates.cpp)
ServoManager g_servo_mgr;
CalibrationManager g_calib_mgr;

void resetServoBus() {
  g_servo_mgr.resetBus(SCSERVO_TX, SCSERVO_RX, SCSERVO_BAUDRATE, 50);
}
 
void setup() {
  Serial.begin(115200);

  // Disable task watchdog timer to prevent timeouts during LVGL operations
  // Note: This removes protection against hung tasks, but allows long-running UI operations
  esp_task_wdt_deinit();
  Serial.println("Task watchdog disabled");

  // Mask noisy low-level I2C driver logs like:
  //   E (xxxx) i2c.master: ...
  // Note: this does NOT fix the underlying I2C issue, it only hides the log spam.
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  // Start WiFi + mDNS in the background (status is reflected in the LVGL UI)
  wifi_manager_init();

  // Init touch I2C bus early (before starting other tasks / other I2C instances)
  Wire.begin(EXAMPLE_PIN_NUM_TOUCH_SDA, EXAMPLE_PIN_NUM_TOUCH_SCL, 100000);
  Wire.setTimeOut(100);
  delay(300);

  // INA226 on I2C bus (Wire1) on pins 1/2
  Wire1.begin(INA_I2C_SDA, INA_I2C_SCL, 100000);
  Wire1.setTimeOut(10);

  // Initialize BatteryManager (handles INA226 initialization and background task)
  g_battery_mgr.begin();
  
  Serial1.begin(SCSERVO_BAUDRATE, SERIAL_8N1, SCSERVO_RX, SCSERVO_TX);
  //while (!Serial1) {
  //}
  // Start ServoManager (status + updates). This binds its internal SCSCL to Serial1.
  g_servo_mgr.begin(&Serial1, 15, 10);
  g_servo_mgr.startUpdateTask();

  // Initialize CalibrationManager (sets up callbacks on ServoManager)
  g_calib_mgr.begin(&g_servo_mgr);

  delay(100); // Allow touch controller to initialize and stabilize after power-on
  Touch_Init();
  delay(300);
  lcd_lvgl_Init();
}

void loop() {
  // LVGL runs in its own task (see lcd_bsp.c)
}
