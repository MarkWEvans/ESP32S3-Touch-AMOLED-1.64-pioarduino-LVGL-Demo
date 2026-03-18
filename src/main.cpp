#include <Arduino.h>

#include "lcd_bsp.h"

void setup() {
  Serial.begin(115200);
  lcd_lvgl_Init();
}

void loop() {
  // LVGL runs in its own task (see lcd_bsp.cpp)
}
