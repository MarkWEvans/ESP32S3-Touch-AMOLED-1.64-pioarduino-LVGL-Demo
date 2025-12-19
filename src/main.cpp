#include <Arduino.h>

#include "lcd_bsp.h"
#include "FT3168.h"

void setup() {
  Serial.begin(115200);

  Touch_Init();
  lcd_lvgl_Init();
}

void loop() {
  // LVGL runs in its own task (see lcd_bsp.c)
}


