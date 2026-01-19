#ifndef FT3168_H
#define FT3168_H

#include <Arduino.h>
#include <Wire.h>

void Touch_Init(void);
uint8_t getTouch(uint16_t *x, uint16_t *y);

#endif


