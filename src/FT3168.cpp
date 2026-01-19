#include "FT3168.h"

#include "lcd_config.h"

// Use Arduino I2C (TwoWire) to avoid mixing ESP-IDF legacy I2C with driver_ng.
static TwoWire *s_touchWire = &Wire;

static uint8_t I2C_writr_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  s_touchWire->beginTransmission(addr);
  s_touchWire->write(reg);
  if (buf && len) {
    s_touchWire->write(buf, len);
  }
  uint8_t err = s_touchWire->endTransmission(true); // 0 == success
  return err;
}

static uint8_t I2C_read_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  if (!buf || !len) return 0;

  s_touchWire->beginTransmission(addr);
  s_touchWire->write(reg);
  // Use STOP here to avoid the "NonStop" path (i2cWriteReadNonStop) which can return INVALID_STATE.
  uint8_t err = s_touchWire->endTransmission(true);
  if (err != 0) {
    return err;
  }

  size_t readLen = s_touchWire->requestFrom((int)addr, (int)len);
  if (readLen != len) {
    return 1;
  }
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = (uint8_t)s_touchWire->read();
  }
  return 0;
}

void Touch_Init(void) {
  // Wire is initialized in setup() (main.cpp). Avoid (re)initializing I2C here
  // because it can invalidate other I2C instances (e.g. Wire1) on some cores.
  s_touchWire = &Wire;

  // Wait for touch controller to be ready after power-on
  delay(50);
  
  // Try to initialize the touch controller with retries
  uint8_t data = 0x00;
  uint8_t retries = 5;
  uint8_t err = 0xFF;
  
  while (retries > 0 && err != 0) {
    err = I2C_writr_buff(I2C_ADDR_FT3168, 0x00, &data, 1); // Switch to normal mode
    if (err != 0) {
      delay(20); // Wait before retry
      retries--;
    }
  }
  
  if (err == 0) {
    delay(50); // Additional delay after successful init to let controller stabilize
  } else {
    Serial.printf("FT3168: Touch init failed after retries (error: %d)\n", err);
  }
}

uint8_t getTouch(uint16_t *x, uint16_t *y) {
  uint8_t data;
  uint8_t buf[4];
  I2C_read_buff(I2C_ADDR_FT3168, 0x02, &data, 1);
  if (data) {
    I2C_read_buff(I2C_ADDR_FT3168, 0x03, buf, 4);
    *x = (((uint16_t)buf[0] & 0x0f) << 8) | (uint16_t)buf[1];
    *y = (((uint16_t)buf[2] & 0x0f) << 8) | (uint16_t)buf[3];
    if (*x > EXAMPLE_LCD_H_RES) *x = EXAMPLE_LCD_H_RES;
    if (*y > EXAMPLE_LCD_V_RES) *y = EXAMPLE_LCD_V_RES;
    return 1;
  }
  return 0;
}


