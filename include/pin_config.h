#pragma once

/*********************** pins *************************/
#define INA_I2C_SDA  2
#define INA_I2C_SCL  1
// INA226 I2C address (default 0x40, can be 0x41, 0x44, or 0x45 depending on A0/A1 pins)
#define INA226_I2C_ADDR 0x40
// INA226 shunt resistor value in ohms
#define INA226_SHUNT_RESISTOR 0.05f
// INA226 maximum expected current in Amps (for calibration)
#define INA226_MAX_CURRENT 1.5f

#define SCSERVO_BAUDRATE 1000000
#define SCSERVO_RX 5
#define SCSERVO_TX 3

// Battery voltage ADC pin (ESP32S3 ADC1)
#define BATTERY_ADC_PIN 4
// Voltage divider ratio: actual_voltage = adc_voltage * BATTERY_VOLTAGE_DIVIDER_RATIO
// If no divider, use 1.0. For 2:1 divider, use 2.0, etc.
#define BATTERY_VOLTAGE_DIVIDER_RATIO 2.0f

/*********************** WiFi *************************/
// Replace with your network credentials
#define WIFI_SSID "TP-Link_CA98"
#define WIFI_PASSWORD "76532474"

#define MDNS_NAME "quadrupedrobot"
/******************************************************/
