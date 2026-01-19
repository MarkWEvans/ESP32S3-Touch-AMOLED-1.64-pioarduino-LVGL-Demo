#include "BatteryManager.h"
#include <Wire.h>
#include <esp_log.h>

BatteryManager g_battery_mgr;

BatteryManager::BatteryManager()
  : m_ina226(INA226_I2C_ADDR, &Wire1)
  , m_ready(false)
  , m_battery_utils(1) // 1S cell
  , m_spinlock(portMUX_INITIALIZER_UNLOCKED)
  , m_bus_mv(0)
  , m_cur_ma(0)
  , m_valid(false)
{
}

BatteryManager::~BatteryManager() {
  // Task cleanup handled by FreeRTOS
}

bool BatteryManager::begin() {
  // Initialize the INA226 on I2C bus (Wire1)
  // Note: Wire1 must be initialized before calling begin()
  m_ready = m_ina226.begin();
  if (!m_ready) {
    Serial.println("BatteryManager: Failed to find INA226 chip");
    return false;
  }

  Serial.println("BatteryManager: INA226 initialized successfully");
  delay(10); // Small delay to ensure device is ready
  
  // Configure INA226 with shunt resistor and max current
  Serial.printf("BatteryManager: Attempting calibration with %.2fA max, %.3f ohm shunt\n", 
                INA226_MAX_CURRENT, INA226_SHUNT_RESISTOR);
  uint16_t err = m_ina226.setMaxCurrentShunt(INA226_MAX_CURRENT, INA226_SHUNT_RESISTOR);
  if (err == 0x0000) {
    Serial.printf("BatteryManager: Configured %.2fA max, %.3f ohm shunt\n", 
                  INA226_MAX_CURRENT, INA226_SHUNT_RESISTOR);
    // Verify calibration was set
    if (m_ina226.isCalibrated()) {
      Serial.printf("BatteryManager: Calibration verified. Current LSB: %.6fA (%.3fmA)\n", 
                    m_ina226.getCurrentLSB(), m_ina226.getCurrentLSB_mA());
    } else {
      Serial.println("BatteryManager: WARNING: Calibration not verified!");
    }
  } else {
    Serial.printf("BatteryManager: Calibration error 0x%04X\n", err);
    m_ready = false;
    return false;
  }

  // Start background polling task (2 Hz)
  xTaskCreate(task, "BatteryMgr", 4096, this, 1, NULL);
  
  return true;
}

bool BatteryManager::getReadings(int32_t *bus_mv, int32_t *cur_ma) const {
  if (!bus_mv || !cur_ma) return false;
  portENTER_CRITICAL(&m_spinlock);
  bool ok = m_valid;
  int32_t mv = m_bus_mv;
  int32_t ma = m_cur_ma;
  portEXIT_CRITICAL(&m_spinlock);
  if (!ok) return false;
  *bus_mv = mv;
  *cur_ma = ma;
  return true;
}

float BatteryManager::getBatteryPercentage(float voltage_v) {
  return m_battery_utils.calculateBatteryPercentage(voltage_v);
}

bool BatteryManager::isUsbPower() const {
  // Check INA226 voltage threshold (USB typically > 4.5V, battery < 4.2V)
  int32_t bus_mv = 0;
  int32_t cur_ma = 0;
  if (getReadings(&bus_mv, &cur_ma)) {
    float voltage_v = (float)bus_mv / 1000.0f;
    // USB power typically provides > 4.5V, battery max is ~4.2V
    if (voltage_v > 4.5f) {
      return true;
    }
  }

  // Fallback: Check if USB Serial is connected (ESP32-S3 specific)
  // This is a heuristic - USB CDC connection suggests USB power
  #ifdef ARDUINO_USB_CDC_ON_BOOT
    if (Serial) {
      return true;
    }
  #endif

  return false;
}

bool BatteryManager::ping() const {
  Wire1.beginTransmission(INA226_I2C_ADDR);
  return Wire1.endTransmission(true) == 0;
}

void BatteryManager::task(void *arg) {
  BatteryManager *self = static_cast<BatteryManager*>(arg);
  self->taskLoop();
}

void BatteryManager::taskLoop() {
  uint32_t consecutive_failures = 0;

  for (;;) {
    if (m_ready) {
      // Avoid hammering the bus if the device disappears or the bus glitches
      if (!ping()) {
        consecutive_failures++;
        if (consecutive_failures == 1 || (consecutive_failures % 10) == 0) {
          Serial.printf("BatteryManager: I2C ping failed (%lu)\n", (unsigned long)consecutive_failures);
        }
        // After a few consecutive failures, treat it as missing and stop reading until re-init
        if (consecutive_failures >= 3) {
          m_ready = false;
        }
        portENTER_CRITICAL(&m_spinlock);
        m_valid = false;
        portEXIT_CRITICAL(&m_spinlock);
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }

      consecutive_failures = 0;

      float v = m_ina226.getBusVoltage();
      float i_ma = m_ina226.getCurrent_mA(); // Returns current in milliamps

      int32_t bus_mv = (int32_t)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
      int32_t cur_ma = (int32_t)(i_ma + (i_ma >= 0.0f ? 0.5f : -0.5f));

      portENTER_CRITICAL(&m_spinlock);
      m_bus_mv = bus_mv;
      m_cur_ma = cur_ma;
      m_valid = true;
      portEXIT_CRITICAL(&m_spinlock);
    } else {
      // If sensor isn't ready, mark data invalid
      portENTER_CRITICAL(&m_spinlock);
      m_valid = false;
      portEXIT_CRITICAL(&m_spinlock);

      // Periodically try to bring the INA226 back if it reappears on the bus
      static uint32_t last_retry_ms = 0;
      const uint32_t now = millis();
      if (now - last_retry_ms > 5000) {
        last_retry_ms = now;
        if (ping()) {
          Serial.println("BatteryManager: device ACKed again, re-initializing...");
          m_ready = m_ina226.begin();
          if (m_ready) {
            // Re-configure INA226 with shunt resistor and max current
            uint16_t err = m_ina226.setMaxCurrentShunt(INA226_MAX_CURRENT, INA226_SHUNT_RESISTOR);
            if (err == 0x0000) {
              Serial.println("BatteryManager: re-init OK");
              if (m_ina226.isCalibrated()) {
                Serial.printf("BatteryManager: re-calibrated. Current LSB: %.6fA\n", m_ina226.getCurrentLSB());
              }
              consecutive_failures = 0;
            } else {
              Serial.printf("BatteryManager: calibration error 0x%04X\n", err);
            }
          } else {
            Serial.println("BatteryManager: re-init failed");
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // 2x per second
  }
}
