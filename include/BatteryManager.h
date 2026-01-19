#pragma once

#include <Arduino.h>
#include <INA226.h>
#include "BatteryUtils.h"
#include "pin_config.h"
#include <FreeRTOS.h>
#include <task.h>

/**
 * BatteryManager - Manages INA226 current sensor and battery measurements
 * 
 * Handles:
 * - INA226 initialization and calibration
 * - Background task for reading voltage/current (2 Hz)
 * - Cached readings access (thread-safe)
 * - Battery percentage calculation (1S cell)
 * - USB power detection
 */
class BatteryManager {
public:
  BatteryManager();
  ~BatteryManager();

  /**
   * Initialize INA226 sensor and start background task
   * Must be called after Wire1.begin()
   * @return true if initialization successful
   */
  bool begin();

  /**
   * Get latest voltage and current readings (thread-safe)
   * @param bus_mv Output: bus voltage in millivolts
   * @param cur_ma Output: current in milliamps
   * @return true if valid readings available
   */
  bool getReadings(int32_t *bus_mv, int32_t *cur_ma) const;

  /**
   * Get battery percentage (0-100) based on voltage
   * Note: This method modifies internal state (smoothing buffer)
   * @param voltage_v Voltage in volts
   * @return Battery percentage (0-100)
   */
  float getBatteryPercentage(float voltage_v);

  /**
   * Check if USB power is being used (vs battery)
   * @return true if USB power detected
   */
  bool isUsbPower() const;

  /**
   * Check if INA226 is ready and providing valid readings
   * @return true if sensor is ready
   */
  bool isReady() const { return m_ready; }

private:
  INA226 m_ina226;
  bool m_ready;
  BatteryUtils m_battery_utils;
  
  // Thread-safe cached readings
  mutable portMUX_TYPE m_spinlock;
  int32_t m_bus_mv;
  int32_t m_cur_ma;
  bool m_valid;

  // Background task
  static void task(void *arg);
  void taskLoop();

  // Helper: ping INA226 on I2C bus
  bool ping() const;
};

// Global instance (defined in BatteryManager.cpp)
extern BatteryManager g_battery_mgr;
