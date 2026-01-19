#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include <stdint.h>
#include <stddef.h>

#include <array>
#include <string>
#include <unordered_map>

#include "SCSCL.h"

struct ServoConfig {
  int id;
  char name[8];
  char description[32];
  bool reverse;
  float zeroAngle;
  float minAngle;
  float maxAngle;
  int minPos;
  int maxPos;
};

struct ServoStatus {
  bool connected = false;
  float angle = 0.0f;
  int position = -1;
  float voltage = 0.0f;
  float load = 0.0f;
  float temperature = 0.0f;
  float speed = 0.0f;
  bool is_moving = false;
  bool temperature_error = false;
  bool overload_error = false;
  float targetAngle = 0.0f;
  int targetPosition = -1;
  bool needToUpdate = false;
  bool torque_enable = false; // current state
  bool needToDisable = false;
  bool needToEnable = false;
};

// Calibration phase configuration (table-driven calibration)
struct CalibPhase {
  static constexpr int SKIP = -1;        // Don't move this servo
  static constexpr int POS_MIN = 60;     // Raw servo minimum position
  static constexpr int POS_MAX = 1000;   // Raw servo maximum position

  const char* name;          // Phase name for logging
  int targets[12];           // Target position per servo index (-1 = skip)
  bool sample_min;           // After settling, sample positions as minPos
  bool sample_max;           // After settling, sample positions as maxPos
  uint16_t speed;            // Movement speed parameter
  uint32_t timeout_ms;       // Max time to wait for all servos to settle
};

class ServoManager {
public:
  static constexpr int kServoIdOffset = 2;
  static constexpr uint32_t kDefaultUpdateRateMs = 100; // movement time + task cadence default

  ServoManager();

  // Attach to the bus. Call after Serial1.begin().
  // sc_timeout is the IO timeout used by SCSerial/SCSCL (matches sc.begin(serial, timeout)).
  void begin(HardwareSerial *serial,
             unsigned long sc_timeout = 15,
             uint32_t update_rate_ms = kDefaultUpdateRateMs);

  // Config persistence (Preferences namespace "servo_config")
  bool loadServoConfigs();
  void saveServoConfigs();

  // Access config/status
  size_t servoCount() const { return _servoCount; }
  const ServoConfig *configs() const { return _configs.data(); }
  ServoConfig *configs() { return _configs.data(); }
  const ServoStatus *statuses() const { return _statuses.data(); }
  ServoStatus *statuses() { return _statuses.data(); }

  // Name → index lookup (populated from configs on begin/load)
  const std::unordered_map<std::string, int> &jointNameToIndex() const { return _jointNameToIndex; }

  ServoConfig *findServoById(int id);
  int indexForId(int id) const;

  // Conversions
  static float posToAngle(int pos, int minPos, int maxPos, float minAngle, float maxAngle, bool reverse);
  static int angleToPos(float angle, const ServoConfig &config);

  // Commands (set flags consumed by update task)
  bool setTargetAngle(int id, float angle);
  bool setTargetPosition(int id, int pos);
  bool requestTorqueEnable(int id, bool enable);

  // One-off operations
  void resetBus(uint8_t pin_tx, uint8_t pin_rx, uint32_t baud, uint32_t idle_delay_ms = 50);

  // Background tasks
  void startUpdateTask();
  void stopUpdateTask();
  void startWiggleTask(int id, uint32_t duration_ms = 3000, int delta_pos = 15);
  void startScanTask(uint8_t id_min = 1, uint8_t id_max = 20, bool skip_leg_ids = true);
  void startCalibrateTask(int id);
  void startCalibrateAllTask(bool only_connected = true);
  bool isCalibrationRunning() const { return _calib_task != nullptr; }

  // Callbacks for scan/wiggle results (optional)
  typedef void (*scan_result_cb_t)(const uint8_t *ids, size_t count, void *user);
  typedef void (*wiggle_result_cb_t)(int id, bool success, void *user);
  enum calib_stage_t : uint8_t {
    CALIB_ALL_BEGIN = 0,
    CALIB_SERVO_BEGIN = 1,
    CALIB_SERVO_OK = 2,
    CALIB_SERVO_FAIL = 3,
    CALIB_ALL_DONE = 4,
  };
  typedef void (*calib_progress_cb_t)(int id, calib_stage_t stage, void *user);
  typedef void (*calib_position_cb_t)(int id, int position, void *user);
  typedef void (*calib_phase_cb_t)(const char* phase_name, void *user);
  void setScanResultCallback(scan_result_cb_t cb, void *user) { _scan_cb = cb; _scan_user = user; }
  void setWiggleResultCallback(wiggle_result_cb_t cb, void *user) { _wiggle_cb = cb; _wiggle_user = user; }
  void setCalibProgressCallback(calib_progress_cb_t cb, void *user) { _calib_cb = cb; _calib_user = user; }
  void setCalibPositionCallback(calib_position_cb_t cb, void *user) { _calib_pos_cb = cb; _calib_pos_user = user; }
  void setCalibPhaseCallback(calib_phase_cb_t cb, void *user) { _calib_phase_cb = cb; _calib_phase_user = user; }

  // Timing stats
  uint32_t lastUpdateCycleMs() const { return _last_update_ms; }

  // Pause updates while calibrating or doing exclusive operations
  void setCalibrationActive(bool active) { _calibration_active = active; }
  bool calibrationActive() const { return _calibration_active; }

private:
  void rebuildNameIndex();
  void updateOnce();

  static void update_task_thunk(void *arg);
  static void wiggle_task_thunk(void *arg);
  static void scan_task_thunk(void *arg);
  static void calibrate_task_thunk(void *arg);

  struct WiggleParams {
    ServoManager *self;
    int id;
    uint32_t duration_ms;
    int delta_pos;
  };
  struct ScanParams {
    ServoManager *self;
    uint8_t id_min;
    uint8_t id_max;
    bool skip_leg_ids;
  };
  struct CalibrateParams {
    ServoManager *self;
    int id;
    bool all;
    bool only_connected;
  };

  bool calibrateServo_locked(int id);
  void calibrateAll_locked(bool only_connected);
  bool executeCalibPhase(const CalibPhase& phase, bool only_connected);

  // Dependencies
  HardwareSerial *_serial = nullptr;
  SCSCL _sc;

  // Preferences
  Preferences _prefs;

  // Config/status storage
  static constexpr size_t kServoCount = 12;
  size_t _servoCount = kServoCount;
  std::array<ServoConfig, kServoCount> _configs{};
  std::array<ServoStatus, kServoCount> _statuses{};

  std::unordered_map<std::string, int> _jointNameToIndex;

  // Task/mutex
  SemaphoreHandle_t _bus_mutex = nullptr;
  TaskHandle_t _update_task = nullptr;
  TaskHandle_t _calib_task = nullptr;
  uint32_t _update_rate_ms = kDefaultUpdateRateMs;
  volatile bool _calibration_active = false;

  // Stats
  uint32_t _last_update_ms = 0;

  // Callbacks
  scan_result_cb_t _scan_cb = nullptr;
  void *_scan_user = nullptr;
  wiggle_result_cb_t _wiggle_cb = nullptr;
  void *_wiggle_user = nullptr;
  calib_progress_cb_t _calib_cb = nullptr;
  void *_calib_user = nullptr;
  calib_position_cb_t _calib_pos_cb = nullptr;
  void *_calib_pos_user = nullptr;
  calib_phase_cb_t _calib_phase_cb = nullptr;
  void *_calib_phase_user = nullptr;
};


