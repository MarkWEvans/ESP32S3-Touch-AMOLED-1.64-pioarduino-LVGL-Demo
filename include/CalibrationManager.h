#pragma once

#include <Arduino.h>
#include "ServoManager.h"

class CalibrationManager {
public:
  static constexpr size_t kMaxServos = 12;

  CalibrationManager();

  // Call once in setup() after ServoManager is initialized
  void begin(ServoManager* servo_mgr);

  // Start calibration (typically called from button callback)
  void startCalibration(bool only_connected = true);

  // Check if calibration is currently running
  bool isRunning() const;

  // Get the servo ID currently being calibrated (-1 if none)
  int currentServoId() const;

  // Check if calibration completed for a servo index
  bool isDone(size_t idx) const;

  // Get real-time position for a servo index (-1 if not available)
  int getRealtimePos(size_t idx) const;

  // Get min/max positions seen during calibration for a servo index
  int getMinSeen(size_t idx) const;
  int getMaxSeen(size_t idx) const;

  // Consume bar pending flags (returns true if flag was set, clears it)
  bool consumeBarPending(size_t idx);
  bool consumeBarInitPending(size_t idx);

  // Get current calibration phase name (thread-safe)
  const char* getCurrentPhaseName() const;

private:
  void onCalibProgress(int id, ServoManager::calib_stage_t stage);
  void onCalibPosition(int id, int position);
  void onCalibPhase(const char* phase_name);

  static void calibProgressThunk(int id, ServoManager::calib_stage_t stage, void* user);
  static void calibPositionThunk(int id, int position, void* user);
  static void calibPhaseThunk(const char* phase_name, void* user);

  ServoManager* _servo_mgr = nullptr;

  // Thread-safe state (accessed from calibration task and LVGL timer)
  mutable portMUX_TYPE _spin = portMUX_INITIALIZER_UNLOCKED;
  bool _run_active = false;
  int _current_id = -1;
  bool _done[kMaxServos] = {false};
  bool _bar_pending[kMaxServos] = {false};
  bool _bar_init_pending[kMaxServos] = {false};
  int _realtime_pos[kMaxServos];
  int _min_seen[kMaxServos];
  int _max_seen[kMaxServos];
  char _current_phase_name[64] = {0}; // Current calibration phase name
};

