#include "CalibrationManager.h"
#include <string.h>

CalibrationManager::CalibrationManager() {
  for (size_t i = 0; i < kMaxServos; i++) {
    _realtime_pos[i] = -1;
    _min_seen[i] = -1;
    _max_seen[i] = -1;
  }
}

void CalibrationManager::begin(ServoManager* servo_mgr) {
  _servo_mgr = servo_mgr;
  if (!_servo_mgr) return;

  _servo_mgr->setCalibProgressCallback(calibProgressThunk, this);
  _servo_mgr->setCalibPositionCallback(calibPositionThunk, this);
  _servo_mgr->setCalibPhaseCallback(calibPhaseThunk, this);
}

void CalibrationManager::startCalibration(bool only_connected) {
  if (!_servo_mgr) return;
  if (_servo_mgr->isCalibrationRunning()) {
    Serial.println("Calibrate: already running");
    return;
  }
  Serial.println("Calibrate: starting ALL servos (sequential)");
  _servo_mgr->startCalibrateAllTask(only_connected);
}

bool CalibrationManager::isRunning() const {
  portENTER_CRITICAL(&_spin);
  bool active = _run_active;
  portEXIT_CRITICAL(&_spin);
  return active;
}

int CalibrationManager::currentServoId() const {
  portENTER_CRITICAL(&_spin);
  int id = _current_id;
  portEXIT_CRITICAL(&_spin);
  return id;
}

bool CalibrationManager::isDone(size_t idx) const {
  if (idx >= kMaxServos) return false;
  portENTER_CRITICAL(&_spin);
  bool done = _done[idx];
  portEXIT_CRITICAL(&_spin);
  return done;
}

int CalibrationManager::getRealtimePos(size_t idx) const {
  if (idx >= kMaxServos) return -1;
  portENTER_CRITICAL(&_spin);
  int pos = _realtime_pos[idx];
  portEXIT_CRITICAL(&_spin);
  return pos;
}

int CalibrationManager::getMinSeen(size_t idx) const {
  if (idx >= kMaxServos) return -1;
  portENTER_CRITICAL(&_spin);
  int val = _min_seen[idx];
  portEXIT_CRITICAL(&_spin);
  return val;
}

int CalibrationManager::getMaxSeen(size_t idx) const {
  if (idx >= kMaxServos) return -1;
  portENTER_CRITICAL(&_spin);
  int val = _max_seen[idx];
  portEXIT_CRITICAL(&_spin);
  return val;
}

bool CalibrationManager::consumeBarPending(size_t idx) {
  if (idx >= kMaxServos) return false;
  portENTER_CRITICAL(&_spin);
  bool pending = _bar_pending[idx];
  _bar_pending[idx] = false;
  portEXIT_CRITICAL(&_spin);
  return pending;
}

bool CalibrationManager::consumeBarInitPending(size_t idx) {
  if (idx >= kMaxServos) return false;
  portENTER_CRITICAL(&_spin);
  bool pending = _bar_init_pending[idx];
  _bar_init_pending[idx] = false;
  portEXIT_CRITICAL(&_spin);
  return pending;
}

void CalibrationManager::calibProgressThunk(int id, ServoManager::calib_stage_t stage, void* user) {
  if (user) static_cast<CalibrationManager*>(user)->onCalibProgress(id, stage);
}

void CalibrationManager::calibPositionThunk(int id, int position, void* user) {
  if (user) static_cast<CalibrationManager*>(user)->onCalibPosition(id, position);
}

void CalibrationManager::calibPhaseThunk(const char* phase_name, void* user) {
  if (user) static_cast<CalibrationManager*>(user)->onCalibPhase(phase_name);
}

void CalibrationManager::onCalibProgress(int id, ServoManager::calib_stage_t stage) {
  portENTER_CRITICAL(&_spin);

  if (stage == ServoManager::CALIB_ALL_BEGIN) {
    _run_active = true;
    _current_id = -1;
    _current_phase_name[0] = '\0'; // Clear phase name at start
    for (size_t i = 0; i < kMaxServos; i++) {
      _done[i] = false;
      _realtime_pos[i] = -1;
      _min_seen[i] = -1;
      _max_seen[i] = -1;
      _bar_init_pending[i] = false;
    }
  } else if (stage == ServoManager::CALIB_SERVO_BEGIN) {
    _current_id = id;
    int idx = _servo_mgr ? _servo_mgr->indexForId(id) : -1;
    if (idx >= 0 && idx < (int)kMaxServos) {
      _bar_init_pending[idx] = true;
      _min_seen[idx] = -1;
      _max_seen[idx] = -1;
    }
  } else if (stage == ServoManager::CALIB_SERVO_OK) {
    int idx = _servo_mgr ? _servo_mgr->indexForId(id) : -1;
    if (idx >= 0 && idx < (int)kMaxServos) {
      _done[idx] = true;
      _bar_pending[idx] = true;
    }
    _current_id = -1;
  } else if (stage == ServoManager::CALIB_SERVO_FAIL) {
    _current_id = -1;
  } else if (stage == ServoManager::CALIB_ALL_DONE) {
    _run_active = false;
    _current_id = -1;
    _current_phase_name[0] = '\0'; // Clear phase name when done
    for (size_t i = 0; i < kMaxServos; i++) {
      _done[i] = false;
    }
  }

  portEXIT_CRITICAL(&_spin);
}

void CalibrationManager::onCalibPosition(int id, int position) {
  int idx = _servo_mgr ? _servo_mgr->indexForId(id) : -1;
  if (idx < 0 || idx >= (int)kMaxServos) return;

  portENTER_CRITICAL(&_spin);
  _realtime_pos[idx] = position;

  if (_min_seen[idx] < 0 || position < _min_seen[idx]) {
    _min_seen[idx] = position;
  }
  if (_max_seen[idx] < 0 || position > _max_seen[idx]) {
    _max_seen[idx] = position;
  }
  portEXIT_CRITICAL(&_spin);
}

void CalibrationManager::onCalibPhase(const char* phase_name) {
  if (!phase_name) return;
  portENTER_CRITICAL(&_spin);
  strncpy(_current_phase_name, phase_name, sizeof(_current_phase_name) - 1);
  _current_phase_name[sizeof(_current_phase_name) - 1] = '\0';
  portEXIT_CRITICAL(&_spin);
}

const char* CalibrationManager::getCurrentPhaseName() const {
  // Return pointer to member variable (valid after critical section exits)
  // Caller should use immediately, as it may be updated by calibration task
  portENTER_CRITICAL(&_spin);
  const char* name = _current_phase_name;
  portEXIT_CRITICAL(&_spin);
  return name; // Pointer to member variable remains valid
}
