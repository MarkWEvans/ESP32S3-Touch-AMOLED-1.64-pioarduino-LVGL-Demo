#include "ServoManager.h"

#include <math.h>
#include <string.h>

ServoManager::ServoManager() {
  // Default configs (your list)
  _configs = {{
      {2, "FR1", "Front Right Coxa (Hip)", false, 0, -25, 25, -1, -1},
      {3, "FR2", "Front Right Femur (Knee)", true, 48, -5, 110, -1, -1},
      {4, "FR3", "Front Right Tibia (Ankle)", true, 48, -8.6f, 150, -1, -1},
      {5, "BR1", "Back Right Coxa (Hip)", true, 0, -25, 25, -1, -1},
      {6, "BR2", "Back Right Femur (Knee)", true, -16.1f, -90, 100, -1, -1},
      {7, "BR3", "Back Right Tibia (Ankle)", true, 81, -8.6f, 150, -1, -1},
      {8, "BL1", "Back Left Coxa (Hip)", false, 0, -25, 25, -1, -1},
      {9, "BL2", "Back Left Femur (Knee)", false, -16.1f, -90, 100, -1, -1},
      {10, "BL3", "Back Left Tibia (Ankle)", false, 81, -8.6f, 150, -1, -1},
      {11, "FL1", "Front Left Coxa (Hip)", true, 0, -25, 25, -1, -1},
      {12, "FL2", "Front Left Femur (Knee)", false, 48, -5, 110, -1, -1},
      {13, "FL3", "Front Left Tibia (Ankle)", false, 48, -8.6f, 150, -1, -1},
  }};
  rebuildNameIndex();
}

void ServoManager::begin(HardwareSerial *serial, unsigned long sc_timeout, uint32_t update_rate_ms) {
  _serial = serial;
  _update_rate_ms = update_rate_ms;

  if (!_bus_mutex) {
    _bus_mutex = xSemaphoreCreateMutex();
  }

  if (_serial) {
    // Initialize the SCSCL driver (bind to UART + set IO timeout)
    _sc.begin(*_serial, sc_timeout);
  }

  // Attempt to load persisted configs; if missing, keep defaults.
  (void)loadServoConfigs();
  rebuildNameIndex();
}

bool ServoManager::loadServoConfigs() {
  _prefs.begin("servo_config", true);
  const size_t want = sizeof(ServoConfig) * _servoCount;
  size_t got = _prefs.getBytes("configs", _configs.data(), want);
  _prefs.end();
  if (got != want) {
    return false;
  }
  rebuildNameIndex();
  return true;
}

void ServoManager::saveServoConfigs() {
  _prefs.begin("servo_config", false);
  _prefs.putBytes("configs", _configs.data(), sizeof(ServoConfig) * _servoCount);
  _prefs.end();
  rebuildNameIndex();
}

void ServoManager::rebuildNameIndex() {
  _jointNameToIndex.clear();
  for (size_t i = 0; i < _servoCount; i++) {
    _jointNameToIndex[std::string(_configs[i].name)] = (int)i;
  }
}

int ServoManager::indexForId(int id) const {
  for (size_t i = 0; i < _servoCount; i++) {
    if (_configs[i].id == id) return (int)i;
  }
  return -1;
}

ServoConfig *ServoManager::findServoById(int id) {
  const int idx = indexForId(id);
  if (idx < 0) return nullptr;
  return &_configs[(size_t)idx];
}

float ServoManager::posToAngle(int pos, int minPos, int maxPos, float minAngle, float maxAngle, bool reverse) {
  if (maxPos == minPos) return minAngle;
  if (pos < minPos) pos = minPos;
  if (pos > maxPos) pos = maxPos;
  float ratio = float(pos - minPos) / float(maxPos - minPos);
  if (reverse) ratio = 1.0f - ratio;
  return minAngle + ratio * (maxAngle - minAngle);
}

int ServoManager::angleToPos(float angle, const ServoConfig &config) {
  if (config.maxAngle == config.minAngle) return config.minPos;
  if (angle < config.minAngle) angle = config.minAngle;
  if (angle > config.maxAngle) angle = config.maxAngle;
  float ratio = (angle - config.minAngle) / (config.maxAngle - config.minAngle);
  if (config.reverse) ratio = 1.0f - ratio;
  float rawPos = (float)config.minPos + ratio * (float)(config.maxPos - config.minPos);
  return (int)lroundf(rawPos);
}

bool ServoManager::setTargetAngle(int id, float angle) {
  int idx = indexForId(id);
  if (idx < 0) return false;
  _statuses[(size_t)idx].targetAngle = angle;
  _statuses[(size_t)idx].targetPosition = -1;
  _statuses[(size_t)idx].needToUpdate = true;
  return true;
}

bool ServoManager::setTargetPosition(int id, int pos) {
  int idx = indexForId(id);
  if (idx < 0) return false;
  _statuses[(size_t)idx].targetPosition = pos;
  _statuses[(size_t)idx].needToUpdate = true;
  return true;
}

bool ServoManager::requestTorqueEnable(int id, bool enable) {
  int idx = indexForId(id);
  if (idx < 0) return false;
  _statuses[(size_t)idx].needToEnable = enable;
  _statuses[(size_t)idx].needToDisable = !enable;
  return true;
}

void ServoManager::resetBus(uint8_t pin_tx, uint8_t pin_rx, uint32_t baud, uint32_t idle_delay_ms) {
  if (!_serial) return;
  _serial->flush();
  while (_serial->available()) (void)_serial->read();
  pinMode(pin_tx, OUTPUT);
  digitalWrite(pin_tx, HIGH);
  vTaskDelay(pdMS_TO_TICKS(idle_delay_ms));
  _serial->begin(baud, SERIAL_8N1, pin_rx, pin_tx);
}

void ServoManager::startUpdateTask() {
  if (_update_task) return;
  xTaskCreate(update_task_thunk, "ServoUpdate", 6144, this, 1, &_update_task);
}

void ServoManager::stopUpdateTask() {
  if (_update_task) {
    vTaskDelete(_update_task);
    _update_task = nullptr;
  }
}

void ServoManager::update_task_thunk(void *arg) {
  ServoManager *self = static_cast<ServoManager *>(arg);
  for (;;) {
    if (self->_calibration_active) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    uint32_t start = millis();
    self->updateOnce();
    uint32_t elapsed = millis() - start;
    self->_last_update_ms = elapsed;
    if (elapsed < self->_update_rate_ms) {
      vTaskDelay(pdMS_TO_TICKS(self->_update_rate_ms - elapsed));
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

void ServoManager::updateOnce() {
  if (!_bus_mutex) return;

  if (xSemaphoreTake(_bus_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    return;
  }

  for (size_t i = 0; i < _servoCount; i++) {
    const int id = _configs[i].id;
    ServoStatus &st = _statuses[i];

    // Minimal feed-back cycle similar to your code:
    // FeedBack() fills internal buffer, then Read*(-1) uses that buffer.
    if (_sc.FeedBack(id) != -1) {
      st.position = _sc.ReadPos(-1);
      st.connected = (st.position >= 0);

      if (st.connected) {
        // Only compute angle if min/max positions are known
        if (_configs[i].minPos >= 0 && _configs[i].maxPos >= 0) {
          st.angle = posToAngle(st.position, _configs[i].minPos, _configs[i].maxPos,
                                _configs[i].minAngle, _configs[i].maxAngle, _configs[i].reverse);
        }

        st.voltage = _sc.ReadVoltage(-1) / 10.0f;  // 0.1V units
        st.temperature = (float)_sc.ReadTemper(-1);
        st.load = _sc.ReadLoad(-1) / 10.0f;        // 0.1% units
        st.speed = (float)_sc.ReadSpeed(-1);
        st.is_moving = (_sc.ReadMove(-1) != 0);

        // Error status byte (implementation-specific; keep as best-effort)
        uint8_t statusByte = (uint8_t)_sc.ReadStatus(-1);
        if (statusByte != 0xFF) {
          // These masks are project-specific; if you have them, adjust here.
          // For now, keep conservative defaults.
          st.temperature_error = false;
          st.overload_error = false;
        }

        uint8_t torqueEnable = _sc.ReadTorqueEnable(id);
        if (torqueEnable == 0) st.torque_enable = false;
        else if (torqueEnable == 1) st.torque_enable = true;

        if (st.needToUpdate) {
          int new_pos = st.targetPosition;
          if (new_pos < 0 && _configs[i].minPos >= 0 && _configs[i].maxPos >= 0) {
            new_pos = angleToPos(st.targetAngle, _configs[i]);
          }
          if (new_pos >= 0) {
            _sc.WritePos(id, new_pos, (int)_update_rate_ms, 0);
          }
          st.needToUpdate = false;
        }

        if (st.needToDisable) {
          _sc.EnableTorque(id, 0);
          st.needToDisable = false;
        }
        if (st.needToEnable) {
          _sc.EnableTorque(id, 1);
          st.needToEnable = false;
        }
      }
    } else {
      st.connected = false;
      st.angle = 0.0f;
      st.position = -1;
      st.load = 0.0f;
      st.temperature = 0.0f;
      st.voltage = 0.0f;
      st.speed = 0.0f;
      st.is_moving = false;
    }
  }

  xSemaphoreGive(_bus_mutex);
}

void ServoManager::startWiggleTask(int id, uint32_t duration_ms, int delta_pos) {
  if (!_bus_mutex) return;
  WiggleParams *p = (WiggleParams *)malloc(sizeof(WiggleParams));
  if (!p) return;
  p->self = this;
  p->id = id;
  p->duration_ms = duration_ms;
  p->delta_pos = delta_pos;
  xTaskCreate(wiggle_task_thunk, "WiggleServo", 4096, p, 1, NULL);
}

void ServoManager::wiggle_task_thunk(void *arg) {
  WiggleParams *p = static_cast<WiggleParams *>(arg);
  ServoManager *self = p->self;
  const int id = p->id;
  const uint32_t dur = p->duration_ms;
  const int delta = p->delta_pos;
  free(p);

  bool success = false;
  if (!self || !self->_bus_mutex) {
    vTaskDelete(NULL);
    return;
  }

  if (xSemaphoreTake(self->_bus_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    self->_calibration_active = true;
    int position = self->_sc.ReadPos(id);
    if (position >= 0) {
      uint32_t start = millis();
      bool toggle = false;
      while ((uint32_t)(millis() - start) < dur) {
        int target = position + (toggle ? delta : -delta);
        target = constrain(target, 63, 1000);
        self->_sc.WritePos(id, target, (int)self->_update_rate_ms, 0);
        toggle = !toggle;
        vTaskDelay(pdMS_TO_TICKS(300));
      }
      self->_sc.WritePos(id, position, (int)self->_update_rate_ms, 0);
      success = true;
    }
    xSemaphoreGive(self->_bus_mutex);
    self->_calibration_active = false;
  }

  if (self->_wiggle_cb) {
    self->_wiggle_cb(id, success, self->_wiggle_user);
  }
  vTaskDelete(NULL);
}

void ServoManager::startScanTask(uint8_t id_min, uint8_t id_max, bool skip_leg_ids) {
  if (!_bus_mutex) return;
  ScanParams *p = (ScanParams *)malloc(sizeof(ScanParams));
  if (!p) return;
  p->self = this;
  p->id_min = id_min;
  p->id_max = id_max;
  p->skip_leg_ids = skip_leg_ids;
  xTaskCreate(scan_task_thunk, "ScanServos", 4096, p, 1, NULL);
}

void ServoManager::startCalibrateTask(int id) {
  if (!_bus_mutex) return;
  if (_calib_task || _calibration_active) {
    Serial.println("Calibrate: already running, ignoring request");
    return;
  }
  CalibrateParams *p = (CalibrateParams *)malloc(sizeof(CalibrateParams));
  if (!p) return;
  p->self = this;
  p->id = id;
  p->all = false;
  p->only_connected = true;
  xTaskCreate(calibrate_task_thunk, "CalibServo", 6144, p, 1, &_calib_task);
}

void ServoManager::startCalibrateAllTask(bool only_connected) {
  if (!_bus_mutex) return;
  if (_calib_task || _calibration_active) {
    Serial.println("Calibrate ALL: already running, ignoring request");
    return;
  }
  CalibrateParams *p = (CalibrateParams *)malloc(sizeof(CalibrateParams));
  if (!p) return;
  p->self = this;
  p->id = -1;
  p->all = true;
  p->only_connected = only_connected;
  xTaskCreate(calibrate_task_thunk, "CalibAll", 7168, p, 1, &_calib_task);
}

void ServoManager::scan_task_thunk(void *arg) {
  ScanParams *p = static_cast<ScanParams *>(arg);
  ServoManager *self = p->self;
  const uint8_t id_min = p->id_min;
  const uint8_t id_max = p->id_max;
  const bool skip_leg_ids = p->skip_leg_ids;
  free(p);

  if (!self || !self->_bus_mutex) {
    vTaskDelete(NULL);
    return;
  }

  uint8_t found[32];
  size_t count = 0;

  if (xSemaphoreTake(self->_bus_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    for (uint8_t id = id_min; id <= id_max; id++) {
      if (skip_leg_ids && id >= 2 && id <= 13) continue;
      int pingResult = self->_sc.Ping((u8)id);
      if (pingResult != -1 && count < sizeof(found)) {
        found[count++] = id;
      }
    }
    xSemaphoreGive(self->_bus_mutex);
  }

  if (self->_scan_cb) {
    self->_scan_cb(found, count, self->_scan_user);
  }

  vTaskDelete(NULL);
}

void ServoManager::calibrate_task_thunk(void *arg) {
  CalibrateParams *p = static_cast<CalibrateParams *>(arg);
  ServoManager *self = p ? p->self : nullptr;
  const int id = p ? p->id : -1;
  const bool all = p ? p->all : false;
  const bool only_connected = p ? p->only_connected : true;
  free(p);

  if (!self || !self->_bus_mutex) {
    vTaskDelete(NULL);
    return;
  }

  bool ok = false;
  // Pause the normal update loop ASAP (before waiting on the mutex), so calibration
  // doesn't fight the periodic feedback task for bus time.
  self->_calibration_active = true;
  if (xSemaphoreTake(self->_bus_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
    if (all) {
      if (self->_calib_cb) self->_calib_cb(-1, ServoManager::CALIB_ALL_BEGIN, self->_calib_user);
      self->calibrateAll_locked(only_connected);
      if (self->_calib_cb) self->_calib_cb(-1, ServoManager::CALIB_ALL_DONE, self->_calib_user);
      ok = true; // "all" run completed (individual failures are reported via callback/serial)
    } else {
      if (self->_calib_cb) self->_calib_cb(id, ServoManager::CALIB_SERVO_BEGIN, self->_calib_user);
      ok = self->calibrateServo_locked(id);
      if (self->_calib_cb) {
        self->_calib_cb(id, ok ? ServoManager::CALIB_SERVO_OK : ServoManager::CALIB_SERVO_FAIL, self->_calib_user);
      }
    }
    xSemaphoreGive(self->_bus_mutex);
  } else {
    Serial.printf("Calibrate %s: failed to acquire bus mutex\n", all ? "ALL" : "servo");
  }
  // Cooldown before resuming the normal monitor/update loop.
  vTaskDelay(pdMS_TO_TICKS(1000));
  self->_calibration_active = false;

  if (!all) {
    Serial.printf("Calibration %s for servo %d\n", ok ? "OK" : "FAILED", id);
  } else {
    Serial.println("Calibration ALL: done");
  }

  // Mark task as no longer running
  self->_calib_task = nullptr;
  vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════════════════════════
// Table-driven calibration phases
// Servo index order: FR1,FR2,FR3, BR1,BR2,BR3, BL1,BL2,BL3, FL1,FL2,FL3
// ═══════════════════════════════════════════════════════════════════════════
static const CalibPhase kCalibPhases[] = {
  // Phase 0: Move FL3, FR3, BL3, BR3 to MAX simultaneously (initial setup)
  {"FL3, FR3, BL3, BR3 to max",
   //FR1  FR2   FR3   BR1  BR2  BR3   BL1  BL2  BL3   FL1  FL2   FL3
   { -1,  -1,  1000,  -1,  -1, 1000,  -1,  -1, 1000,  -1,  -1,  1000},
   false, false, 700, 4000},

  // Phase 1: Pre-move ??2/??3 servos to safe endpoints (clear space for ??1 calibration)
  // BL2(idx 7), BR2(idx 4) go to MIN; all other ??2/??3 go to MAX
  {"premove ??2/??3",
   //FR1  FR2   FR3   BR1  BR2  BR3   BL1  BL2  BL3   FL1  FL2   FL3
   { -1, 1000, 1000,  -1,  60, 1000,  -1,  60, 1000,  -1, 1000, 1000},
   false, false, 700, 4000},

  // Phase 1: Move all ??1 servos to MIN position
  {"??1 to min",
   //FR1 FR2  FR3  BR1 BR2  BR3  BL1 BL2  BL3  FL1 FL2  FL3
   { 60, -1,  -1,  60, -1,  -1,  60, -1,  -1,  60, -1,  -1},
   true, false, 700, 4000},

  // Phase 2: Move all ??1 servos to MAX position
  {"??1 to max",
   //FR1  FR2  FR3  BR1  BR2  BR3  BL1  BR2  BL3  FL1  FL2  FL3
   {1000, -1,  -1, 1000, -1,  -1, 1000, -1,  -1, 1000, -1,  -1},
   false, true, 700, 4000},

  // Phase 3: FR2 to MIN
  {"FR2 to min",
   { -1,  60,  -1,  -1, -1,  -1,  -1, -1,  -1,  -1, -1,  -1},
   true, false, 700, 4000},

  // Phase 4: FR2 to MAX
  {"FR2 to max",
   { -1, 1000, -1,  -1, -1,  -1,  -1, -1,  -1,  -1, -1,  -1},
   false, true, 700, 4000},

  // Phase 5: FR3 to MIN
  {"FR3 to min",
   { -1,  -1,  60,  -1, -1,  -1,  -1, -1,  -1,  -1, -1,  -1},
   true, false, 700, 4000},

  // Phase 6: FR3 to MAX
  {"FR3 to max",
   { -1,  -1, 1000, -1, -1,  -1,  -1, -1,  -1,  -1, -1,  -1},
   false, true, 700, 4000},

  // Phase 7: BR2 to MIN
  {"BR2 to min",
   { -1,  -1,  -1,  -1, 60,  -1,  -1, -1,  -1,  -1, -1,  -1},
   true, false, 700, 4000},

  // Phase 8: BR2 to MAX
  {"BR2 to max",
   { -1,  -1,  -1,  -1, 1000,-1,  -1, -1,  -1,  -1, -1,  -1},
   false, true, 700, 4000},

  // Phase 9: BR3 to MIN
  {"BR3 to min",
   { -1,  -1,  -1,  -1, -1,  60,  -1, -1,  -1,  -1, -1,  -1},
   true, false, 700, 4000},

  // Phase 10: BR3 to MAX
  {"BR3 to max",
   { -1,  -1,  -1,  -1, -1, 1000, -1, -1,  -1,  -1, -1,  -1},
   false, true, 700, 4000},

  // Phase 11: BL2 to MIN
  {"BL2 to min",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, 60,  -1,  -1, -1,  -1},
   true, false, 700, 4000},

  // Phase 12: BL2 to MAX
  {"BL2 to max",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, 1000,-1,  -1, -1,  -1},
   false, true, 700, 4000},

  // Phase 13: BL3 to MIN
  {"BL3 to min",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, -1,  60,  -1, -1,  -1},
   true, false, 700, 4000},

  // Phase 14: BL3 to MAX
  {"BL3 to max",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, -1, 1000, -1, -1,  -1},
   false, true, 700, 4000},

  // Phase 15: FL2 to MIN
  {"FL2 to min",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, -1,  -1,  -1, 60,  -1},
   true, false, 700, 4000},

  // Phase 16: FL2 to MAX
  {"FL2 to max",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, -1,  -1,  -1, 1000,-1},
   false, true, 700, 4000},

  // Phase 17: FL3 to MIN
  {"FL3 to min",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, -1,  -1,  -1, -1,  60},
   true, false, 700, 4000},

  // Phase 18: FL3 to MAX
  {"FL3 to max",
   { -1,  -1,  -1,  -1, -1,  -1,  -1, -1,  -1,  -1, -1, 1000},
   false, true, 700, 4000},
};
static constexpr size_t kCalibPhaseCount = sizeof(kCalibPhases) / sizeof(kCalibPhases[0]);

bool ServoManager::executeCalibPhase(const CalibPhase& phase, bool only_connected) {
  Serial.printf("Calibrate phase: %s\n", phase.name);
  
  // Notify phase start
  if (_calib_phase_cb) _calib_phase_cb(phase.name, _calib_phase_user);

  // Track which servos are active in this phase
  bool active[kServoCount] = {false};
  int lastPos[kServoCount] = {-1};
  int stableCount[kServoCount] = {0};
  int settledPos[kServoCount] = {-1};  // Position when settled (for sampling)

  // Start all movements for servos with targets
  for (size_t i = 0; i < kServoCount; i++) {
    if (phase.targets[i] == CalibPhase::SKIP) continue;

    const int id = _configs[i].id;
    if (only_connected && _sc.Ping((u8)id) == -1) {
      Serial.printf("  skip %s (id=%d) - no response\n", _configs[i].name, id);
      continue;
    }

    _sc.EnableTorque((u8)id, 1);
    _sc.WritePos((u8)id, (u16)phase.targets[i], 0, phase.speed);
    active[i] = true;
    Serial.printf("  %s (id=%d) -> %d\n", _configs[i].name, id, phase.targets[i]);
  }

  // Wait for all active servos to settle
  uint32_t t0 = millis();
  while (millis() - t0 < phase.timeout_ms) {
    bool anyActive = false;

    for (size_t i = 0; i < kServoCount; i++) {
      if (!active[i]) continue;

      const int id = _configs[i].id;
      int pos = _sc.ReadPos(id);
      if (pos < 0) {
        active[i] = false;  // Lost connection
        continue;
      }

      // Report position for real-time UI
      if (_calib_pos_cb) _calib_pos_cb(id, pos, _calib_pos_user);

      // Check if settled (position stable for consecutive readings)
      if (lastPos[i] >= 0 && abs(pos - lastPos[i]) <= 2) {
        stableCount[i]++;
        if (stableCount[i] >= 4) {
          // Servo has settled
          settledPos[i] = pos;
          active[i] = false;
          Serial.printf("  %s settled at %d\n", _configs[i].name, pos);
        }
      } else {
        stableCount[i] = 0;
      }
      lastPos[i] = pos;

      if (active[i]) anyActive = true;
    }

    if (!anyActive) break;  // All servos settled
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Sample positions if requested (torque off, read, torque on)
  if (phase.sample_min || phase.sample_max) {
    for (size_t i = 0; i < kServoCount; i++) {
      if (phase.targets[i] == CalibPhase::SKIP) continue;
      if (settledPos[i] < 0) continue;  // Didn't settle properly

      const int id = _configs[i].id;

      // Disable torque briefly to get true resting position
      _sc.EnableTorque((u8)id, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      int sampledPos = _sc.ReadPos(id);
      _sc.EnableTorque((u8)id, 1);

      if (sampledPos < 0) continue;

      if (phase.sample_min) {
        _configs[i].minPos = sampledPos;
        Serial.printf("  %s sampled minPos=%d\n", _configs[i].name, sampledPos);
      }
      if (phase.sample_max) {
        _configs[i].maxPos = sampledPos;
        Serial.printf("  %s sampled maxPos=%d\n", _configs[i].name, sampledPos);
      }
    }
  }

  return true;
}

void ServoManager::calibrateAll_locked(bool only_connected) {
  Serial.printf("Calibrate ALL: begin (only_connected=%d, %d phases)\n",
                only_connected ? 1 : 0, (int)kCalibPhaseCount);

  // Track which servos have been calibrated (both min and max sampled)
  bool hasMin[kServoCount] = {false};
  bool hasMax[kServoCount] = {false};

  // Execute each phase
  for (size_t p = 0; p < kCalibPhaseCount; p++) {
    const CalibPhase& phase = kCalibPhases[p];

    // For phases that move servos, notify CALIB_SERVO_BEGIN for servos starting calibration
    if (p > 1) {  // Skip initial setup phases (phase 0: FL3/FR3/BL3/BR3 to max, phase 1: premove)
      for (size_t i = 0; i < kServoCount; i++) {
        if (phase.targets[i] == CalibPhase::SKIP) continue;
        if (phase.sample_min && !hasMin[i]) {
          // This servo is starting calibration
          if (_calib_cb) _calib_cb(_configs[i].id, CALIB_SERVO_BEGIN, _calib_user);
        }
      }
    }

    // Execute the phase
    executeCalibPhase(phase, only_connected);

    // Track what was sampled
    for (size_t i = 0; i < kServoCount; i++) {
      if (phase.targets[i] == CalibPhase::SKIP) continue;
      if (phase.sample_min) hasMin[i] = (_configs[i].minPos >= 0);
      if (phase.sample_max) hasMax[i] = (_configs[i].maxPos >= 0);

      // If both min and max are now sampled, this servo calibration is complete
      if (hasMin[i] && hasMax[i] && phase.sample_max) {
        // Validate range
        int lo = _configs[i].minPos;
        int hi = _configs[i].maxPos;
        if (lo > hi) { int tmp = lo; lo = hi; hi = tmp; _configs[i].minPos = lo; _configs[i].maxPos = hi; }

        bool ok = (hi - lo >= 50);
        Serial.printf("  %s calibrated: min=%d max=%d %s\n",
                      _configs[i].name, lo, hi, ok ? "OK" : "FAIL (range too small)");
        if (_calib_cb) _calib_cb(_configs[i].id, ok ? CALIB_SERVO_OK : CALIB_SERVO_FAIL, _calib_user);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200));  // Brief pause between phases
  }

  // Persist all configs at once
  saveServoConfigs();
  Serial.println("Calibrate ALL: finished");
}

bool ServoManager::calibrateServo_locked(int id) {
  ServoConfig *cfg = findServoById(id);
  if (!cfg) return false;

  int idx = indexForId(id);
  if (idx < 0 || idx >= (int)kServoCount) return false;

  // Check connected
  int cur = _sc.ReadPos(id);
  if (cur < 0) return false;

  Serial.printf("Calibrate servo %d (%s): start (cur=%d)\n", id, cfg->name, cur);

  // Build single-servo phases dynamically
  CalibPhase phaseMin = {"single servo to min", {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}, true, false, 700, 4000};
  CalibPhase phaseMax = {"single servo to max", {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}, false, true, 700, 4000};
  phaseMin.targets[idx] = CalibPhase::POS_MIN;
  phaseMax.targets[idx] = CalibPhase::POS_MAX;

  // Execute min phase
  executeCalibPhase(phaseMin, false);

  // Execute max phase
  executeCalibPhase(phaseMax, false);

  // Validate
  int lo = cfg->minPos;
  int hi = cfg->maxPos;
  if (lo > hi) { int tmp = lo; lo = hi; hi = tmp; cfg->minPos = lo; cfg->maxPos = hi; }

  if (hi - lo < 50) {
    Serial.printf("Calibrate servo %d: range too small (%d..%d)\n", id, lo, hi);
    return false;
  }

  Serial.printf("Calibrate servo %d: calibrated range minPos=%d maxPos=%d\n", id, cfg->minPos, cfg->maxPos);

  // Return to zero angle
  int zeroPos = angleToPos(cfg->zeroAngle, *cfg);
  if (zeroPos >= 0) {
    Serial.printf("Calibrate servo %d: return to zeroPos=%d\n", id, zeroPos);
    _sc.EnableTorque((u8)id, 1);
    _sc.WritePos((u8)id, (u16)zeroPos, 0, 700);
  }

  // Persist
  saveServoConfigs();
  Serial.printf("Calibrate servo %d: done\n", id);
  return true;
}


