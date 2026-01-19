#ifndef BATTERY_UTILS_H
#define BATTERY_UTILS_H

#include <Arduino.h>

class BatteryUtils {
public:
  // cells must be 1, 2, 3 or 4
  explicit BatteryUtils(uint8_t cells)
    : cells_(cells),
      currentIndex_(0),
      totalVoltage_(0.0f),
      smoothedVoltage_(0.0f),
      validReadingsCount_(0),
      lastRawVoltage_(kMinValidPerCell * cells),
      cutoffPerCell_(kCutoffPerCellDefault) {

    if (cells_ < 1 || cells_ > 4) {
      // Enforce contract strictly; clamp + warn
      uint8_t clamped = cells_ < 1 ? 1 : 4;
      Serial.printf(
        "[BatteryUtils] Invalid cell count %u. Clamping to %u (allowed: 1,2,3,4)\n",
        cells_, clamped);
      cells_ = clamped;
    }

    minValidVoltagePack_ = kMinValidPerCell * cells_;
    resetThresholdPack_  = kResetThresholdPerCell * cells_;

    for (int i = 0; i < kNumReadings; ++i) {
      voltageReadings_[i] = 0.0f;
    }
  }

  // Add a new PACK voltage reading (volts), smoothed internally
  void addVoltageReading(float newVoltage) {
    // Flush if we drop below plausibility or come back from brown-out to healthy
    if (newVoltage < minValidVoltagePack_ ||
        (lastRawVoltage_ < minValidVoltagePack_ && newVoltage > resetThresholdPack_)) {
      flushBuffer_();
    }

    totalVoltage_ -= voltageReadings_[currentIndex_];
    voltageReadings_[currentIndex_] = newVoltage;
    totalVoltage_ += newVoltage;

    currentIndex_ = (currentIndex_ + 1) % kNumReadings;

    if (validReadingsCount_ < kNumReadings) {
      ++validReadingsCount_;
    }

    smoothedVoltage_ = totalVoltage_ / validReadingsCount_;
    lastRawVoltage_  = newVoltage;
  }

  // Return 0–100% based on PACK voltage, mapped via per-cell table
  float calculateBatteryPercentage(float rawPackVoltage) {
    addVoltageReading(rawPackVoltage);

    if (validReadingsCount_ == 0 || cells_ == 0) return 0.0f;

    const float perCell = smoothedVoltage_ / static_cast<float>(cells_);

    // Bounds check
    if (perCell >= kPerCellVoltageTable[0][0]) {
      return 100.0f; // above max voltage => 100%
    }
    if (perCell <= kPerCellVoltageTable[kTableSize - 1][0]) {
      return 0.0f; // below min of table => 0%
    }

    // Piecewise linear interpolation across table
    for (size_t i = 0; i < kTableSize - 1; ++i) {
      const float v_hi = kPerCellVoltageTable[i][0];
      const float p_hi = kPerCellVoltageTable[i][1];
      const float v_lo = kPerCellVoltageTable[i + 1][0];
      const float p_lo = kPerCellVoltageTable[i + 1][1];

      if (perCell <= v_hi && perCell > v_lo) {
        const float dv = v_hi - v_lo;
        const float dp = p_hi - p_lo;
        const float t  = (perCell - v_lo) / dv; // 0..1 between lo and hi
        return p_lo + t * dp;
      }
    }

    return 0.0f; // should not reach
  }

  float getMinCutoffVoltage() const {
    // Returns the PACK-level cutoff voltage (per-cell * cells)
    return cutoffPerCell_ * static_cast<float>(cells_);
  }

  // Helpers
  uint8_t cells() const { return cells_; }
  float smoothedPackVoltage() const { return smoothedVoltage_; }
  float smoothedPerCellVoltage() const {
    return (validReadingsCount_ == 0 || cells_ == 0) ? 0.0f : (smoothedVoltage_ / cells_);
  }

private:
  // ── Configuration constants ──────────────────────────────────────────────
  static constexpr int   kNumReadings            = 10;
  static constexpr float kMinValidPerCell        = 2.0f; // plausibility floor per cell
  static constexpr float kResetThresholdPerCell  = 3.5f; // buffer reset when recovering
  static constexpr float kCutoffPerCellDefault   = 3.3f; // V per cell (under load)

  // Per-cell voltage (%) table (derived from your 2S table / 2)
  // { per-cell volts, percent }
  static constexpr size_t kTableSize = 19;
  static const float kPerCellVoltageTable[kTableSize][2];

  // ── State ────────────────────────────────────────────────────────────────
  uint8_t cells_;
  float   minValidVoltagePack_;
  float   resetThresholdPack_;
  float cutoffPerCell_;

  float voltageReadings_[kNumReadings];
  int   currentIndex_;
  float totalVoltage_;
  float smoothedVoltage_;
  int   validReadingsCount_;
  float lastRawVoltage_;

  // Reset smoothing buffer
  void flushBuffer_() {
    for (int i = 0; i < kNumReadings; ++i) voltageReadings_[i] = 0.0f;
    totalVoltage_      = 0.0f;
    smoothedVoltage_   = 0.0f;
    currentIndex_      = 0;
    validReadingsCount_ = 0;
  }
};

// Inline definition so the header stays self-contained (C++17)
inline const float BatteryUtils::kPerCellVoltageTable[BatteryUtils::kTableSize][2] = {
  { 4.20f, 100 }, { 4.15f, 95 }, { 4.11f, 90 }, { 4.08f, 85 }, { 4.02f, 80 },
  { 3.98f, 75 },  { 3.95f, 70 }, { 3.91f, 65 }, { 3.87f, 60 }, { 3.85f, 55 },
  { 3.84f, 50 },  { 3.82f, 45 }, { 3.80f, 40 }, { 3.79f, 35 }, { 3.77f, 30 },
  { 3.75f, 25 },  { 3.73f, 20 }, { 3.71f, 15 }, { 3.69f, 10 }
};

#endif // BATTERY_UTILS_H