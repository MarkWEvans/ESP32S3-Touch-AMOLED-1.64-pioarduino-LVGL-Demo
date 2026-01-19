#include "gui_updates.h"
#include "ui.h"
#include "WifiManager.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

// Constants
static const uint32_t WATCHDOG_TIMEOUT_MS = 5000; // 5 seconds without update = freeze
static const uint32_t GESTURE_DEBOUNCE_MS = 500; // Ignore button clicks for 500ms after a gesture

// Global state variables (used only in this file)
int g_selected_servo_idx = -1;
int g_wiggling_servo_idx = -1; // Track which servo is currently being wiggled (-1 if none)
uint32_t g_last_ui_update_ms = 0; // Watchdog for screen freeze detection
uint32_t g_last_gesture_ms = 0; // Gesture detection to prevent accidental button presses during swipes
uint32_t g_last_screen_change_ms = 0; // Track screen changes to skip heavy updates temporarily

// Helper: get bar widget for servo index (0-11: FL1,FL2,FL3, FR1..., BL1..., BR1...)
static lv_obj_t *get_servo_bar(int idx) {
  switch (idx) {
    case 0: return ui_BarFL1;
    case 1: return ui_BarFL2;
    case 2: return ui_BarFL3;
    case 3: return ui_BarFR1;
    case 4: return ui_BarFR2;
    case 5: return ui_BarFR3;
    case 6: return ui_BarBL1;
    case 7: return ui_BarBL2;
    case 8: return ui_BarBL3;
    case 9: return ui_BarBR1;
    case 10: return ui_BarBR2;
    case 11: return ui_BarBR3;
    default: return nullptr;
  }
}

// Helper: get temperature label widget for servo name
static lv_obj_t *get_servo_temp_label(const char *name) {
  if (!name) return nullptr;
  if (strcmp(name, "FL1") == 0) return ui_TempFL1;
  if (strcmp(name, "FL2") == 0) return ui_TempFL2;
  if (strcmp(name, "FL3") == 0) return ui_TempFL3;
  if (strcmp(name, "FR1") == 0) return ui_TempFR1;
  if (strcmp(name, "FR2") == 0) return ui_TempFR2;
  if (strcmp(name, "FR3") == 0) return ui_TempFR3;
  if (strcmp(name, "BL1") == 0) return ui_TempBL1;
  if (strcmp(name, "BL2") == 0) return ui_TempBL2;
  if (strcmp(name, "BL3") == 0) return ui_TempBL3;
  if (strcmp(name, "BR1") == 0) return ui_TempBR1;
  if (strcmp(name, "BR2") == 0) return ui_TempBR2;
  if (strcmp(name, "BR3") == 0) return ui_TempBR3;
  return nullptr;
}

// Helper: set bar min, max, and color
// If min > max, sets both start_value and value to 0
// Ensures max - min >= 3
static void set_bar_range_color(lv_obj_t *bar, int min_val, int max_val, lv_color_t color) {
  if (!bar) return;
  
  // Set color
  lv_obj_set_style_bg_color(bar, color, (lv_style_selector_t)((lv_style_selector_t)LV_PART_INDICATOR | (lv_style_selector_t)LV_STATE_DEFAULT));
  
  // If min > max, set both to 0
  if (min_val > max_val) {
    lv_bar_set_range(bar, 0, 1024);
    lv_bar_set_mode(bar, LV_BAR_MODE_RANGE);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    lv_bar_set_start_value(bar, 0, LV_ANIM_OFF);  // start value must be set after value
    return;
  }
  
  // Ensure max - min >= 3
  if (max_val - min_val < 3) {
    // Expand range to meet minimum requirement
    int center = (min_val + max_val) / 2;
    min_val = center - 2;
    max_val = center + 2;
  }
    // Clamp to valid range
  if (min_val < 0) min_val = 0;
  if (max_val > 1024) max_val = 1024;
  // Re-check if still too small (edge case)
  if (max_val - min_val < 3) {
    if (min_val == 0) {
      max_val = 3;
    } else {
      min_val = 1021;
      max_val = 1024;
    }
  }
  
  // Set bar range, mode, and values
  lv_bar_set_range(bar, 0, 1024);
  lv_bar_set_mode(bar, LV_BAR_MODE_RANGE);
  lv_bar_set_value(bar, max_val, LV_ANIM_OFF); 
  lv_bar_set_start_value(bar, min_val, LV_ANIM_OFF); // start value must be set after value
}

// Format uptime smartly: seconds, minutes, or hours+minutes
static void format_uptime(char *buf, size_t buf_size, uint32_t uptime_ms) {
  uint32_t uptime_sec = uptime_ms / 1000;
  
  if (uptime_sec < 60) {
    // Less than 1 minute: show seconds
    snprintf(buf, buf_size, "%lus", (unsigned long)uptime_sec);
  } else if (uptime_sec < 3600) {
    // Less than 1 hour: show minutes
    uint32_t minutes = uptime_sec / 60;
    snprintf(buf, buf_size, "%lum", (unsigned long)minutes);
  } else {
    // 1 hour or more: show hours and minutes
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    snprintf(buf, buf_size, "%luh %lum", (unsigned long)hours, (unsigned long)minutes);
  }
}

// Check if a gesture was recently detected (to prevent accidental button presses)
static bool was_recent_gesture() {
  if (g_last_gesture_ms == 0) return false;
  uint32_t now = millis();
  // Handle millis() overflow (wraps every ~49 days)
  uint32_t elapsed;
  if (now >= g_last_gesture_ms) {
    elapsed = now - g_last_gesture_ms;
  } else {
    // Overflow occurred - elapsed is very large, so definitely not recent
    elapsed = 0xFFFFFFFF;
  }
  return elapsed < GESTURE_DEBOUNCE_MS;
}

static void status_timer_cb(lv_timer_t *t) {
  (void)t;

  // Update watchdog timestamp to indicate UI is alive
  g_last_ui_update_ms = millis();
  
  // Skip ALL updates for 500ms after screen change to let SPI queue clear
  // This prevents SPI queue overflow when switching screens
  const uint32_t now = millis();
  const bool skip_updates = (g_last_screen_change_ms > 0) && 
                            ((now - g_last_screen_change_ms) < 500);
  
  if (skip_updates) {
    return; // Skip all updates to reduce SPI queue pressure
  }

  // Update uptime label
  if (ui_Uptime) {
    char uptime_buf[32];
    char time_buf[16];
    format_uptime(time_buf, sizeof(time_buf), millis());
    snprintf(uptime_buf, sizeof(uptime_buf), "Uptime: %s", time_buf);
    lv_label_set_text(ui_Uptime, uptime_buf);
  }

  // Update servo temperature labels on Screen3 (DISABLED)
  // Temperature updates are disabled to reduce CPU load
  /*
  if (!skip_heavy_updates) {
    const ServoConfig *cfg = g_servo_mgr.configs();
    const ServoStatus *st = g_servo_mgr.statuses();
    const size_t n = g_servo_mgr.servoCount();
    
    for (size_t i = 0; i < n && i < 12; i++) {
      lv_obj_t *temp_label = get_servo_temp_label(cfg[i].name);
      if (!temp_label) continue;
      
      // Format temperature: show as integer degrees Celsius
      char temp_buf[16];
      lv_color_t bg_color;
      
      if (st[i].connected && st[i].temperature > 0) {
        float temp = st[i].temperature;
        snprintf(temp_buf, sizeof(temp_buf), "%.0f°C", temp);
        
        // Set background color based on temperature (darker shades)
        if (temp < 40.0f) {
          bg_color = lv_color_hex(0x006600); // Dark green
        } else if (temp < 50.0f) {
          bg_color = lv_color_hex(0xCC5500); // Dark orange
        } else {
          bg_color = lv_color_hex(0xCC0000); // Dark red
        }
      } else {
        // Show "-" if servo is not connected or temperature is invalid
        snprintf(temp_buf, sizeof(temp_buf), "-");
        bg_color = lv_color_hex(0x808080); // Grey
      }
      
      lv_label_set_text(temp_label, temp_buf);
      // Set background color and make it visible
      lv_obj_set_style_bg_color(temp_label, bg_color, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
      lv_obj_set_style_bg_opa(temp_label, LV_OPA_COVER, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
    }
  }
  */

  // Update WiFi widgets (exists in current SquareLine export)
  if (ui_WifiAPLabel || ui_WifiStatusLabel || ui_WifiIcon || ui_ConnectedIcon) {
    wifi_mgr_snapshot_t w{};
    if (wifi_manager_snapshot_get(&w)) {
      const bool connected = (w.state == WIFI_MGR_STATE_CONNECTED) && (w.has_ip != 0);
      static bool s_prev_connected = false;

      if (ui_WifiAPLabel) {
        if (connected && w.ssid[0]) {
          // Router name + RSSI on the same line
          if (w.rssi_dbm != 0) {
            lv_label_set_text_fmt(ui_WifiAPLabel, "%s (%ld dBm)", w.ssid, (long)w.rssi_dbm);
          } else {
            lv_label_set_text_fmt(ui_WifiAPLabel, "%s (RSSI ?)", w.ssid);
          }
        } else if (w.state == WIFI_MGR_STATE_CONNECTING) {
          lv_label_set_text(ui_WifiAPLabel, "Connecting...");
        } else {
          lv_label_set_text(ui_WifiAPLabel, "WiFi: off");
        }
      }

      if (ui_WifiStatusLabel) {
        if (connected) {
          lv_label_set_text_fmt(ui_WifiStatusLabel,
                                "IP: %d.%d.%d.%d",
                                (int)w.ip[0], (int)w.ip[1], (int)w.ip[2], (int)w.ip[3]);
        } else if (w.state == WIFI_MGR_STATE_CONNECTING) {
          // Only show "Connecting..." once (in ui_WifiAPLabel). Keep status minimal here.
          lv_label_set_text(ui_WifiStatusLabel, "IP: -");
        } else {
          lv_label_set_text(ui_WifiStatusLabel, "IP: -\nDisconnected");
        }
      }

      if (ui_ConnectedIcon) {
        if (connected != s_prev_connected) {
          if (connected) {
            lv_obj_clear_flag(ui_ConnectedIcon, LV_OBJ_FLAG_HIDDEN);
          } else {
            lv_obj_add_flag(ui_ConnectedIcon, LV_OBJ_FLAG_HIDDEN);
          }
          s_prev_connected = connected;
        }
      }

      // WiFi icon is a 160x32 sprite sheet (5 frames of 32px each). The container is 32x32 and clips it.
      if (ui_WifiIcon) {
        uint8_t frame = 0; // 0 = no signal / disconnected

        if (connected) {
          const int32_t r = w.rssi_dbm;
          if (r >= -55) frame = 4;
          else if (r >= -67) frame = 3;
          else if (r >= -75) frame = 2;
          else if (r >= -85) frame = 1;
          else frame = 0;
        } else if (w.state == WIFI_MGR_STATE_CONNECTING) {
          // Simple blink while connecting
          static uint8_t blink = 0;
          blink ^= 1;
          frame = blink ? 1 : 0;
        }

        lv_obj_set_pos(ui_WifiIcon, -(int16_t)(frame * 32), 0);
      }
    }
  }

  // Update voltage/current label (exists in current SquareLine export)
  if (ui_VoltageCurrentLabel) {
    int32_t bus_mv = 0;
    int32_t cur_ma = 0;
    bool has_readings = false;
    
    // Get voltage and current from BatteryManager if available
    has_readings = g_battery_mgr.getReadings(&bus_mv, &cur_ma);
    
    if (has_readings) {
      // Show battery/voltage widgets
      if (ui_BatteryStatusLabel) lv_obj_clear_flag(ui_BatteryStatusLabel, LV_OBJ_FLAG_HIDDEN);
      if (ui_BatteryIcon) lv_obj_clear_flag(ui_BatteryIcon, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_VoltageCurrentLabel, LV_OBJ_FLAG_HIDDEN);
      // Format voltage and current to buffer using standard printf formatting
      char label_buf[32];
      float voltage_v = (float)bus_mv / 1000.0f;
      snprintf(label_buf, sizeof(label_buf), "%.2fV, %ldmA", voltage_v, (long)cur_ma);
      lv_label_set_text(ui_VoltageCurrentLabel, label_buf);
      
      // Calculate battery percentage using BatteryManager (1S cell)
      float battery_percent = g_battery_mgr.getBatteryPercentage(voltage_v);
      
      // Update battery status label
      if (ui_BatteryStatusLabel) {
        char bat_buf[8];
        snprintf(bat_buf, sizeof(bat_buf), "%.0f%%", battery_percent);
        lv_label_set_text(ui_BatteryStatusLabel, bat_buf);
      }
      
      // Update battery icon sprite position based on percentage
      // Battery sprite sheet has 5 frames: 10%, 25%, 50%, 75%, 100% (42px wide each)
      if (ui_BatteryIcon) {
        uint8_t frame = 0;
        if (battery_percent >= 100) frame = 4;
        else if (battery_percent >= 75) frame = 3;
        else if (battery_percent >= 50) frame = 2;
        else if (battery_percent >= 25) frame = 1;
        else frame = 0; // 0-24%
        
        // Set sprite position (each frame is 42px wide)
        lv_obj_set_pos(ui_BatteryIcon, -(int16_t)(frame * 42), 0);
      }
    } else {
      // No voltage or current available: hide these widgets
      if (ui_BatteryStatusLabel) lv_obj_add_flag(ui_BatteryStatusLabel, LV_OBJ_FLAG_HIDDEN);
      if (ui_BatteryIcon) lv_obj_add_flag(ui_BatteryIcon, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_VoltageCurrentLabel, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // Update per-servo status label background colors (grey = disconnected, green = connected)
  // Labels are created by SquareLine as ui_ServoStatusXX.
  // Skip color updates during calibration - only update bar values
  // Also skip if recent screen change to prevent watchdog timeout
  if (!g_calib_mgr.isRunning()) {
    auto label_for = [](const char *name) -> lv_obj_t * {
      if (!name) return nullptr;
      if (strcmp(name, "FL1") == 0) return ui_ServoStatusFL1;
      if (strcmp(name, "FL2") == 0) return ui_ServoStatusFL2;
      if (strcmp(name, "FL3") == 0) return ui_ServoStatusFL3;
      if (strcmp(name, "FR1") == 0) return ui_ServoStatusFR1;
      if (strcmp(name, "FR2") == 0) return ui_ServoStatusFR2;
      if (strcmp(name, "FR3") == 0) return ui_ServoStatusFR3;
      if (strcmp(name, "BL1") == 0) return ui_ServoStatusBL1;
      if (strcmp(name, "BL2") == 0) return ui_ServoStatusBL2;
      if (strcmp(name, "BL3") == 0) return ui_ServoStatusBL3;
      if (strcmp(name, "BR1") == 0) return ui_ServoStatusBR1;
      if (strcmp(name, "BR2") == 0) return ui_ServoStatusBR2;
      if (strcmp(name, "BR3") == 0) return ui_ServoStatusBR3;
      return nullptr;
    };

    const ServoConfig *cfg = g_servo_mgr.configs();
    const ServoStatus *st = g_servo_mgr.statuses();
    const size_t n = g_servo_mgr.servoCount();
    const lv_color_t c_grey = lv_color_hex(0x606060);
    const lv_color_t c_green = lv_color_hex(0x08EF03);

    for (size_t i = 0; i < n && i < 12; i++) {
      lv_obj_t *lbl = label_for(cfg[i].name);
      if (!lbl) continue;

      // Normal visualization: green if connected, grey if disconnected.
      lv_color_t bg = st[i].connected ? c_green : c_grey;

      lv_obj_set_style_bg_color(lbl, bg, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
      lv_obj_set_style_bg_opa(lbl, LV_OPA_COVER, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
    }
  }

  // Update bar ranges when calibration completes for a servo
  {
    for (int i = 0; i < 12; i++) {
      if (!g_calib_mgr.consumeBarPending(i)) continue;
      lv_obj_t *bar = get_servo_bar(i);
      if (!bar) continue;
      const ServoConfig *cfgs = g_servo_mgr.configs();
      if (cfgs[i].minPos >= 0 && cfgs[i].maxPos >= 0) {
        int minV = cfgs[i].minPos;
        int maxV = cfgs[i].maxPos;
        if (minV > maxV) { int tmp = minV; minV = maxV; maxV = tmp; }
        //lv_bar_set_range(bar, minV, maxV);
        Serial.printf("Bar %s range set: %d..%d\n", cfgs[i].name, minV, maxV);
      }
    }
  }

  // Real-time bar updates during calibration (updates every 0.2 seconds)
  // Cache previous values to only update when changed (reduces CPU load)
  // Update only 4 servos per timer tick to prevent watchdog timeout
  {
    static int prev_range_min[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    static int prev_range_max[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    static int prev_value[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    static int update_offset = 0; // Rotate which servos to update
    
    if (g_calib_mgr.isRunning()) {
      // Always update the currently active servo, plus 3 others per tick
      const int servos_per_tick = 3;
      const int current_id = g_calib_mgr.currentServoId();
      int current_idx = -1;
      if (current_id >= 0) {
        const ServoConfig *cfgs = g_servo_mgr.configs();
        const size_t n = g_servo_mgr.servoCount();
        for (size_t i = 0; i < n && i < 12; i++) {
          if (cfgs[i].id == current_id) {
            current_idx = (int)i;
            break;
          }
        }
      }
      
      // Update current servo first, then rotate through others
      int start_idx = update_offset;
      int end_idx = (start_idx + servos_per_tick < 12) ? (start_idx + servos_per_tick) : 12;
      
      // Build list of indices to update (current servo + rotating batch)
      int update_list[4];
      int update_count = 0;
      
      // Always include current servo if it exists
      if (current_idx >= 0 && current_idx < 12) {
        update_list[update_count++] = current_idx;
      }
      
      // Add rotating batch (skip if it's the current servo)
      for (int i = start_idx; i < end_idx && update_count < 4; i++) {
        if (i != current_idx) {
          update_list[update_count++] = i;
        }
      }
      
      for (int u = 0; u < update_count; u++) {
        int i = update_list[u];
        lv_obj_t *bar = get_servo_bar(i);
        if (!bar) continue;

        // Skip all updates for wiggling servo (preserve wiggle start_value and value)
        if (g_wiggling_servo_idx == i) {
          // Don't update anything - start_value and value are set once at wiggle start
          continue;
        }

        const int realtime_pos = g_calib_mgr.getRealtimePos(i);
        const int min_seen = g_calib_mgr.getMinSeen(i);
        const int max_seen = g_calib_mgr.getMaxSeen(i);
        const bool bar_init_pending = g_calib_mgr.consumeBarInitPending(i);

        // Initialize bar range at start of calibration for this servo
        if (bar_init_pending && realtime_pos >= 0) {
          // Set initial range to current position ± 10 (small visible window)
          int init_min = realtime_pos - 10;
          int init_max = realtime_pos + 10;
          if (init_min < 0) init_min = 0;
          
          // Set bar range and default blue color using helper function
          const lv_color_t c_default = lv_color_hex(0x5E87EE); // Default blue
          set_bar_range_color(bar, init_min, init_max, c_default);
          
          prev_range_min[i] = init_min;
          prev_range_max[i] = init_max;
          prev_value[i] = realtime_pos;
          continue;
        }

        // Update bar range dynamically as servo moves (expand to track min/max seen)
        // Update periodically even if only realtime_pos is available
        if (realtime_pos >= 0) {
          int range_min, range_max;
          
          if (min_seen >= 0 && max_seen >= 0) {
            // Both min and max have been seen - expand to include current position
            range_min = (min_seen < realtime_pos) ? min_seen : (realtime_pos - 10);
            range_max = (max_seen > realtime_pos) ? max_seen : (realtime_pos + 10);
            if (range_min < 0) range_min = 0;
            // Ensure min < max
            if (range_max <= range_min) {
              range_min = realtime_pos - 10;
              range_max = realtime_pos + 10;
              if (range_min < 0) range_min = 0;
            }
          } else if (min_seen >= 0) {
            // Only min seen so far - expand range to include current position
            range_min = (min_seen < realtime_pos) ? min_seen : (realtime_pos - 10);
            range_max = realtime_pos + 10;
            if (range_min < 0) range_min = 0;
          } else if (max_seen >= 0) {
            // Only max seen so far - expand range to include current position
            range_min = (realtime_pos - 10 >= 0) ? (realtime_pos - 10) : 0;
            range_max = (max_seen > realtime_pos) ? max_seen : (realtime_pos + 10);
          } else {
            // Neither min nor max seen yet - use current position ± 10
            range_min = realtime_pos - 10;
            range_max = realtime_pos + 10;
            if (range_min < 0) range_min = 0;
          }
          
          // Update only when values changed significantly (reduces SPI queue pressure)
          bool range_changed = (prev_range_min[i] != range_min || prev_range_max[i] != range_max);
          bool value_changed = (prev_value[i] != realtime_pos);
          bool is_current = (i == current_idx);
          
          // For current servo, update if position changed by at least 2 units (reduces SPI spam)
          bool significant_change = is_current && (abs(realtime_pos - prev_value[i]) >= 2);
          
          if (range_changed || significant_change) {
            // Set bar range and default blue color using helper function
            const lv_color_t c_default = lv_color_hex(0x5E87EE); // Default blue
            set_bar_range_color(bar, range_min, range_max, c_default);
            prev_range_min[i] = range_min;
            prev_range_max[i] = range_max;
          }
          
          if (value_changed && (significant_change || !is_current)) {
            // Update value to show current position (within the range set above)
            lv_bar_set_value(bar, realtime_pos, LV_ANIM_OFF);
            prev_value[i] = realtime_pos;
          }
        }
      }
      
      // Rotate offset for next timer tick (skip current servo in rotation)
      if (current_idx >= 0) {
        // If current servo is in the range we just processed, advance past it
        if (current_idx >= start_idx && current_idx < end_idx) {
          update_offset = (end_idx) % 12;
        } else {
          update_offset = (update_offset + servos_per_tick) % 12;
        }
      } else {
        update_offset = (update_offset + servos_per_tick) % 12;
      }
    } else {
      // Reset cache when calibration stops
      update_offset = 0;
      for (int i = 0; i < 12; i++) {
        prev_range_min[i] = -1;
        prev_range_max[i] = -1;
        prev_value[i] = -1;
      }
    }
  }

  // Update bar colors during wiggle (don't change start_value or value)
  {
    if (g_wiggling_servo_idx >= 0 && g_wiggling_servo_idx < 12) {
      lv_obj_t *bar = get_servo_bar(g_wiggling_servo_idx);
      if (bar) {
        // Set wiggling servo bar to orange/yellow color
        const lv_color_t c_wiggle = lv_color_hex(0xFFA500); // Orange
        lv_obj_set_style_bg_color(bar, c_wiggle, (lv_style_selector_t)((lv_style_selector_t)LV_PART_INDICATOR | (lv_style_selector_t)LV_STATE_DEFAULT));
        // Don't update start_value or value - they are set once at wiggle start
      }
    } else {
      // Reset all bar colors to default (blue) when not wiggling
      static int prev_wiggling = -1;
      if (prev_wiggling != g_wiggling_servo_idx) {
        for (int i = 0; i < 12; i++) {
          lv_obj_t *bar = get_servo_bar(i);
          if (bar) {
            const lv_color_t c_default = lv_color_hex(0x5E87EE); // Default blue
            lv_obj_set_style_bg_color(bar, c_default, (lv_style_selector_t)((lv_style_selector_t)LV_PART_INDICATOR | (lv_style_selector_t)LV_STATE_DEFAULT));
          }
        }
        prev_wiggling = g_wiggling_servo_idx;
      }
    }
  }

  // Update CalibrationInfo label with current phase during calibration
  // Only update when phase changes to reduce CPU load
  if (ui_CalibrationInfo) {
    static char s_prev_phase_name[64] = {0};
    static bool s_prev_calib_running = false;
    
    const bool calib_running = g_calib_mgr.isRunning();
    
    if (calib_running) {
      const char* phase_name = g_calib_mgr.getCurrentPhaseName();
      // Always check if phase changed (compare strings) or calibration just started
      bool phase_changed = false;
      bool has_phase_name = (phase_name && phase_name[0] != '\0');
      
      if (!s_prev_calib_running) {
        // Calibration just started
        phase_changed = true;
      } else if (has_phase_name) {
        // Compare with previous phase name
        if (strcmp(phase_name, s_prev_phase_name) != 0) {
          phase_changed = true;
        }
      } else if (s_prev_phase_name[0] != '\0') {
        // Phase name was cleared
        phase_changed = true;
      }
      
      if (phase_changed) {
        if (has_phase_name) {
          lv_label_set_text(ui_CalibrationInfo, phase_name);
          strncpy(s_prev_phase_name, phase_name, sizeof(s_prev_phase_name) - 1);
          s_prev_phase_name[sizeof(s_prev_phase_name) - 1] = '\0';
        } else {
          lv_label_set_text(ui_CalibrationInfo, "Calibrating...");
          s_prev_phase_name[0] = '\0';
        }
      }
    } else if (s_prev_calib_running) {
      // Calibration just stopped - clear label
      lv_label_set_text(ui_CalibrationInfo, "");
      s_prev_phase_name[0] = '\0';
    }
    
    s_prev_calib_running = calib_running;
  }

  // Disable buttons while calibration or wiggle is running
  {
    const bool calib_running = g_calib_mgr.isRunning();
    const bool wiggle_running = (g_wiggling_servo_idx >= 0);
    const bool should_disable = calib_running || wiggle_running;
    
    // Update calibrate button
    if (ui_CalibrateButton) {
      static bool s_prev_calib_disabled = false;
      if (should_disable != s_prev_calib_disabled) {
        s_prev_calib_disabled = should_disable;
        if (should_disable) lv_obj_add_state(ui_CalibrateButton, LV_STATE_DISABLED);
        else lv_obj_clear_state(ui_CalibrateButton, LV_STATE_DISABLED);
      }
    }
    
    // Update wiggle button
    if (ui_WiggleButton) {
      static bool s_prev_wiggle_disabled = false;
      if (should_disable != s_prev_wiggle_disabled) {
        s_prev_wiggle_disabled = should_disable;
        if (should_disable) lv_obj_add_state(ui_WiggleButton, LV_STATE_DISABLED);
        else lv_obj_clear_state(ui_WiggleButton, LV_STATE_DISABLED);
      }
    }
  }
}

static void backlight_boot_timer_cb(lv_timer_t *tmr) {
  (void)tmr;
  // Panel brightness is held at 0 during init (see lcd_bsp.cpp) to avoid a white flash.
  // Turn it on shortly after LVGL/UI init, when the first black frame is ready.
  set_amoled_backlight(255);
  lv_timer_del(tmr);
}

static void watchdog_timer_cb(lv_timer_t *tmr) {
  (void)tmr;
  const uint32_t now = millis();
  
  // Check if UI has frozen (no updates for WATCHDOG_TIMEOUT_MS)
  if (g_last_ui_update_ms > 0 && (now - g_last_ui_update_ms) > WATCHDOG_TIMEOUT_MS) {
    // Screen has frozen - set to black
    if (ui_Screen1) {
      lv_obj_set_style_bg_color(ui_Screen1, lv_color_black(), (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
      lv_obj_set_style_bg_opa(ui_Screen1, LV_OPA_COVER, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
    }
    if (ui_Screen2) {
      lv_obj_set_style_bg_color(ui_Screen2, lv_color_black(), (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
      lv_obj_set_style_bg_opa(ui_Screen2, LV_OPA_COVER, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
    }
    // Force refresh
    lv_refr_now(NULL);
  }
}

static void servo_status_label_clicked_cb(lv_event_t *e) {
  const char *name = (const char *)lv_event_get_user_data(e);
  if (!name) return;
  
  // Ignore label click if a gesture was recently detected (prevents accidental selection during swipe)
  if (was_recent_gesture()) {
    return;
  }
  const auto &m = g_servo_mgr.jointNameToIndex();
  auto it = m.find(std::string(name));
  if (it == m.end()) return;
  g_selected_servo_idx = it->second;
  if (g_selected_servo_idx >= 0 && g_selected_servo_idx < (int)g_servo_mgr.servoCount()) {
    Serial.printf("Selected servo %s (id %d)\n", name, g_servo_mgr.configs()[g_selected_servo_idx].id);
  }
}

static void calibrate_button_cb(lv_event_t *e) {
  (void)e;
  
  // Ignore button press if a gesture was recently detected (prevents accidental presses during swipe)
  if (was_recent_gesture()) {
    return;
  }
  
  // Set bar mode to RANGE mode for all connected servos at start of calibration
  const ServoStatus *st = g_servo_mgr.statuses();
  const size_t n = g_servo_mgr.servoCount();
  
  for (size_t i = 0; i < n && i < 12; i++) {
    if (!st[i].connected || st[i].position < 0) continue;
    
    lv_obj_t *bar = get_servo_bar(i);
    if (!bar) continue;
    
    // Set bar mode to RANGE mode for calibration
    // The actual range and color will be set by set_bar_range_color in the status timer
    lv_bar_set_mode(bar, LV_BAR_MODE_RANGE);
  }
  
  g_calib_mgr.startCalibration(true /*only_connected*/);
}

// Sequential wiggle task: wiggles each servo one by one
static void sequential_wiggle_task(void *arg) {
  (void)arg;
  
  // Disable both buttons at the start of wiggle
  if (ui_WiggleButton) {
    lv_obj_add_state(ui_WiggleButton, LV_STATE_DISABLED);
  }
  if (ui_CalibrateButton) {
    lv_obj_add_state(ui_CalibrateButton, LV_STATE_DISABLED);
  }
  
  const ServoConfig *cfgs = g_servo_mgr.configs();
  const ServoStatus *st = g_servo_mgr.statuses();
  const size_t n = g_servo_mgr.servoCount();
  const int wiggle_offset = 30;  // Actual servo movement range
  const int wiggle_ui_offset = 30;  // UI bar display range (thin bar)
  
  for (size_t i = 0; i < n && i < 12; i++) {
    if (!st[i].connected || st[i].position < 0) continue;
    
    int id = cfgs[i].id;
    int initial_pos = st[i].position; // Capture initial position before any movement
    
    // Set this servo as the one being wiggled
    g_wiggling_servo_idx = (int)i;
    
    // Set bar start_value (min) and value (max) to show thin wiggle range
    lv_obj_t *bar = get_servo_bar(i);
    if (bar) {
      int wiggle_min = initial_pos - wiggle_ui_offset;
      int wiggle_max = initial_pos + wiggle_ui_offset;
      if (wiggle_min < 0) wiggle_min = 0;
      if (wiggle_max > 1024) wiggle_max = 1024;
      
      // Set bar range and orange color using helper function
      const lv_color_t c_wiggle = lv_color_hex(0xFFA500); // Orange
      set_bar_range_color(bar, wiggle_min, wiggle_max, c_wiggle);
      
      // Small delay to ensure values are set before movement starts
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Toggle twice between initialPos+30 and initialPos-30 (actual servo movement)
    int pos_high = initial_pos + wiggle_offset;
    int pos_low = initial_pos - wiggle_offset;
    
    // Constrain to valid servo range
    if (pos_high > 1024) pos_high = 1024;
    if (pos_low < 0) pos_low = 0;
    
    Serial.printf("Wiggling %s (id=%d) from initial pos %d (movement: %d-%d, UI bar: %d-%d)\n", cfgs[i].name, id, initial_pos, initial_pos - wiggle_offset, initial_pos + wiggle_offset, initial_pos - wiggle_ui_offset, initial_pos + wiggle_ui_offset);
    
    // First toggle: to high
    g_servo_mgr.setTargetPosition(id, pos_high);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Second toggle: to low
    g_servo_mgr.setTargetPosition(id, pos_low);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Third toggle: to high
    g_servo_mgr.setTargetPosition(id, pos_high);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Fourth toggle: to low
    g_servo_mgr.setTargetPosition(id, pos_low);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Return to original position
    g_servo_mgr.setTargetPosition(id, initial_pos);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Clear wiggling flag for this servo
    g_wiggling_servo_idx = -1;
  }
  
  g_wiggling_servo_idx = -1; // Ensure cleared
  
  // Re-enable both buttons when wiggle completes
  if (ui_WiggleButton) {
    lv_obj_clear_state(ui_WiggleButton, LV_STATE_DISABLED);
  }
  if (ui_CalibrateButton) {
    lv_obj_clear_state(ui_CalibrateButton, LV_STATE_DISABLED);
  }
  
  Serial.println("Sequential wiggle complete");
  vTaskDelete(NULL);
}

static void wiggle_button_cb(lv_event_t *e) {
  (void)e;
  
  // Ignore button press if a gesture was recently detected (prevents accidental presses during swipe)
  if (was_recent_gesture()) {
    return;
  }
  
  // Don't start if calibration is running
  if (g_calib_mgr.isRunning() || g_servo_mgr.isCalibrationRunning()) {
    Serial.println("Wiggle: calibration running, ignoring");
    return;
  }
  
  // Don't start if wiggle is already running
  if (g_wiggling_servo_idx >= 0) {
    Serial.println("Wiggle: already running, ignoring");
    return;
  }
  
  // Start sequential wiggle task
  xTaskCreate(sequential_wiggle_task, "WiggleSeq", 4096, NULL, 1, NULL);
  Serial.println("Sequential wiggle started");
}

// ── Swipe gesture navigation between Screen1, Screen2, and Screen3 ──
// Use LVGL timer to defer screen load to LVGL task context
static lv_obj_t* s_pending_screen = nullptr;

static void deferred_screen_load_timer_cb(lv_timer_t *tmr) {
  if (s_pending_screen) {
    lv_obj_t* target = s_pending_screen;
    s_pending_screen = nullptr;
    
    Serial.printf("  -> Loading screen in timer (deferred)...\n");
    
    // Record screen change time BEFORE loading to skip updates during render
    g_last_screen_change_ms = millis();
    
    // Use lv_disp_load_scr which is what ui_init() uses
    lv_disp_t* disp = lv_disp_get_default();
    if (disp) {
      lv_disp_load_scr(target);
      Serial.printf("  -> Screen load completed in timer\n");
    } else {
      // Fallback to lv_scr_load if disp not available
      lv_scr_load(target);
      Serial.printf("  -> Screen load completed (fallback)\n");
    }
    
    // Verify screen actually changed
    lv_obj_t* new_current = lv_scr_act();
    if (new_current == target) {
      Serial.printf("  -> Screen successfully switched\n");
    } else {
      Serial.printf("  -> WARNING: Screen switch failed! Expected %p, got %p\n", target, new_current);
    }
  }
  
  // Delete this one-shot timer
  lv_timer_del(tmr);
}

static void screen_gesture_cb(lv_event_t *e) {
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
  lv_obj_t *current = lv_scr_act();
  
  // Debug output to verify gesture is detected
  const char* dir_str = "UNKNOWN";
  if (dir == LV_DIR_LEFT) dir_str = "LEFT";
  else if (dir == LV_DIR_RIGHT) dir_str = "RIGHT";
  else if (dir == LV_DIR_TOP) dir_str = "TOP";
  else if (dir == LV_DIR_BOTTOM) dir_str = "BOTTOM";
  
  const char* screen_str = "UNKNOWN";
  if (current == ui_Screen1) screen_str = "Screen1";
  else if (current == ui_Screen2) screen_str = "Screen2";
  else if (current == ui_Screen3) screen_str = "Screen3";
  
  Serial.printf("Swipe detected: dir=%s, current=%s\n", dir_str, screen_str);
  
  // Record gesture timestamp to prevent accidental button presses
  g_last_gesture_ms = millis();

  // Determine target screen based on swipe direction
  lv_obj_t* target_screen = nullptr;
  if (dir == LV_DIR_LEFT && current == ui_Screen1) {
    target_screen = ui_Screen2;
    Serial.println("  -> Switching to Screen2");
  } else if (dir == LV_DIR_RIGHT && current == ui_Screen2) {
    target_screen = ui_Screen1;
    Serial.println("  -> Switching to Screen1");
  } else if (dir == LV_DIR_LEFT && current == ui_Screen2) {
    target_screen = ui_Screen3;
    Serial.println("  -> Switching to Screen3");
  } else if (dir == LV_DIR_RIGHT && current == ui_Screen3) {
    target_screen = ui_Screen2;
    Serial.println("  -> Switching to Screen2");
  }
  
  if (target_screen) {
    // Store target and create a one-shot timer to load it in LVGL task context
    // Use longer delay (200ms) to let SPI queue process pending transactions
    // This prevents the queue from overflowing when the new screen renders
    s_pending_screen = target_screen;
    lv_timer_create(deferred_screen_load_timer_cb, 200, NULL); // 200ms delay to clear SPI queue
    
    // Reset scroll position to prevent unwanted scrolling
    if (current) {
      lv_obj_scroll_to(current, 0, 0, LV_ANIM_OFF);
    }
  }
  
  (void)e;
}

void app_squareline_glue_init(void) {
  // Create a periodic timer to refresh dynamic labels
  // Increased interval to 500ms to reduce CPU load and prevent watchdog timeouts
  lv_timer_create(status_timer_cb, 500, NULL);
  
  // Create watchdog timer to detect screen freezes (check every 1 second)
  lv_timer_create(watchdog_timer_cb, 1000, NULL);
  
  // Initialize watchdog timestamp
  g_last_ui_update_ms = millis();

  // Force a true black background regardless of theme defaults.
  if (ui_Screen1) {
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_black(), (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
    lv_obj_set_style_bg_opa(ui_Screen1, LV_OPA_COVER, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
  }
  if (ui_Screen2) {
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_black(), (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
    lv_obj_set_style_bg_opa(ui_Screen2, LV_OPA_COVER, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
  }
  if (ui_Screen3) {
    lv_obj_set_style_bg_color(ui_Screen3, lv_color_black(), (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
    lv_obj_set_style_bg_opa(ui_Screen3, LV_OPA_COVER, (lv_style_selector_t)((lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT));
  }

  // One-shot: enable brightness after UI init to prevent a visible white flash at boot.
  lv_timer_create(backlight_boot_timer_cb, 200, NULL);

  // Servo calibration UI hooks (Screen1 and Screen2 buttons)
  if (ui_CalibrateButton) {
    lv_obj_add_event_cb(ui_CalibrateButton, calibrate_button_cb, LV_EVENT_CLICKED, NULL);
  }
  
  // Wiggle button hook
  if (ui_WiggleButton) {
    lv_obj_add_event_cb(ui_WiggleButton, wiggle_button_cb, LV_EVENT_CLICKED, NULL);
  } 

  // Tap a servo label to select which servo is calibrated.
  // Labels are not clickable by default, so mark them clickable.
  auto hook_servo_label = [](lv_obj_t *lbl, const char *name) {
    if (!lbl) return;
    lv_obj_add_flag(lbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl, servo_status_label_clicked_cb, LV_EVENT_CLICKED, (void *)name);
  };
  hook_servo_label(ui_ServoStatusFL1, "FL1");
  hook_servo_label(ui_ServoStatusFL2, "FL2");
  hook_servo_label(ui_ServoStatusFL3, "FL3");
  hook_servo_label(ui_ServoStatusFR1, "FR1");
  hook_servo_label(ui_ServoStatusFR2, "FR2");
  hook_servo_label(ui_ServoStatusFR3, "FR3");
  hook_servo_label(ui_ServoStatusBL1, "BL1");
  hook_servo_label(ui_ServoStatusBL2, "BL2");
  hook_servo_label(ui_ServoStatusBL3, "BL3");
  hook_servo_label(ui_ServoStatusBR1, "BR1");
  hook_servo_label(ui_ServoStatusBR2, "BR2");
  hook_servo_label(ui_ServoStatusBR3, "BR3");

  // Swipe left/right to navigate between screens
  // Note: In LVGL 8.3, gestures require objects to be scrollable
  // We enable scrollable for gestures but disable actual scrolling
  Serial.println("Registering gesture callbacks...");
  if (ui_Screen1) {
    lv_obj_add_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE); // Enable for gestures
    lv_obj_set_scroll_dir(ui_Screen1, LV_DIR_HOR); // Only allow horizontal scrolling
    lv_obj_set_scroll_snap_x(ui_Screen1, LV_SCROLL_SNAP_NONE); // Disable snap
    lv_obj_add_event_cb(ui_Screen1, screen_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_GESTURE_BUBBLE);
    Serial.println("  Screen1 gesture callback registered");
  }
  if (ui_Screen2) {
    lv_obj_add_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE); // Enable for gestures
    lv_obj_set_scroll_dir(ui_Screen2, LV_DIR_HOR); // Only allow horizontal scrolling
    lv_obj_set_scroll_snap_x(ui_Screen2, LV_SCROLL_SNAP_NONE); // Disable snap
    lv_obj_add_event_cb(ui_Screen2, screen_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_GESTURE_BUBBLE);
    Serial.println("  Screen2 gesture callback registered");
  }
  if (ui_Screen3) {
    lv_obj_add_flag(ui_Screen3, LV_OBJ_FLAG_SCROLLABLE); // Enable for gestures
    lv_obj_set_scroll_dir(ui_Screen3, LV_DIR_HOR); // Only allow horizontal scrolling
    lv_obj_set_scroll_snap_x(ui_Screen3, LV_SCROLL_SNAP_NONE); // Disable snap
    lv_obj_add_event_cb(ui_Screen3, screen_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(ui_Screen3, LV_OBJ_FLAG_GESTURE_BUBBLE);
    Serial.println("  Screen3 gesture callback registered");
  }
  Serial.println("Gesture callbacks registration complete");
}
