#pragma once

#include "lvgl.h"
#include "ServoManager.h"
#include "CalibrationManager.h"
#include "BatteryManager.h"
#include "lcd_bsp.h"

// External references to globals
extern ServoManager g_servo_mgr;
extern CalibrationManager g_calib_mgr;
extern BatteryManager g_battery_mgr;

// UI initialization function (called from main.cpp setup)
void app_squareline_glue_init(void);
