#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H

#define LCD_H_RES              280
#define LCD_V_RES              456

#define LCD_BIT_PER_PIXEL      16

#define PIN_NUM_LCD_CS         9
#define PIN_NUM_LCD_PCLK       10
#define PIN_NUM_LCD_DATA0      11
#define PIN_NUM_LCD_DATA1      12
#define PIN_NUM_LCD_DATA2      13
#define PIN_NUM_LCD_DATA3      14
#define PIN_NUM_LCD_RST        21
#define PIN_NUM_BK_LIGHT       (-1)

#if (LVGL_ROTATION_DEG == 0)
#define LVGL_BUF_HEIGHT        (LCD_V_RES / 4)
#else
// Rotation uses extra temporary buffers; keep the draw buffers smaller to reduce RAM pressure.
#define LVGL_BUF_HEIGHT        (LCD_V_RES / 8)
#endif
#define LVGL_TICK_PERIOD_MS    2                          // Timer time
#define LVGL_TASK_MAX_DELAY_MS 500                        // LVGL max time for a task to run
#define LVGL_TASK_MIN_DELAY_MS 1                          // LVGL min time to run a task
#define LVGL_TASK_STACK_SIZE   (4 * 1024)                 // LVGL task stack
#define LVGL_TASK_PRIORITY     2                          // LVGL task priority

// Display/UI rotation in degrees (0, 90, 180, 270).
// This uses LVGL software rotation so the panel driver doesn't need swap-XY support.
#define LVGL_ROTATION_DEG 90

#define I2C_ADDR_FT3168        0x38
#define PIN_NUM_TOUCH_SCL      48
#define PIN_NUM_TOUCH_SDA      47

#endif
