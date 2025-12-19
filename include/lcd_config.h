#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H

#define EXAMPLE_LCD_H_RES              280
#define EXAMPLE_LCD_V_RES              456

#define LCD_BIT_PER_PIXEL              16

#define EXAMPLE_PIN_NUM_LCD_CS            9
#define EXAMPLE_PIN_NUM_LCD_PCLK          10
#define EXAMPLE_PIN_NUM_LCD_DATA0         11
#define EXAMPLE_PIN_NUM_LCD_DATA1         12
#define EXAMPLE_PIN_NUM_LCD_DATA2         13
#define EXAMPLE_PIN_NUM_LCD_DATA3         14
#define EXAMPLE_PIN_NUM_LCD_RST           21
#define EXAMPLE_PIN_NUM_BK_LIGHT          (-1)

#if (EXAMPLE_LVGL_ROTATION_DEG == 0)
#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 4)
#else
// Rotation uses extra temporary buffers; keep the draw buffers smaller to reduce RAM pressure.
#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 8)
#endif
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2                          // Timer time
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500                        // LVGL max time for a task to run
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1                          // LVGL min time to run a task
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)                 // LVGL task stack
#define EXAMPLE_LVGL_TASK_PRIORITY     2                          // LVGL task priority

// Display/UI rotation in degrees (0, 90, 180, 270).
// This uses LVGL software rotation so the panel driver doesn't need swap-XY support.
#define EXAMPLE_LVGL_ROTATION_DEG 90

#define I2C_ADDR_FT3168 0x38
#define EXAMPLE_PIN_NUM_TOUCH_SCL 48
#define EXAMPLE_PIN_NUM_TOUCH_SDA 47

#endif


