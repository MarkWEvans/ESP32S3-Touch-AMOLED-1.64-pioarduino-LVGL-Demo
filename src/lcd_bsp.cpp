#include "lcd_bsp.h"

#include <assert.h>
#include <stdlib.h>

#include "driver/i2c.h"
#include "esp_heap_caps.h"

#include "esp_lcd_sh8601.h"
#include "lcd_config.h"
#include "ui_demo.h"

static SemaphoreHandle_t lvgl_mux = NULL;
#define LCD_HOST     SPI2_HOST
#define TOUCH_I2C    I2C_NUM_0

static esp_lcd_panel_io_handle_t amoled_panel_io_handle = NULL;

// ── LCD init sequence ────────────────────────────────────────────────────────

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 80},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 1},
    {0x63, (uint8_t[]){0xFF}, 1, 1},
    {0x51, (uint8_t[]){0x00}, 1, 1},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
};

// ── Touch (FT3168 over I2C) ──────────────────────────────────────────────────

static uint8_t touch_i2c_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  uint8_t *pbuf = (uint8_t *)malloc(len + 1);
  pbuf[0] = reg;
  for (uint8_t i = 0; i < len; i++) pbuf[i + 1] = buf[i];
  uint8_t ret = i2c_master_write_to_device(TOUCH_I2C, addr, pbuf, len + 1, 1000);
  free(pbuf);
  return ret;
}

static uint8_t touch_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  return i2c_master_write_read_device(TOUCH_I2C, addr, &reg, 1, buf, len, 1000);
}

static void touch_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = PIN_NUM_TOUCH_SDA,
      .scl_io_num = PIN_NUM_TOUCH_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {.clk_speed = 300 * 1000},
      .clk_flags = 0,
  };
  ESP_ERROR_CHECK(i2c_param_config(TOUCH_I2C, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(TOUCH_I2C, conf.mode, 0, 0, 0));

  uint8_t data = 0x00;
  touch_i2c_write(I2C_ADDR_FT3168, 0x00, &data, 1);
}

static uint8_t touch_get(uint16_t *x, uint16_t *y) {
  uint8_t data;
  uint8_t buf[4];
  touch_i2c_read(I2C_ADDR_FT3168, 0x02, &data, 1);
  if (data) {
    touch_i2c_read(I2C_ADDR_FT3168, 0x03, buf, 4);
    *x = (((uint16_t)buf[0] & 0x0f) << 8) | (uint16_t)buf[1];
    *y = (((uint16_t)buf[2] & 0x0f) << 8) | (uint16_t)buf[3];
    if (*x > LCD_H_RES) *x = LCD_H_RES;
    if (*y > LCD_V_RES) *y = LCD_V_RES;
    return 1;
  }
  return 0;
}

// ── LVGL port ────────────────────────────────────────────────────────────────

static bool lvgl_notify_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx) {
  (void)panel_io;
  (void)edata;
  lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
  lv_disp_flush_ready(disp_driver);
  return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  const int offsetx1 = area->x1 + 0x14;
  const int offsetx2 = area->x2 + 0x14;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area) {
  (void)disp_drv;
  uint16_t x1 = area->x1;
  uint16_t x2 = area->x2;
  uint16_t y1 = area->y1;
  uint16_t y2 = area->y2;

  area->x1 = (x1 >> 1) << 1;
  area->y1 = (y1 >> 1) << 1;
  area->x2 = ((x2 >> 1) << 1) + 1;
  area->y2 = ((y2 >> 1) << 1) + 1;
}

static void lvgl_tick_cb(void *arg) {
  (void)arg;
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_unlock(void) {
  assert(lvgl_mux && "lcd_lvgl_Init must be called first");
  xSemaphoreGive(lvgl_mux);
}

static bool lvgl_lock(int timeout_ms) {
  assert(lvgl_mux && "lcd_lvgl_Init must be called first");
  const TickType_t timeout_ticks =
      (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  (void)drv;
  uint16_t tp_x, tp_y;
  if (touch_get(&tp_x, &tp_y)) {
    data->point.x = tp_x;
    data->point.y = tp_y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void lvgl_port_task(void *arg) {
  (void)arg;
  uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
  for (;;) {
    if (lvgl_lock(-1)) {
      task_delay_ms = lv_timer_handler();
      lvgl_unlock();
    }
    if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
      task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
      task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
    }
    vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
  }
}

// ── Public API ───────────────────────────────────────────────────────────────

void lcd_lvgl_Init(void) {
  touch_init();

  static lv_disp_draw_buf_t disp_buf;
  static lv_disp_drv_t disp_drv;

  const spi_bus_config_t buscfg =
      SH8601_PANEL_BUS_QSPI_CONFIG(PIN_NUM_LCD_PCLK,
                                   PIN_NUM_LCD_DATA0,
                                   PIN_NUM_LCD_DATA1,
                                   PIN_NUM_LCD_DATA2,
                                   PIN_NUM_LCD_DATA3,
                                   LCD_H_RES * LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
  ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t io_handle = NULL;
  const esp_lcd_panel_io_spi_config_t io_config =
      SH8601_PANEL_IO_QSPI_CONFIG(PIN_NUM_LCD_CS,
                                  lvgl_notify_flush_ready,
                                  &disp_drv);

  sh8601_vendor_config_t vendor_config = {
      .init_cmds = lcd_init_cmds,
      .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
      .flags = {.use_qspi_interface = 1},
  };

  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
  amoled_panel_io_handle = io_handle;

  esp_lcd_panel_handle_t panel_handle = NULL;
  const esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_NUM_LCD_RST,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel = LCD_BIT_PER_PIXEL,
      .vendor_config = &vendor_config,
  };

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_disp_on_off(panel_handle, true));

  lv_init();

  lv_color_t *buf1 =
      (lv_color_t *)heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf1);
  lv_color_t *buf2 =
      (lv_color_t *)heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf2);

  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LVGL_BUF_HEIGHT);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.rounder_cb = lvgl_rounder_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;

#if (LVGL_ROTATION_DEG == 90)
  // Let LVGL rotate the rendered buffer before flushing to the panel.
  disp_drv.sw_rotate = 1;
  disp_drv.rotated = LV_DISP_ROT_90;
#elif (LVGL_ROTATION_DEG == 180)
  disp_drv.sw_rotate = 1;
  disp_drv.rotated = LV_DISP_ROT_180;
#elif (LVGL_ROTATION_DEG == 270)
  disp_drv.sw_rotate = 1;
  disp_drv.rotated = LV_DISP_ROT_270;
#endif

  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
  (void)disp;

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.disp = disp;
  indev_drv.read_cb = lvgl_touch_cb;
  lv_indev_drv_register(&indev_drv);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &lvgl_tick_cb,
      .name = "lvgl_tick",
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

  lvgl_mux = xSemaphoreCreateMutex();
  assert(lvgl_mux);

  xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL,
              LVGL_TASK_PRIORITY, NULL);

  if (lvgl_lock(-1)) {
    ui_demo_init();
    lvgl_unlock();
  }
}

esp_err_t set_amoled_backlight(uint8_t brig) {
  uint32_t lcd_cmd = 0x51;
  lcd_cmd &= 0xff;
  lcd_cmd <<= 8;
  lcd_cmd |= 0x02 << 24;
  return esp_lcd_panel_io_tx_param(amoled_panel_io_handle, lcd_cmd, &brig, 1);
}
