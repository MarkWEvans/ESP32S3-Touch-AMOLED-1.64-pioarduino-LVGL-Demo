#include "ui_demo.h"

#include "lvgl.h"

#include "lcd_bsp.h"
#include "lcd_config.h"

static void backlight_slider_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int32_t v = lv_slider_get_value(slider);
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  set_amoled_backlight((uint8_t)v);
}

void ui_demo_init(void) {
  lv_obj_t *scr = lv_scr_act();

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text_fmt(title, "ESP32-S3 Touch AMOLED 1.64 (rot=%d)", (int)EXAMPLE_LVGL_ROTATION_DEG);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t *lbl = lv_label_create(scr);
  lv_label_set_text(lbl, "Backlight");
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 10, 70);

  lv_obj_t *slider = lv_slider_create(scr);
  lv_slider_set_range(slider, 0, 255);
  lv_slider_set_value(slider, 255, LV_ANIM_OFF);
  lv_obj_set_width(slider, 240);
  lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 95);
  lv_obj_add_event_cb(slider, backlight_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Set initial backlight
  set_amoled_backlight(255);
}


