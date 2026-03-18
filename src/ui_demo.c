#include "ui_demo.h"

#include "lvgl.h"

#include "lcd_bsp.h"
#include "lcd_config.h"

static lv_obj_t *scr;
static lv_obj_t *title;
static lv_obj_t *lbl_backlight;

static void backlight_slider_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int32_t v = lv_slider_get_value(slider);
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  set_amoled_backlight((uint8_t)v);
}

static void invert_cb(lv_event_t *e) {
  lv_obj_t *btn = lv_event_get_target(e);
  bool inverted = lv_obj_has_state(btn, LV_STATE_CHECKED);

  lv_color_t bg = inverted ? lv_color_black() : lv_color_white();
  lv_color_t fg = inverted ? lv_color_white() : lv_color_black();

  lv_obj_set_style_bg_color(scr, bg, LV_PART_MAIN);
  lv_obj_set_style_text_color(title, fg, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_backlight, fg, LV_PART_MAIN);
}

void ui_demo_init(void) {
  scr = lv_scr_act();

  title = lv_label_create(scr);
  lv_label_set_text(title, "ESP32-S3 Touch AMOLED 1.64");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_22, LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lbl_backlight = lv_label_create(scr);
  lv_label_set_text(lbl_backlight, "Backlight");
  lv_obj_set_style_text_font(lbl_backlight, &lv_font_montserrat_22, LV_PART_MAIN);
  lv_obj_align(lbl_backlight, LV_ALIGN_TOP_LEFT, 10, 70);

  lv_obj_t *slider = lv_slider_create(scr);
  lv_slider_set_range(slider, 0, 255);
  lv_slider_set_value(slider, 255, LV_ANIM_OFF);
  lv_obj_set_width(slider, 240);
  lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 95);
  lv_obj_add_event_cb(slider, backlight_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn, 160, 50);
  lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 150);
  lv_obj_add_event_cb(btn, invert_cb, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t *btn_lbl = lv_label_create(btn);
  lv_label_set_text(btn_lbl, "Invert");
  lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_22, LV_PART_MAIN);
  lv_obj_center(btn_lbl);

  // Set initial backlight
  set_amoled_backlight(255);
}
