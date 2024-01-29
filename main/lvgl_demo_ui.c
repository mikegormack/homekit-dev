/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter

#include "lvgl.h"
#include "esp_log.h"

static lv_obj_t *meter;
static lv_obj_t *textArea;
static lv_obj_t * btn;
lv_style_t style_ta;

static const char *TAG = "UI";

static void set_value(void *indic, int32_t v)
{
    //lv_meter_set_indicator_end_value(meter, indic, v);
}

static void btn_cb(lv_event_t * e)
{
    lv_disp_t *disp = lv_event_get_user_data(e);

    ESP_LOGI(TAG, "Button press");
}

void example_lvgl_demo_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    textArea = lv_textarea_create(scr);
    //lv_obj_center(textArea);
    lv_obj_align(textArea, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_size(textArea, 240, 320);

    lv_style_init(&style_ta);
    lv_style_set_bg_color(&style_ta, lv_color_black());
    lv_style_set_text_color(&style_ta, lv_palette_main(LV_PALETTE_GREEN));
	lv_style_set_text_font(&style_ta, &lv_font_montserrat_12);
    lv_obj_add_style(textArea, &style_ta, 0);

    lv_textarea_set_text(textArea, "Testing testing fark# testing testing testing testing testing 123");

   /* meter = lv_meter_create(scr);
    lv_obj_center(meter);
    lv_obj_set_size(meter, 200, 200);

    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 8, 4, 15, lv_color_black(), 10);

    lv_meter_indicator_t *indic;

    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    indic = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);*/

    btn = lv_btn_create(scr);
    lv_obj_t * lbl = lv_label_create(btn);
    lv_label_set_text_static(lbl, LV_SYMBOL_REFRESH" ROTATE");
    lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, 30, -30);

    lv_obj_add_event_cb(btn, btn_cb, LV_EVENT_CLICKED, disp);

	/*lv_obj_t * label = lv_label_create(scr);
	lv_style_t style;
    lv_style_init(&style);

	lv_style_set_text_font(&style, &lv_font_montserrat_28);

	lv_obj_add_style(label, &style, 0);

    lv_label_set_text(label, "Test");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0,20);*/
    //lv_obj_center(label);


    /*lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_value);
    lv_anim_set_var(&a, indic);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, 2000);
    lv_anim_set_repeat_delay(&a, 100);
    lv_anim_set_playback_time(&a, 500);
    lv_anim_set_playback_delay(&a, 100);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);*/
}
