#include "nurse_ui.h"

#include <string.h>

static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_aura = nullptr;
static lv_obj_t* s_head = nullptr;
static lv_obj_t* s_body = nullptr;
static lv_obj_t* s_cap = nullptr;
static lv_obj_t* s_cross_h = nullptr;
static lv_obj_t* s_cross_v = nullptr;
static lv_obj_t* s_eye_l = nullptr;
static lv_obj_t* s_eye_r = nullptr;
static lv_obj_t* s_mouth = nullptr;
static lv_obj_t* s_status = nullptr;
static lv_obj_t* s_tip = nullptr;
static lv_obj_t* s_dot[3] = {};
static lv_timer_t* s_timer = nullptr;

static nurse_ui_state_t s_state = NURSE_UI_IDLE;
static uint32_t s_tick = 0;

static lv_color_t s_accent_idle = lv_color_hex(0x48D7FF);
static lv_color_t s_accent_listen = lv_color_hex(0x4A8DFF);
static lv_color_t s_accent_speak = lv_color_hex(0x7CFFCB);
static lv_color_t s_accent_alert = lv_color_hex(0xFF4B4B);

static lv_obj_t* make_rect(lv_obj_t* parent, int w, int h, int radius, lv_color_t color)
{
    lv_obj_t* obj = lv_obj_create(parent);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_radius(obj, radius, 0);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    return obj;
}

static lv_obj_t* make_circle(lv_obj_t* parent, int size, lv_color_t color)
{
    return make_rect(parent, size, size, size / 2, color);
}

static void set_obj_color(lv_obj_t* obj, lv_color_t color)
{
    if (obj) {
        lv_obj_set_style_bg_color(obj, color, 0);
    }
}

static void set_dots_visible(bool visible)
{
    for (int i = 0; i < 3; i++) {
        if (!s_dot[i]) {
            continue;
        }

        if (visible) {
            lv_obj_remove_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void apply_state_style()
{
    if (!s_root) {
        return;
    }

    lv_color_t accent = s_accent_idle;
    const char* title = "NIGHT CARE";

    switch (s_state) {
        case NURSE_UI_IDLE:
            accent = s_accent_idle;
            title = "NIGHT CARE";
            set_dots_visible(false);
            break;

        case NURSE_UI_LISTENING:
            accent = s_accent_listen;
            title = "LISTENING";
            set_dots_visible(true);
            break;

        case NURSE_UI_THINKING:
            accent = s_accent_listen;
            title = "THINKING";
            set_dots_visible(true);
            break;

        case NURSE_UI_SPEAKING:
            accent = s_accent_speak;
            title = "SPEAKING";
            set_dots_visible(false);
            break;

        case NURSE_UI_ALERT:
            accent = s_accent_alert;
            title = "SOS ALERT";
            set_dots_visible(true);
            break;

        default:
            break;
    }

    if (s_status) {
        lv_label_set_text(s_status, title);
        lv_obj_set_style_text_color(s_status, accent, 0);
    }

    if (s_aura) {
        lv_obj_set_style_arc_color(s_aura, accent, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(s_aura, lv_color_hex(0x1A2A38), LV_PART_MAIN);
    }

    set_obj_color(s_cap, lv_color_hex(0xF8FBFF));
    set_obj_color(s_cross_h, s_state == NURSE_UI_ALERT ? s_accent_alert : lv_color_hex(0xFF5A6A));
    set_obj_color(s_cross_v, s_state == NURSE_UI_ALERT ? s_accent_alert : lv_color_hex(0xFF5A6A));
    set_obj_color(s_body, lv_color_hex(0xF6FAFF));
    set_obj_color(s_mouth, accent);

    for (int i = 0; i < 3; i++) {
        set_obj_color(s_dot[i], accent);
    }
}

static void nurse_timer_cb(lv_timer_t* timer)
{
    (void)timer;

    if (!s_root) {
        return;
    }

    s_tick++;

    int phase = s_tick % 40;
    int dy = 0;

    if (phase < 20) {
        dy = -phase / 6;
    } else {
        dy = -(40 - phase) / 6;
    }

    lv_obj_align(s_root, LV_ALIGN_CENTER, 0, dy);

    if (s_aura) {
        int val = 35 + (s_tick * 3) % 50;
        lv_arc_set_value(s_aura, val);
    }

    if (s_state == NURSE_UI_SPEAKING && s_mouth) {
        int w = (s_tick % 2) ? 24 : 14;
        lv_obj_set_width(s_mouth, w);
        lv_obj_align(s_mouth, LV_ALIGN_CENTER, 0, 44);
    }

    if (s_state == NURSE_UI_IDLE && s_mouth) {
        lv_obj_set_width(s_mouth, 16);
        lv_obj_align(s_mouth, LV_ALIGN_CENTER, 0, 44);
    }

    if (s_state == NURSE_UI_LISTENING || s_state == NURSE_UI_THINKING || s_state == NURSE_UI_ALERT) {
        for (int i = 0; i < 3; i++) {
            if (!s_dot[i]) {
                continue;
            }

            int local = (s_tick + i * 4) % 12;
            lv_opa_t opa = local < 6 ? LV_OPA_100 : LV_OPA_30;
            lv_obj_set_style_bg_opa(s_dot[i], opa, 0);
        }
    }

    if (s_state == NURSE_UI_ALERT && s_aura) {
        lv_opa_t opa = (s_tick % 2) ? LV_OPA_100 : LV_OPA_40;
        lv_obj_set_style_arc_opa(s_aura, opa, LV_PART_INDICATOR);
    } else if (s_aura) {
        lv_obj_set_style_arc_opa(s_aura, LV_OPA_90, LV_PART_INDICATOR);
    }
}

void NurseUiCreate(lv_obj_t* parent)
{
    if (parent == nullptr || s_root != nullptr) {
        return;
    }

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, 210, 175);
    lv_obj_align(s_root, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 0, 0);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    s_aura = lv_arc_create(s_root);
    lv_obj_set_size(s_aura, 138, 138);
    lv_obj_align(s_aura, LV_ALIGN_CENTER, 0, 4);
    lv_arc_set_range(s_aura, 0, 100);
    lv_arc_set_value(s_aura, 55);
    lv_arc_set_rotation(s_aura, 135);
    lv_arc_set_bg_angles(s_aura, 0, 270);
    lv_obj_remove_style(s_aura, NULL, LV_PART_KNOB);
    lv_obj_remove_flag(s_aura, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_aura, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_aura, 6, LV_PART_INDICATOR);

    s_body = make_rect(s_root, 74, 58, 18, lv_color_hex(0xF6FAFF));
    lv_obj_align(s_body, LV_ALIGN_CENTER, 0, 50);

    s_head = make_circle(s_root, 64, lv_color_hex(0xFFE5D6));
    lv_obj_align(s_head, LV_ALIGN_CENTER, 0, 8);

    s_cap = make_rect(s_root, 70, 24, 8, lv_color_hex(0xF8FBFF));
    lv_obj_align(s_cap, LV_ALIGN_CENTER, 0, -25);

    s_cross_h = make_rect(s_root, 26, 6, 3, lv_color_hex(0xFF5A6A));
    lv_obj_align(s_cross_h, LV_ALIGN_CENTER, 0, -25);

    s_cross_v = make_rect(s_root, 6, 26, 3, lv_color_hex(0xFF5A6A));
    lv_obj_align(s_cross_v, LV_ALIGN_CENTER, 0, -25);

    s_eye_l = make_circle(s_root, 7, lv_color_hex(0x1F2B3A));
    lv_obj_align(s_eye_l, LV_ALIGN_CENTER, -15, 6);

    s_eye_r = make_circle(s_root, 7, lv_color_hex(0x1F2B3A));
    lv_obj_align(s_eye_r, LV_ALIGN_CENTER, 15, 6);

    s_mouth = make_rect(s_root, 16, 4, 2, s_accent_idle);
    lv_obj_align(s_mouth, LV_ALIGN_CENTER, 0, 44);

    s_status = lv_label_create(s_root);
    lv_label_set_text(s_status, "NIGHT CARE");
    lv_obj_set_width(s_status, 200);
    lv_obj_set_style_text_align(s_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_status, s_accent_idle, 0);
    lv_obj_align(s_status, LV_ALIGN_TOP_MID, 0, 0);

    s_tip = lv_label_create(s_root);
    lv_label_set_text(s_tip, "Smart Care Lamp");
    lv_obj_set_width(s_tip, 200);
    lv_obj_set_style_text_align(s_tip, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_tip, lv_color_hex(0xAAB6C8), 0);
    lv_obj_align(s_tip, LV_ALIGN_BOTTOM_MID, 0, 0);

    for (int i = 0; i < 3; i++) {
        s_dot[i] = make_circle(s_root, 7, s_accent_idle);
        lv_obj_align(s_dot[i], LV_ALIGN_CENTER, -16 + i * 16, 75);
        lv_obj_add_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
    }

    apply_state_style();

    if (s_timer == nullptr) {
        s_timer = lv_timer_create(nurse_timer_cb, 120, nullptr);
    }
}
    
void NurseUiSetState(nurse_ui_state_t state)
{
    s_state = state;
    apply_state_style();
}

void NurseUiSetTip(const char* text)
{
    if (s_tip == nullptr || text == nullptr) {
        return;
    }

    lv_label_set_text(s_tip, text);
}