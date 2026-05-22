#include "nurse_ui.h"

#include <string.h>

LV_IMAGE_DECLARE(nurse_sprite_closed);
LV_IMAGE_DECLARE(nurse_sprite_mouth_open);

static const void* k_nurse_frames[] = {
    &nurse_sprite_closed,
    &nurse_sprite_mouth_open,
};

static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_img = nullptr;
static lv_obj_t* s_status = nullptr;
static lv_obj_t* s_tip = nullptr;
static lv_obj_t* s_dot[3] = {};
static lv_obj_t* s_glow = nullptr;
static lv_timer_t* s_timer = nullptr;

static nurse_ui_state_t s_state = NURSE_UI_IDLE;
static uint32_t s_tick = 0;
static uint8_t s_last_frame = 0xFF;

static lv_color_t color_idle   = lv_color_hex(0x6EDCFF);
static lv_color_t color_listen = lv_color_hex(0x5C8DFF);
static lv_color_t color_speak  = lv_color_hex(0x7CFFCB);
static lv_color_t color_alert  = lv_color_hex(0xFF4B4B);

static void nurse_set_frame(uint8_t frame)
{
    if (!s_img) {
        return;
    }

    if (frame > 1) {
        frame = 0;
    }

    if (s_last_frame == frame) {
        return;
    }

    lv_image_set_src(s_img, k_nurse_frames[frame]);

    // 固定缩放和对齐。这里不能跟着嘴型帧做呼吸缩放，否则说话时会忽大忽小。
    lv_image_set_scale(s_img, 256);
    lv_obj_align(s_img, LV_ALIGN_CENTER, 0, 2);

    s_last_frame = frame;
}

static lv_obj_t* make_dot(lv_obj_t* parent)
{
    lv_obj_t* dot = lv_obj_create(parent);
    lv_obj_set_size(dot, 7, 7);
    lv_obj_set_style_radius(dot, 4, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_set_style_bg_color(dot, color_idle, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    return dot;
}

static const char* state_title(nurse_ui_state_t state)
{
    switch (state) {
        case NURSE_UI_IDLE: return "NIGHT CARE";
        case NURSE_UI_LISTENING: return "LISTENING";
        case NURSE_UI_THINKING: return "THINKING";
        case NURSE_UI_SPEAKING: return "SPEAKING";
        case NURSE_UI_ALERT: return "SOS ALERT";
        default: return "NIGHT CARE";
    }
}

static lv_color_t state_color(nurse_ui_state_t state)
{
    switch (state) {
        case NURSE_UI_IDLE: return color_idle;
        case NURSE_UI_LISTENING: return color_listen;
        case NURSE_UI_THINKING: return color_listen;
        case NURSE_UI_SPEAKING: return color_speak;
        case NURSE_UI_ALERT: return color_alert;
        default: return color_idle;
    }
}

static void update_state_style()
{
    if (!s_root) {
        return;
    }

    lv_color_t c = state_color(s_state);

    if (s_status) {
        lv_label_set_text(s_status, state_title(s_state));
        lv_obj_set_style_text_color(s_status, c, 0);
    }

    if (s_glow) {
        lv_obj_set_style_border_color(s_glow, c, 0);
        lv_obj_set_style_shadow_color(s_glow, c, 0);
    }

    bool show_dots =
        s_state == NURSE_UI_LISTENING ||
        s_state == NURSE_UI_THINKING ||
        s_state == NURSE_UI_ALERT;

    for (int i = 0; i < 3; i++) {
        if (!s_dot[i]) {
            continue;
        }

        lv_obj_set_style_bg_color(s_dot[i], c, 0);

        if (show_dots) {
            lv_obj_clear_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void nurse_timer_cb(lv_timer_t* timer)
{
    (void)timer;

    if (!s_root || !s_img) {
        return;
    }

    s_tick++;

    // 小护士图像本体不做任何缩放动画，只切换嘴型帧。
    // 否则状态在 SPEAKING/IDLE 之间快速跳变时，会看到人物尺寸抖动。
    if (s_state == NURSE_UI_SPEAKING) {
        nurse_set_frame((s_tick % 2) ? 1 : 0);
    } else {
        nurse_set_frame(0);
    }

    if (s_glow) {
        lv_opa_t opa = LV_OPA_30;

        if (s_state == NURSE_UI_ALERT) {
            opa = (s_tick % 2) ? LV_OPA_70 : LV_OPA_20;
        } else if (s_state == NURSE_UI_LISTENING || s_state == NURSE_UI_THINKING) {
            opa = (s_tick % 2) ? LV_OPA_50 : LV_OPA_20;
        } else if (s_state == NURSE_UI_SPEAKING) {
            opa = LV_OPA_40;
        }

        lv_obj_set_style_shadow_opa(s_glow, opa, 0);
    }

    if (s_state == NURSE_UI_LISTENING ||
        s_state == NURSE_UI_THINKING ||
        s_state == NURSE_UI_ALERT) {
        for (int i = 0; i < 3; i++) {
            if (!s_dot[i]) {
                continue;
            }

            int local = (s_tick + i * 4) % 12;
            lv_opa_t opa = local < 6 ? LV_OPA_100 : LV_OPA_30;
            lv_obj_set_style_bg_opa(s_dot[i], opa, 0);
        }
    }
}

void NurseUiCreate(lv_obj_t* parent)
{
    if (parent == nullptr || s_root != nullptr) {
        return;
    }

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, 240, 200);
    lv_obj_align(s_root, LV_ALIGN_CENTER, 0, 8);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 0, 0);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    s_glow = lv_obj_create(s_root);
    lv_obj_set_size(s_glow, 150, 150);
    lv_obj_align(s_glow, LV_ALIGN_CENTER, 0, 5);
    lv_obj_set_style_radius(s_glow, 75, 0);
    lv_obj_set_style_bg_opa(s_glow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_glow, 1, 0);
    lv_obj_set_style_border_color(s_glow, color_idle, 0);
    lv_obj_set_style_shadow_width(s_glow, 24, 0);
    lv_obj_set_style_shadow_opa(s_glow, LV_OPA_30, 0);
    lv_obj_set_style_shadow_color(s_glow, color_idle, 0);
    lv_obj_clear_flag(s_glow, LV_OBJ_FLAG_SCROLLABLE);

    s_img = lv_image_create(s_root);
    lv_image_set_scale(s_img, 256);
    lv_obj_align(s_img, LV_ALIGN_CENTER, 0, 2);
    nurse_set_frame(0);

    s_status = lv_label_create(s_root);
    lv_obj_set_width(s_status, 220);
    lv_label_set_text(s_status, "NIGHT CARE");
    lv_obj_set_style_text_align(s_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_status, color_idle, 0);
    lv_obj_align(s_status, LV_ALIGN_TOP_MID, 0, 0);

    s_tip = lv_label_create(s_root);
    lv_obj_set_width(s_tip, 220);
    lv_label_set_text(s_tip, "Smart Care Lamp");
    lv_obj_set_style_text_align(s_tip, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_tip, lv_color_hex(0xAAB6C8), 0);
    lv_obj_align(s_tip, LV_ALIGN_BOTTOM_MID, 0, 0);

    for (int i = 0; i < 3; i++) {
        s_dot[i] = make_dot(s_root);
        lv_obj_align(s_dot[i], LV_ALIGN_BOTTOM_MID, -16 + i * 16, -18);
        lv_obj_add_flag(s_dot[i], LV_OBJ_FLAG_HIDDEN);
    }

    update_state_style();

    if (s_timer == nullptr) {
        s_timer = lv_timer_create(nurse_timer_cb, 180, nullptr);
    }
}

void NurseUiSetState(nurse_ui_state_t state)
{
    if (s_state == state) {
        return;
    }

    s_state = state;
    update_state_style();

    if (s_state != NURSE_UI_SPEAKING) {
        nurse_set_frame(0);
    }
}

void NurseUiSetTip(const char* text)
{
    if (s_tip == nullptr || text == nullptr) {
        return;
    }

    lv_label_set_text(s_tip, text);
}
