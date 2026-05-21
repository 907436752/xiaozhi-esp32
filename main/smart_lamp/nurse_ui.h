#pragma once

#include <lvgl.h>

typedef enum {
    NURSE_UI_IDLE = 0,
    NURSE_UI_LISTENING,
    NURSE_UI_THINKING,
    NURSE_UI_SPEAKING,
    NURSE_UI_ALERT
} nurse_ui_state_t;

void NurseUiCreate(lv_obj_t* parent);
void NurseUiSetState(nurse_ui_state_t state);
void NurseUiSetTip(const char* text);