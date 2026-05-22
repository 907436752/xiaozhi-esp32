#pragma once

#include <stdint.h>

#define SMART_LAMP_ESPNOW_CHANNEL 6

#define S3_STATUS_PACKET_MAGIC     0x534C5031u
#define S3_STATUS_PACKET_VERSION   1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint32_t seq;

    uint8_t presence_raw;
    uint8_t presence_recent;
    uint16_t lux_x10;

    uint8_t is_dark;
    uint8_t sos_pressed;
    uint8_t alarm;
    uint8_t state;
    uint8_t lamp_level;
} s3_status_packet_t;

#define C5_COMMAND_MAGIC       0x43354331u
#define C5_COMMAND_VERSION     1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint32_t seq;

    uint8_t mode;              // 0=IDLE, 1=AUTO_LIGHT, 2=ALARM
    uint8_t target_brightness; // 0~100
    uint8_t blink_enable;      // 0/1
} c5_command_packet_t;

#define LARGE_DISPLAY_MAGIC    0x534C4431u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t seq;
    uint8_t page;
    uint8_t level;

    char title[32];
    char line1[48];
    char line2[48];
    char line3[48];
} large_display_packet_t;

typedef enum {
    LARGE_PAGE_WEATHER  = 0,
    LARGE_PAGE_MESSAGE  = 1,
    LARGE_PAGE_MEDICINE = 2,
    LARGE_PAGE_VOICE    = 3,
    LARGE_PAGE_ALERT    = 4,
    LARGE_PAGE_CLOCK    = 5
} large_page_t;