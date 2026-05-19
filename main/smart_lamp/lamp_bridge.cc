#include "lamp_bridge.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"

#define TAG "SmartLampBridge"

static const uint8_t kBroadcastMac[6] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

typedef struct {
    uint8_t mac[6];
    s3_status_packet_t packet;
} recv_status_msg_t;

static QueueHandle_t s_recv_queue = nullptr;
static s3_status_packet_t s_last_status = {};
static TickType_t s_last_status_tick = 0;
static bool s_has_status = false;

SmartLampBridge& SmartLampBridge::GetInstance()
{
    static SmartLampBridge instance;
    return instance;
}

static const char* state_to_str(uint8_t state)
{
    switch (state) {
        case 0: return "IDLE";
        case 1: return "LIGHT_ON";
        case 2: return "ALARM";
        default: return "UNKNOWN";
    }
}

void SmartLampBridge::Start()
{
    if (started_) {
        return;
    }

    started_ = true;

    s_recv_queue = xQueueCreate(10, sizeof(recv_status_msg_t));
    if (s_recv_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create recv queue");
        return;
    }

    esp_err_t ret;

    ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps failed: %s", esp_err_to_name(ret));
    }

    // 第一阶段先固定信道 6。后面 C5 上云时再处理“跟随路由器信道”的问题。
    ret = esp_wifi_set_channel(SMART_LAMP_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_channel(%d) failed: %s",
                 SMART_LAMP_ESPNOW_CHANNEL, esp_err_to_name(ret));
    }

    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_now_register_recv_cb(SmartLampBridge::RecvCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(ret));
        return;
    }

    AddBroadcastPeer();

    xTaskCreate(RecvTask, "lamp_recv", 4096, this, 4, nullptr);
    xTaskCreate(CommandTask, "lamp_cmd", 4096, this, 3, nullptr);

    ESP_LOGI(TAG, "Smart lamp ESP-NOW bridge started on channel %d",
             SMART_LAMP_ESPNOW_CHANNEL);
}

void SmartLampBridge::AddBroadcastPeer()
{
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastMac, 6);
    peer.channel = SMART_LAMP_ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(ret));
    }
}

void SmartLampBridge::RecvCallback(const esp_now_recv_info_t* recv_info,
                                   const uint8_t* data,
                                   int data_len)
{
    if (recv_info == nullptr || data == nullptr || data_len <= 0) {
        return;
    }

    if (data_len != sizeof(s3_status_packet_t)) {
        return;
    }

    s3_status_packet_t packet;
    memcpy(&packet, data, sizeof(packet));

    if (packet.magic != S3_STATUS_PACKET_MAGIC ||
        packet.version != S3_STATUS_PACKET_VERSION) {
        return;
    }

    recv_status_msg_t msg = {};
    memcpy(msg.mac, recv_info->src_addr, 6);
    msg.packet = packet;

    if (s_recv_queue != nullptr) {
        xQueueSend(s_recv_queue, &msg, 0);
    }
}

void SmartLampBridge::RecvTask(void* arg)
{
    auto* self = static_cast<SmartLampBridge*>(arg);
    recv_status_msg_t msg;

    while (true) {
        if (xQueueReceive(s_recv_queue, &msg, portMAX_DELAY) == pdTRUE) {
            self->HandleStatusPacket(msg.packet);
        }
    }
}

void SmartLampBridge::HandleStatusPacket(const s3_status_packet_t& packet)
{
    s_last_status = packet;
    s_last_status_tick = xTaskGetTickCount();
    s_has_status = true;

    float lux = packet.lux_x10 / 10.0f;

    ESP_LOGI(TAG,
             "S3 status: seq=%lu raw=%u recent=%u lux=%.1f dark=%u sos=%u alarm=%u state=%s lamp=%u",
             (unsigned long)packet.seq,
             packet.presence_raw,
             packet.presence_recent,
             lux,
             packet.is_dark,
             packet.sos_pressed,
             packet.alarm,
             state_to_str(packet.state),
             packet.lamp_level);
}

bool SmartLampBridge::IsOnline(uint32_t timeout_ms)
{
    if (!s_has_status) {
        return false;
    }

    TickType_t now = xTaskGetTickCount();
    return (now - s_last_status_tick) < pdMS_TO_TICKS(timeout_ms);
}

bool SmartLampBridge::GetLastStatus(s3_status_packet_t* out, uint32_t timeout_ms)
{
    if (!IsOnline(timeout_ms)) {
        return false;
    }

    if (out != nullptr) {
        *out = s_last_status;
    }

    return true;
}

void SmartLampBridge::SendCommand(uint8_t mode, uint8_t brightness, uint8_t blink)
{
    static uint32_t seq = 0;

    if (brightness > 100) {
        brightness = 100;
    }

    c5_command_packet_t packet = {};
    packet.magic = C5_COMMAND_MAGIC;
    packet.version = C5_COMMAND_VERSION;
    packet.seq = seq++;
    packet.mode = mode;
    packet.target_brightness = brightness;
    packet.blink_enable = blink ? 1 : 0;

    esp_err_t ret = esp_now_send(kBroadcastMac,
                                 reinterpret_cast<const uint8_t*>(&packet),
                                 sizeof(packet));

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SendCommand failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SendCommand: seq=%lu mode=%u brightness=%u blink=%u",
                 (unsigned long)packet.seq,
                 packet.mode,
                 packet.target_brightness,
                 packet.blink_enable);
    }
}

void SmartLampBridge::SendLargeDisplay(uint8_t page,
                                       uint8_t level,
                                       const char* title,
                                       const char* line1,
                                       const char* line2,
                                       const char* line3)
{
    static uint16_t seq = 0;

    large_display_packet_t packet = {};
    packet.magic = LARGE_DISPLAY_MAGIC;
    packet.seq = seq++;
    packet.page = page;
    packet.level = level;

    snprintf(packet.title, sizeof(packet.title), "%s", title ? title : "");
    snprintf(packet.line1, sizeof(packet.line1), "%s", line1 ? line1 : "");
    snprintf(packet.line2, sizeof(packet.line2), "%s", line2 ? line2 : "");
    snprintf(packet.line3, sizeof(packet.line3), "%s", line3 ? line3 : "");

    esp_err_t ret = esp_now_send(kBroadcastMac,
                                 reinterpret_cast<const uint8_t*>(&packet),
                                 sizeof(packet));

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SendLargeDisplay failed: %s", esp_err_to_name(ret));
    }
}

void SmartLampBridge::CommandTask(void* arg)
{
    auto* self = static_cast<SmartLampBridge*>(arg);

    while (true) {
        s3_status_packet_t status = {};

        if (self->GetLastStatus(&status, 3000)) {
            if (status.sos_pressed || status.alarm) {
                self->SendCommand(2, 100, 1);
                self->SendLargeDisplay(
                    LARGE_PAGE_ALERT,
                    100,
                    "SOS ALERT",
                    "Please check now.",
                    "Alarm mode active.",
                    "Lamp is blinking."
                );
            } else if (status.presence_recent && status.is_dark) {
                self->SendCommand(1, 70, 0);
            } else {
                self->SendCommand(0, 0, 0);
            }
        } else {
            ESP_LOGW(TAG, "S3 offline or no status packet");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}