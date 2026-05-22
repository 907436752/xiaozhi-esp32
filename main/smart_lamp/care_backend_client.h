#pragma once

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"

#include "lamp_bridge.h"
#include "lamp_protocol.h"

#ifndef CARE_BACKEND_URL
#define CARE_BACKEND_URL "http://192.168.1.100:3000/api/status"
#endif

#ifndef CARE_BACKEND_UPLOAD_PERIOD_MS
#define CARE_BACKEND_UPLOAD_PERIOD_MS 5000
#endif

#ifndef CARE_BACKEND_STATUS_TIMEOUT_MS
#define CARE_BACKEND_STATUS_TIMEOUT_MS 3000
#endif

class CareBackendClient {
public:
    static void Start()
    {
        if (started_) {
            return;
        }

        started_ = true;
        xTaskCreate(UploadTask, "care_backend", 8192, nullptr, 3, nullptr);
        ESP_LOGI(TAG, "Care backend client started: %s", CARE_BACKEND_URL);
    }

private:
    static constexpr const char* TAG = "CareBackend";
    inline static bool started_ = false;

    static const char* StateToString(uint8_t state)
    {
        switch (state) {
            case 0:
                return "IDLE";
            case 1:
                return "LIGHT_ON";
            case 2:
                return "ALARM";
            default:
                return "UNKNOWN";
        }
    }

    static esp_err_t UploadStatus(const s3_status_packet_t& status)
    {
        float lux = status.lux_x10 / 10.0f;
        char body[512] = {0};

        int written = snprintf(
            body,
            sizeof(body),
            "{"
            "\"device_id\":\"lamp_001\","
            "\"presence\":%u,"
            "\"presence_raw\":%u,"
            "\"lux\":%.1f,"
            "\"is_dark\":%u,"
            "\"sos\":%u,"
            "\"alarm\":%u,"
            "\"lamp_level\":%u,"
            "\"state\":\"%s\""
            "}",
            status.presence_recent,
            status.presence_raw,
            lux,
            status.is_dark,
            status.sos_pressed,
            status.alarm,
            status.lamp_level,
            StateToString(status.state)
        );

        if (written <= 0 || written >= (int)sizeof(body)) {
            ESP_LOGW(TAG, "JSON body truncated or invalid");
            return ESP_ERR_INVALID_SIZE;
        }

        esp_http_client_config_t config = {};
        config.url = CARE_BACKEND_URL;
        config.method = HTTP_METHOD_POST;
        config.timeout_ms = 5000;

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client == nullptr) {
            ESP_LOGW(TAG, "HTTP client init failed");
            return ESP_FAIL;
        }

        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, body, strlen(body));

        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int status_code = esp_http_client_get_status_code(client);
            ESP_LOGI(TAG, "Backend upload status=%d body=%s", status_code, body);
            if (status_code < 200 || status_code >= 300) {
                err = ESP_FAIL;
            }
        } else {
            ESP_LOGW(TAG, "Backend upload failed: %s", esp_err_to_name(err));
        }

        esp_http_client_cleanup(client);
        return err;
    }

    static void UploadTask(void* arg)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));

        while (true) {
            s3_status_packet_t status = {};

            if (SmartLampBridge::GetInstance().GetLastStatus(&status, CARE_BACKEND_STATUS_TIMEOUT_MS)) {
                esp_err_t ret = UploadStatus(status);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "UploadStatus returned %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGW(TAG, "Upload skipped: S3 status not available");
            }

            vTaskDelay(pdMS_TO_TICKS(CARE_BACKEND_UPLOAD_PERIOD_MS));
        }
    }
};
