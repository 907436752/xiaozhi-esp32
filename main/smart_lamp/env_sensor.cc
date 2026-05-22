#include "env_sensor.h"
#include "lamp_bridge.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_sntp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "bme68x.h"
}

#define TAG "BME690_ENV"

#define CLOCK_SYNC_SEND_INTERVAL_MS     (60 * 1000)
#define CLOCK_SYNC_RETRY_INTERVAL_MS    (10 * 1000)

static struct bme68x_dev s_bme_dev = {};
static env_sensor_data_t s_latest = {};
static bool s_has_latest = false;

static env_care_state_t s_latest_care = {};
static bool s_has_latest_care = false;

static bool s_sntp_started = false;
static TickType_t s_last_clock_sync_tick = 0;
static TickType_t s_last_clock_sync_attempt_tick = 0;

static uint32_t elapsed_ms(TickType_t now, TickType_t then)
{
    return (uint32_t)((now - then) * portTICK_PERIOD_MS);
}

static const char* env_level_to_str(env_level_t level)
{
    switch (level) {
        case ENV_LEVEL_NORMAL: return "normal";
        case ENV_LEVEL_NOTICE: return "notice";
        case ENV_LEVEL_WARNING: return "warning";
        default: return "unknown";
    }
}

static void ensure_sntp_started()
{
    if (s_sntp_started) {
        return;
    }

    setenv("TZ", "CST-8", 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();

    s_sntp_started = true;
    ESP_LOGI(TAG, "SNTP started for S3 clock sync");
}

static bool get_local_clock_text(char* time_buf,
                                 size_t time_size,
                                 char* date_buf,
                                 size_t date_size)
{
    time_t now = 0;
    struct tm timeinfo = {};

    time(&now);
    localtime_r(&now, &timeinfo);

    int year = timeinfo.tm_year + 1900;
    if (year < 2024) {
        return false;
    }

    snprintf(time_buf,
             time_size,
             "%02d:%02d:%02d",
             timeinfo.tm_hour,
             timeinfo.tm_min,
             timeinfo.tm_sec);

    snprintf(date_buf,
             date_size,
             "%04d-%02d-%02d",
             year,
             timeinfo.tm_mon + 1,
             timeinfo.tm_mday);

    return true;
}

static void maybe_send_clock_sync()
{
    SmartLampBridge& bridge = SmartLampBridge::GetInstance();

    if (!bridge.IsOnline(10000)) {
        return;
    }

    TickType_t now = xTaskGetTickCount();

    if (s_last_clock_sync_tick != 0 &&
        elapsed_ms(now, s_last_clock_sync_tick) < CLOCK_SYNC_SEND_INTERVAL_MS) {
        return;
    }

    if (s_last_clock_sync_attempt_tick != 0 &&
        elapsed_ms(now, s_last_clock_sync_attempt_tick) < CLOCK_SYNC_RETRY_INTERVAL_MS) {
        return;
    }

    s_last_clock_sync_attempt_tick = now;
    ensure_sntp_started();

    char time_buf[16];
    char date_buf[24];

    if (!get_local_clock_text(time_buf, sizeof(time_buf), date_buf, sizeof(date_buf))) {
        ESP_LOGW(TAG, "System time is not synced yet, skip S3 clock sync");
        return;
    }

    bridge.SendLargeDisplay(
        LARGE_PAGE_CLOCK,
        0,
        time_buf,
        date_buf,
        "",
        ""
    );

    s_last_clock_sync_tick = now;
    ESP_LOGI(TAG, "Clock sync sent to S3: %s %s", date_buf, time_buf);
}

static temp_state_t classify_temp(float t)
{
    if (t < 16.0f) {
        return TEMP_VERY_COLD;
    }
    if (t < 18.0f) {
        return TEMP_COLD;
    }
    if (t < 23.0f) {
        return TEMP_COOL_COMFORT;
    }
    if (t <= 27.0f) {
        return TEMP_COMFORT;
    }
    if (t < 30.0f) {
        return TEMP_HOT;
    }
    return TEMP_VERY_HOT;
}

static humidity_state_t classify_humidity(float h)
{
    if (h < 25.0f) {
        return HUM_VERY_DRY;
    }
    if (h < 30.0f) {
        return HUM_DRY;
    }
    if (h < 45.0f) {
        return HUM_COMFORT_DRY;
    }
    if (h <= 60.0f) {
        return HUM_COMFORT;
    }
    if (h < 70.0f) {
        return HUM_HUMID;
    }
    return HUM_VERY_HUMID;
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int calc_comfort_score(float temp_c, float humidity)
{
    int score = 100;

    score -= (int)(fabsf(temp_c - 24.0f) * 4.0f);
    score -= (int)(fabsf(humidity - 45.0f) * 1.0f);

    return clamp_int(score, 0, 100);
}

static void set_care_text(env_care_state_t* out,
                          env_level_t level,
                          const char* title,
                          const char* summary,
                          const char* suggestion)
{
    out->level = level;
    snprintf(out->title, sizeof(out->title), "%s", title);
    snprintf(out->summary, sizeof(out->summary), "%s", summary);
    snprintf(out->suggestion, sizeof(out->suggestion), "%s", suggestion);
}

static env_care_state_t evaluate_env_care(float temp_c, float humidity)
{
    env_care_state_t out = {};

    out.temp_state = classify_temp(temp_c);
    out.humidity_state = classify_humidity(humidity);
    out.comfort_score = calc_comfort_score(temp_c, humidity);
    out.valid = true;

    if (temp_c >= 30.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_WARNING,
            "High Temperature",
            "Room temperature is too high.",
            "Please ventilate or cool the room."
        );
        return out;
    }

    if (humidity >= 70.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_WARNING,
            "High Humidity",
            "Room humidity is too high.",
            "Please ventilate or dehumidify."
        );
        return out;
    }

    if (temp_c < 18.0f && humidity < 30.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_NOTICE,
            "Cold & Dry",
            "Room is cold and dry.",
            "Keep warm and consider humidifying."
        );
        return out;
    }

    if (temp_c < 18.0f && humidity >= 60.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_NOTICE,
            "Cold & Humid",
            "Room feels cold and damp.",
            "Keep warm and ventilate properly."
        );
        return out;
    }

    if (temp_c > 27.0f && humidity < 30.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_NOTICE,
            "Hot & Dry",
            "Room is warm and dry.",
            "Ventilate and drink some water."
        );
        return out;
    }

    if (temp_c > 27.0f && humidity >= 60.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_NOTICE,
            "Hot & Humid",
            "Room feels stuffy.",
            "Ventilation or cooling is suggested."
        );
        return out;
    }

    if (humidity < 30.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_NOTICE,
            "Dry Air",
            "Humidity is low.",
            "Consider drinking water or humidifying."
        );
        return out;
    }

    if (temp_c < 18.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_NOTICE,
            "Low Temperature",
            "Room is a little cold.",
            "Please keep warm."
        );
        return out;
    }

    if (temp_c > 27.0f) {
        set_care_text(
            &out,
            ENV_LEVEL_NOTICE,
            "Warm Room",
            "Room is a little warm.",
            "Ventilation is suggested."
        );
        return out;
    }

    set_care_text(
        &out,
        ENV_LEVEL_NORMAL,
        "Comfortable",
        "Room environment is suitable.",
        "Keep the current environment."
    );

    return out;
}

static void send_env_to_large_clock(const env_sensor_data_t& data)
{
    SmartLampBridge& bridge = SmartLampBridge::GetInstance();

    if (!bridge.IsOnline(10000)) {
        ESP_LOGW(TAG, "Skip large display env update: S3 bridge is not online yet");
        return;
    }

    char temp_buf[16];
    char humid_buf[16];

    snprintf(temp_buf, sizeof(temp_buf), "%.1f C", (double)data.temperature_c);
    snprintf(humid_buf, sizeof(humid_buf), "%.0f %%", (double)data.humidity_percent);

    bridge.SendLargeDisplay(
        LARGE_PAGE_CLOCK,
        0,
        "",
        "",
        temp_buf,
        humid_buf
    );
}

Bme690EnvSensor::Bme690EnvSensor(i2c_master_bus_handle_t bus, uint8_t addr)
    : bus_(bus), addr_(addr)
{
}

bool Bme690EnvSensor::Start()
{
    if (started_) {
        return true;
    }

    if (bus_ == nullptr) {
        ESP_LOGE(TAG, "I2C bus is null");
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_,
        .scl_speed_hz = 100000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BME690 I2C device 0x%02X: %s",
                 addr_, esp_err_to_name(err));
        return false;
    }

    if (!InitDevice()) {
        ESP_LOGE(TAG, "BME690 init failed");
        return false;
    }

    started_ = true;

    xTaskCreate(SensorTask, "bme690_task", 4096, this, 3, nullptr);

    ESP_LOGI(TAG, "BME690 started at I2C address 0x%02X", addr_);
    return true;
}

bool Bme690EnvSensor::InitDevice()
{
    memset(&s_bme_dev, 0, sizeof(s_bme_dev));

    s_bme_dev.intf = BME68X_I2C_INTF;
    s_bme_dev.read = Bme690EnvSensor::I2cRead;
    s_bme_dev.write = Bme690EnvSensor::I2cWrite;
    s_bme_dev.delay_us = Bme690EnvSensor::DelayUs;
    s_bme_dev.intf_ptr = this;
    s_bme_dev.amb_temp = 25;

    int8_t rslt = bme68x_init(&s_bme_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_init failed: %d", rslt);
        return false;
    }

    struct bme68x_conf conf = {};
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;

    rslt = bme68x_set_conf(&conf, &s_bme_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_conf failed: %d", rslt);
        return false;
    }

    struct bme68x_heatr_conf heatr_conf = {};
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 0;
    heatr_conf.heatr_dur = 0;

    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &s_bme_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_heatr_conf failed: %d", rslt);
        return false;
    }

    ESP_LOGI(TAG, "BME690 chip id=0x%02X", s_bme_dev.chip_id);
    return true;
}

bool Bme690EnvSensor::ReadOnce(env_sensor_data_t* out)
{
    if (!started_) {
        return false;
    }

    struct bme68x_conf conf = {};
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;

    int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &s_bme_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGW(TAG, "bme68x_set_op_mode failed: %d", rslt);
        return false;
    }

    uint32_t meas_dur_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &s_bme_dev);
    uint32_t total_delay_us = meas_dur_us + 1000;
    DelayUs(total_delay_us, this);

    struct bme68x_data data = {};
    uint8_t n_fields = 0;

    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &s_bme_dev);
    if (rslt != BME68X_OK || n_fields == 0) {
        ESP_LOGW(TAG, "bme68x_get_data failed: rslt=%d n_fields=%u", rslt, n_fields);
        return false;
    }

    env_sensor_data_t result = {};
    result.temperature_c = data.temperature;
    result.humidity_percent = data.humidity;
    result.pressure_hpa = data.pressure / 100.0f;
    result.pressure_valid = true;

    if (result.pressure_hpa < 850.0f || result.pressure_hpa > 1150.0f) {
        ESP_LOGW(TAG, "Pressure out of range: raw=%.2f, hPa=%.2f",
                (double)data.pressure,
                (double)result.pressure_hpa);

        result.pressure_hpa = 0.0f;
        result.pressure_valid = false;
    }
    result.gas_resistance_ohm = data.gas_resistance;
    result.gas_valid = (data.status & BME68X_GASM_VALID_MSK) != 0;
    result.valid = true;

    s_latest = result;
    s_has_latest = true;

    s_latest_care = evaluate_env_care(result.temperature_c, result.humidity_percent);
    s_has_latest_care = true;

    if (out != nullptr) {
        *out = result;
    }

    return true;
}

bool Bme690EnvSensor::GetLatest(env_sensor_data_t* out)
{
    if (!s_has_latest || out == nullptr) {
        return false;
    }

    *out = s_latest;
    return true;
}

bool Bme690EnvSensor::GetLatestCareState(env_care_state_t* out)
{
    if (!s_has_latest_care || out == nullptr) {
        return false;
    }

    *out = s_latest_care;
    return true;
}

void Bme690EnvSensor::SensorTask(void* arg)
{
    auto* self = static_cast<Bme690EnvSensor*>(arg);

    while (true) {
        env_sensor_data_t data = {};

        if (self->ReadOnce(&data)) {
            env_care_state_t care = {};
            bool has_care = self->GetLatestCareState(&care);

            if (data.pressure_valid) {
                ESP_LOGI(TAG,
                        "T=%.2f C, H=%.2f %%, P=%.2f hPa, Gas=%.0f ohm, gas_valid=%u",
                        data.temperature_c,
                        data.humidity_percent,
                        data.pressure_hpa,
                        data.gas_resistance_ohm,
                        data.gas_valid ? 1 : 0);
            } else {
                ESP_LOGI(TAG,
                        "T=%.2f C, H=%.2f %%, P=N/A, Gas=%.0f ohm, gas_valid=%u",
                        data.temperature_c,
                        data.humidity_percent,
                        data.gas_resistance_ohm,
                        data.gas_valid ? 1 : 0);
            }

            send_env_to_large_clock(data);
            maybe_send_clock_sync();

            if (has_care) {
                ESP_LOGI(TAG,
                        "ENV: %s, score=%d, level=%s, summary=%s, tip=%s",
                        care.title,
                        care.comfort_score,
                        env_level_to_str(care.level),
                        care.summary,
                        care.suggestion);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

int8_t Bme690EnvSensor::I2cRead(uint8_t reg_addr,
                                uint8_t* reg_data,
                                uint32_t len,
                                void* intf_ptr)
{
    auto* self = static_cast<Bme690EnvSensor*>(intf_ptr);

    esp_err_t err = i2c_master_transmit_receive(
        self->dev_handle_,
        &reg_addr,
        1,
        reg_data,
        len,
        pdMS_TO_TICKS(1000)
    );

    return err == ESP_OK ? BME68X_OK : BME68X_E_COM_FAIL;
}

int8_t Bme690EnvSensor::I2cWrite(uint8_t reg_addr,
                                 const uint8_t* reg_data,
                                 uint32_t len,
                                 void* intf_ptr)
{
    auto* self = static_cast<Bme690EnvSensor*>(intf_ptr);

    uint8_t buffer[64] = {};
    if (len + 1 > sizeof(buffer)) {
        return BME68X_E_COM_FAIL;
    }

    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, len);

    esp_err_t err = i2c_master_transmit(
        self->dev_handle_,
        buffer,
        len + 1,
        pdMS_TO_TICKS(1000)
    );

    return err == ESP_OK ? BME68X_OK : BME68X_E_COM_FAIL;
}

void Bme690EnvSensor::DelayUs(uint32_t period_us, void* intf_ptr)
{
    if (period_us < 1000) {
        esp_rom_delay_us(period_us);
    } else {
        vTaskDelay(pdMS_TO_TICKS((period_us + 999) / 1000));
    }
}

bool EnvSensorGetLatest(env_sensor_data_t* out)
{
    if (!s_has_latest || out == nullptr) {
        return false;
    }

    *out = s_latest;
    return true;
}

bool EnvSensorGetLatestCareState(env_care_state_t* out)
{
    if (!s_has_latest_care || out == nullptr) {
        return false;
    }

    *out = s_latest_care;
    return true;
}

const char* EnvLevelToString(env_level_t level)
{
    switch (level) {
        case ENV_LEVEL_NORMAL:
            return "normal";
        case ENV_LEVEL_NOTICE:
            return "notice";
        case ENV_LEVEL_WARNING:
            return "warning";
        default:
            return "unknown";
    }
}
