#include "env_sensor.h"
#include "lamp_bridge.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_sntp.h"
#include "esp_crt_bundle.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "bme68x.h"
}

#define TAG "BME690_ENV"

#define CLOCK_SYNC_SEND_INTERVAL_MS     (60 * 1000)
#define CLOCK_SYNC_RETRY_INTERVAL_MS    (10 * 1000)
#define WEATHER_SEND_INTERVAL_MS        (30 * 60 * 1000)
#define WEATHER_RETRY_INTERVAL_MS       (60 * 1000)

// Qinhuangdao approximate city center. Open-Meteo does not require an API key.
#define WEATHER_API_URL "https://api.open-meteo.com/v1/forecast?latitude=39.9354&longitude=119.6005&current=temperature_2m,relative_humidity_2m,weather_code,wind_speed_10m,precipitation,rain,snowfall&daily=temperature_2m_max,temperature_2m_min,precipitation_probability_max&timezone=Asia%2FShanghai&forecast_days=1"
#define WEATHER_CITY_NAME "Qinhuangdao"

static struct bme68x_dev s_bme_dev = {};
static env_sensor_data_t s_latest = {};
static bool s_has_latest = false;

static env_care_state_t s_latest_care = {};
static bool s_has_latest_care = false;

static bool s_sntp_started = false;
static TickType_t s_last_clock_sync_tick = 0;
static TickType_t s_last_clock_sync_attempt_tick = 0;
static TickType_t s_last_weather_send_tick = 0;
static TickType_t s_last_weather_attempt_tick = 0;

struct weather_http_buffer_t {
    char *data;
    size_t capacity;
    size_t length;
};

struct weather_data_t {
    char condition[20];
    int now_temp_c;
    int low_temp_c;
    int high_temp_c;
    int humidity_percent;
    int wind_speed_kmh;
    int rain_probability_percent;
};

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
    if (time_buf == nullptr || date_buf == nullptr ||
        time_size < 9 || date_size < 11) {
        return false;
    }

    time_t now = 0;
    struct tm timeinfo = {};

    time(&now);
    localtime_r(&now, &timeinfo);

    int year = timeinfo.tm_year + 1900;
    if (year < 2024) {
        return false;
    }

    if (strftime(time_buf, time_size, "%H:%M:%S", &timeinfo) == 0) {
        return false;
    }

    if (strftime(date_buf, date_size, "%Y-%m-%d", &timeinfo) == 0) {
        return false;
    }

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

static esp_err_t weather_http_event_handler(esp_http_client_event_t *evt)
{
    weather_http_buffer_t *buf = static_cast<weather_http_buffer_t *>(evt->user_data);

    if (evt->event_id == HTTP_EVENT_ON_DATA && buf != nullptr && evt->data != nullptr) {
        if (buf->length + evt->data_len >= buf->capacity) {
            ESP_LOGW(TAG, "Weather response buffer overflow, drop extra data");
            return ESP_OK;
        }

        memcpy(buf->data + buf->length, evt->data, evt->data_len);
        buf->length += evt->data_len;
        buf->data[buf->length] = '\0';
    }

    return ESP_OK;
}

static bool json_find_number_after(const char *json, const char *key, double *out)
{
    if (json == nullptr || key == nullptr || out == nullptr) {
        return false;
    }

    const char *p = strstr(json, key);
    if (p == nullptr) {
        return false;
    }

    p = strchr(p, ':');
    if (p == nullptr) {
        return false;
    }

    ++p;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == '[') {
        ++p;
    }

    char *end = nullptr;
    double value = strtod(p, &end);
    if (end == p) {
        return false;
    }

    *out = value;
    return true;
}

static int round_to_int(double value)
{
    if (value >= 0.0) {
        return (int)(value + 0.5);
    }
    return (int)(value - 0.5);
}

static const char *weather_code_to_condition(int code)
{
    if (code == 0) {
        return "Sunny";
    }
    if (code == 1 || code == 2) {
        return "Partly";
    }
    if (code == 3) {
        return "Cloudy";
    }
    if (code == 45 || code == 48) {
        return "Fog";
    }
    if ((code >= 51 && code <= 67) ||
        (code >= 80 && code <= 82)) {
        return "Rain";
    }
    if ((code >= 71 && code <= 77) ||
        (code >= 85 && code <= 86)) {
        return "Snow";
    }
    if (code >= 95 && code <= 99) {
        return "Thunder";
    }
    return "Cloudy";
}

static bool parse_weather_json(const char *json, weather_data_t *out)
{
    if (json == nullptr || out == nullptr) {
        return false;
    }

    double now_temp = 0.0;
    double humidity = 0.0;
    double weather_code = 0.0;
    double wind_speed = 0.0;
    double high_temp = 0.0;
    double low_temp = 0.0;
    double rain_probability = 0.0;

    if (!json_find_number_after(json, "\"temperature_2m\"", &now_temp) ||
        !json_find_number_after(json, "\"relative_humidity_2m\"", &humidity) ||
        !json_find_number_after(json, "\"weather_code\"", &weather_code) ||
        !json_find_number_after(json, "\"wind_speed_10m\"", &wind_speed) ||
        !json_find_number_after(json, "\"temperature_2m_max\"", &high_temp) ||
        !json_find_number_after(json, "\"temperature_2m_min\"", &low_temp)) {
        return false;
    }

    if (!json_find_number_after(json, "\"precipitation_probability_max\"", &rain_probability)) {
        rain_probability = 0.0;
    }

    snprintf(out->condition, sizeof(out->condition), "%s",
             weather_code_to_condition(round_to_int(weather_code)));
    out->now_temp_c = round_to_int(now_temp);
    out->low_temp_c = round_to_int(low_temp);
    out->high_temp_c = round_to_int(high_temp);
    out->humidity_percent = round_to_int(humidity);
    out->wind_speed_kmh = round_to_int(wind_speed);
    out->rain_probability_percent = round_to_int(rain_probability);

    return true;
}

static bool fetch_weather_from_open_meteo(weather_data_t *out)
{
    if (out == nullptr) {
        return false;
    }

    char *response = static_cast<char *>(calloc(1, 4096));
    if (response == nullptr) {
        ESP_LOGE(TAG, "No memory for weather response");
        return false;
    }

    weather_http_buffer_t buffer = {
        .data = response,
        .capacity = 4096,
        .length = 0,
    };

    esp_http_client_config_t config = {};
    config.url = WEATHER_API_URL;
    config.method = HTTP_METHOD_GET;
    config.timeout_ms = 8000;
    config.event_handler = weather_http_event_handler;
    config.user_data = &buffer;
    config.crt_bundle_attach = esp_crt_bundle_attach;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        free(response);
        ESP_LOGE(TAG, "esp_http_client_init failed");
        return false;
    }

    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status_code != 200) {
        ESP_LOGW(TAG, "Weather HTTP failed: err=%s status=%d len=%u",
                 esp_err_to_name(err), status_code, (unsigned)buffer.length);
        free(response);
        return false;
    }

    bool ok = parse_weather_json(response, out);
    if (!ok) {
        ESP_LOGW(TAG, "Weather JSON parse failed, len=%u", (unsigned)buffer.length);
    }

    free(response);
    return ok;
}

static void maybe_send_real_weather()
{
    SmartLampBridge& bridge = SmartLampBridge::GetInstance();

    if (!bridge.IsOnline(10000)) {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    if (s_last_weather_send_tick != 0 &&
        elapsed_ms(now, s_last_weather_send_tick) < WEATHER_SEND_INTERVAL_MS) {
        return;
    }

    if (s_last_weather_attempt_tick != 0 &&
        elapsed_ms(now, s_last_weather_attempt_tick) < WEATHER_RETRY_INTERVAL_MS) {
        return;
    }
    s_last_weather_attempt_tick = now;

    ensure_sntp_started();

    weather_data_t weather = {};
    if (!fetch_weather_from_open_meteo(&weather)) {
        return;
    }

    char title[64];
    char humidity_buf[16];
    char wind_buf[16];
    char rain_buf[16];

    snprintf(title, sizeof(title), "W|0|%s|%d|%d|%d",
            weather.condition,
            weather.now_temp_c,
            weather.low_temp_c,
            weather.high_temp_c);
    snprintf(humidity_buf, sizeof(humidity_buf), "%d%%", weather.humidity_percent);
    snprintf(wind_buf, sizeof(wind_buf), "%d km/h", weather.wind_speed_kmh);
    snprintf(rain_buf, sizeof(rain_buf), "%d%%", weather.rain_probability_percent);

    bridge.SendLargeDisplay(
        LARGE_PAGE_WEATHER,
        0,
        title,
        humidity_buf,
        wind_buf,
        rain_buf
    );

    s_last_weather_send_tick = now;
    ESP_LOGI(TAG,
             "Weather sent to S3: %s temp=%d low=%d high=%d hum=%d wind=%d rain=%d",
             weather.condition,
             weather.now_temp_c,
             weather.low_temp_c,
             weather.high_temp_c,
             weather.humidity_percent,
             weather.wind_speed_kmh,
             weather.rain_probability_percent);
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

    ensure_sntp_started();

    char time_buf[16] = "";
    char date_buf[24] = "";
    char temp_buf[16];
    char humid_buf[16];

    (void)get_local_clock_text(time_buf, sizeof(time_buf), date_buf, sizeof(date_buf));

    snprintf(temp_buf, sizeof(temp_buf), "%.1f C", (double)data.temperature_c);
    snprintf(humid_buf, sizeof(humid_buf), "%.0f %%", (double)data.humidity_percent);

    bridge.SendLargeDisplay(
        LARGE_PAGE_CLOCK,
        0,
        time_buf,
        date_buf,
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
            maybe_send_real_weather();

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
