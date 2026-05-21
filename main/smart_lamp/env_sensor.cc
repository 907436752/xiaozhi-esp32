#include "env_sensor.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_rom_sys.h"

extern "C" {
#include "bme68x.h"
}

#define TAG "BME690_ENV"

static struct bme68x_dev s_bme_dev = {};
static env_sensor_data_t s_latest = {};
static bool s_has_latest = false;

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
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;

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

    struct bme68x_heatr_conf heatr_conf = {};
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;

    int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &s_bme_dev);
    if (rslt != BME68X_OK) {
        ESP_LOGW(TAG, "bme68x_set_op_mode failed: %d", rslt);
        return false;
    }

    uint32_t meas_dur_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &s_bme_dev);
    uint32_t total_delay_us = meas_dur_us + heatr_conf.heatr_dur * 1000;
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
    result.gas_resistance_ohm = data.gas_resistance;
    result.gas_valid = (data.status & BME68X_GASM_VALID_MSK) != 0;
    result.valid = true;

    s_latest = result;
    s_has_latest = true;

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

void Bme690EnvSensor::SensorTask(void* arg)
{
    auto* self = static_cast<Bme690EnvSensor*>(arg);

    while (true) {
        env_sensor_data_t data = {};

        if (self->ReadOnce(&data)) {
            ESP_LOGI(TAG,
                     "T=%.2f C, H=%.2f %%, P=%.2f hPa, Gas=%.0f ohm, gas_valid=%u",
                     data.temperature_c,
                     data.humidity_percent,
                     data.pressure_hpa,
                     data.gas_resistance_ohm,
                     data.gas_valid ? 1 : 0);
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