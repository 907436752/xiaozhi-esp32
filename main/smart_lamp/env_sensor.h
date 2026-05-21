#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/i2c_master.h"

typedef struct {
    float temperature_c;
    float humidity_percent;

    float pressure_hpa;
    bool pressure_valid;

    float gas_resistance_ohm;
    bool gas_valid;

    bool valid;
} env_sensor_data_t;

class Bme690EnvSensor {
public:
    Bme690EnvSensor(i2c_master_bus_handle_t bus, uint8_t addr = 0x77);

    bool Start();
    bool ReadOnce(env_sensor_data_t* out);
    bool GetLatest(env_sensor_data_t* out);

private:
    static void SensorTask(void* arg);

    static int8_t I2cRead(uint8_t reg_addr,
                          uint8_t* reg_data,
                          uint32_t len,
                          void* intf_ptr);

    static int8_t I2cWrite(uint8_t reg_addr,
                           const uint8_t* reg_data,
                           uint32_t len,
                           void* intf_ptr);

    static void DelayUs(uint32_t period_us, void* intf_ptr);

    bool InitDevice();

private:
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    uint8_t addr_ = 0x77;
    bool started_ = false;
};