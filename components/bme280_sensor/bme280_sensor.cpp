//
// Created by badzxb on 25-3-14.
//


#include "bme280_sensor.h"
#include "esp_log.h"
#include <Cmath>

#include "../project_conf/project_conf.h"

#define I2C_MASTER_FREQ_HZ 100000
#define SEA_LEVEL_PRESSURE 1013.25f
#define TAG "BME280Sensor"

BME280Sensor::BME280Sensor(i2c_port_t i2c_port, uint8_t i2c_addr)
        : _i2c_port(i2c_port), _i2c_addr(i2c_addr), _i2c_bus(nullptr), _bme280(nullptr) {}

BME280Sensor::~BME280Sensor() {
    if (_bme280) {
        bme280_delete(&_bme280);
    }
    if (_i2c_bus) {
        i2c_bus_delete(&_i2c_bus);
    }
}

bool BME280Sensor::init() {
    ESP_LOGI(TAG, "Initializing BME280...");

    i2c_config_bus_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = IIC_BME_SDA_IO,
            .scl_io_num = IIC_BME_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                    .clk_speed = I2C_MASTER_FREQ_HZ
            }
    };

    _i2c_bus = i2c_bus_create(_i2c_port, &conf);
    if (i2c_bus_create(_i2c_port, &conf) == nullptr) {
        ESP_LOGE(TAG, "I2C bus creation failed!");
        return false;
    }

    _bme280 = bme280_create(_i2c_bus, _i2c_addr);
    if (bme280_create(_i2c_bus, _i2c_addr) == nullptr) {
        ESP_LOGE(TAG, "BME280 creation failed!");
        return false;
    }

    if (bme280_default_init(_bme280) != ESP_OK) {
        ESP_LOGE(TAG, "BME280 initialization failed!");
        return false;
    }

    return true;
}

bool BME280Sensor::read(float &press, float &altitude) {
    if (!_bme280) return false;

    if (bme280_read_pressure(_bme280, &press) != ESP_OK) {
        return false;
    }
    bme280_read_altitude(_bme280, SEA_LEVEL_PRESSURE, &altitude);
    // altitude = 44330.0 * (1.0 - pow(press / SEA_LEVEL_PRESSURE, 0.1903));

    return true;
}