//
// Created by badzxb on 25-3-14.
//

#ifndef SUBSIDENCE_MONITOR_BME280_SENSOR_H
#define SUBSIDENCE_MONITOR_BME280_SENSOR_H

#pragma once

// #include "driver/i2c.h"
#include "i2c_bus.h"
#include "bme280.h"
#include <iostream>

class BME280Sensor {
public:
    BME280Sensor(i2c_port_t i2c_port, uint8_t i2c_addr);
    ~BME280Sensor();

    bool init();
    bool read(float &press, float &altitude);

private:
    i2c_port_t _i2c_port;
    uint8_t _i2c_addr;
    i2c_bus_handle_t _i2c_bus;
    bme280_handle_t _bme280;
};

#endif //SUBSIDENCE_MONITOR_BME280_SENSOR_H
