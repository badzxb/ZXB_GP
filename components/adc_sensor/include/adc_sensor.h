//
// Created by badzxb on 25-3-14.
//
#ifndef ADC_SENSOR_H
#define ADC_SENSOR_H

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <esp_timer.h>

class ADCSensor {
public:
    ADCSensor(adc_unit_t unit, adc1_channel_t channel, adc_atten_t attenuation, adc_bits_width_t width);
    ~ADCSensor();

    int read_raw() const;


private:
    int raw_adc_i_;

    adc_unit_t _unit;
    adc1_channel_t _channel;
    adc_atten_t _attenuation;         //衰减，用于控制ADC的输入电压测量范围
    adc_bits_width_t _width;
    esp_adc_cal_characteristics_t *_adc_chars;

    esp_timer_handle_t scale_timer_{};  // 定时器句柄

    static void _timer_callback_static(void * p);
    void _scale_loop();
    int readRaw() const;
};

#endif // ADC_SENSOR_H
