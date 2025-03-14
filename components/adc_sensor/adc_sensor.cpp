//
// Created by badzxb on 25-3-14.
//
#include "adc_sensor.h"
#include "esp_log.h"

#define DEFAULT_VREF 1100
#define TAG "ADCSensor"

ADCSensor::ADCSensor(adc_unit_t unit, adc1_channel_t channel, adc_atten_t attenuation, adc_bits_width_t width)
    : _unit(unit), _channel(channel), _attenuation(attenuation), _width(width), _adc_chars(nullptr) {
    ESP_LOGI(TAG, "Initializing ADC...");

    // 配置 ADC 宽度、通道衰减
    if (_unit == ADC_UNIT_1) {
        adc1_config_width(_width);
        adc1_config_channel_atten(_channel, _attenuation);
    } else {
        ESP_LOGE(TAG, "Unsupported ADC unit: %d", _unit);
    }

    // 分配并初始化校准特性
    _adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(_unit, _attenuation, _width, DEFAULT_VREF, _adc_chars);

    // 创建并启动定时器, 定期读取重量
    const esp_timer_create_args_t timer_args = {
        .callback = &ADCSensor::_timer_callback_static,
        .arg = this,
        .name = "scale_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &scale_timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(scale_timer_, 100000)); // 100 ms 读取一次
}

ADCSensor::~ADCSensor() {
    if (_adc_chars) {
        free(_adc_chars);
    }
}

int ADCSensor::read_raw() const {
    return raw_adc_i_;
}


void ADCSensor::_timer_callback_static(void *args) {
    auto *self = static_cast<ADCSensor *>(args);
    self->_scale_loop();
}

void ADCSensor::_scale_loop() {
    raw_adc_i_ = readRaw();
}

int ADCSensor::readRaw() const {
    if (_unit == ADC_UNIT_1) {
        return adc1_get_raw(_channel);
    }
    ESP_LOGE(TAG, "Invalid ADC unit");
    return 0;
}
