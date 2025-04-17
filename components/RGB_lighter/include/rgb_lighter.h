//
// Created by badzxb on 25-3-15.
//

#ifndef RGB_LIGHTER_H
#define RGB_LIGHTER_H

#pragma once

#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

class RGBLighter {
public:
    /**
     * @brief 构造函数
     * @param channel 使用的 RMT 通道，默认为 RMT_CHANNEL_0
     * @param gpio 指定输出 GPIO，此处默认 GPIO48
     */
    explicit RGBLighter(rmt_channel_t channel = RMT_CHANNEL_0, gpio_num_t gpio = GPIO_NUM_48);
    ~RGBLighter();

    /**
     * @brief 设置 RGB 灯颜色
     * @param red 红色分量 (0~255)
     * @param green 绿色分量 (0~255)
     * @param blue 蓝色分量 (0~255)
     */
    void setColor(uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief 闪烁指定次数
     * @param times 闪烁次数
     * @param delay_ms 每次闪烁延时（毫秒）
     */
    void blink(int times, int delay_ms);

    /**
    * @brief 指定阈值闪烁以警报
    * @param num 指定阈值
    * @param num_now 当前数值
    */
    void warn_num_int(int num, int num_now);

private:
    rmt_channel_t _channel;
    gpio_num_t _gpio;
    void sendColor(uint8_t red, uint8_t green, uint8_t blue);
};

#endif //RGB_LIGHTER_H
