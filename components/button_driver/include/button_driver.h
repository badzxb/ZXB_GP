//
// Created by HAIRONG ZHU on 25-3-11.
//

#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H



#include <functional>
#include "driver/gpio.h"
#include "esp_timer.h"

/**
 * @brief 按钮回调函数类型, 无参数无返回值
 */
using ButtonCallback = std::function<void(void)>;

class ButtonDriver {
public:
    /**
     * @brief 构造函数
     * @param gpio_num          按钮所使用的 GPIO 编号
     * @param active_level      为true表示高电平有效, 为false表示低电平有效(常见硬件上拉, 按下拉低)
     * @param debounce_time_ms  消抖时间 (毫秒) , 当检测到变化时只有在该时间内状态保持一致才判定为稳定
     * @param poll_interval_ms  定时器轮询间隔 (毫秒) 
     */
    ButtonDriver(gpio_num_t gpio_num, 
                 bool active_level = true,
                 uint32_t debounce_time_ms = 50,
                 uint32_t poll_interval_ms = 10);

    /**
     * @brief 析构函数, 销毁前停止并删除定时器
     */
    ~ButtonDriver();

    /**
     * @brief 设置按下按键时的回调函数
     */
    void setOnPressCallback(ButtonCallback cb);

    /**
     * @brief 设置松开按键时的回调函数
     */
    void setOnReleaseCallback(ButtonCallback cb);

private:
    gpio_num_t gpio_num_;             ///< 按钮GPIO引脚 
    bool active_level_;               ///< 是否高电平有效(默认false: 低电平有效)
    uint32_t debounce_time_ms_;       ///< 消抖时间 
    uint32_t poll_interval_ms_;       ///< 轮询间隔 
    bool last_stable_state_;          ///< 上一次稳定状态 (true=按下, false=松开)
    int debounce_counter_;            ///< 消抖计数器 (单位: ms)

    esp_timer_handle_t timer_handle_; ///< esp_timer 句柄

    ButtonCallback onPress_;          ///< 按下时回调
    ButtonCallback onRelease_;        ///< 松开时回调

    /**
     * @brief 静态定时器回调函数, 内部会调用成员函数 process()
     */
    static void timerCallback(void* arg);

    /**
     * @brief 实际的处理函数, 用来轮询按钮状态 、 消抖并触发回调
     */
    void process();
};


#endif //BUTTON_DRIVER_H
