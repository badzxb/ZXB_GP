//
// Created by BADZXB on 25-3-11.
//

#include "button_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ButtonDriver";

ButtonDriver::ButtonDriver(gpio_num_t gpio_num,
                           bool active_level,
                           uint32_t debounce_time_ms,
                           uint32_t poll_interval_ms)
    : gpio_num_(gpio_num),
      active_level_(active_level),
      debounce_time_ms_(debounce_time_ms),
      poll_interval_ms_(poll_interval_ms),
      last_stable_state_(false),
      debounce_counter_(0),
      onPress_(nullptr),
      onRelease_(nullptr) {
    // 配置GPIO为输入模式, 可以根据active_level选择内部上拉或下拉
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // 不使用中断
    io_conf.mode = GPIO_MODE_INPUT; // 输入模式
    io_conf.pin_bit_mask = (1ULL << gpio_num_); // 要配置的GPIO位掩码
    // 如果默认是低电平有效(常用场景), 则打开内部上拉, 否则可打开内部下拉
    io_conf.pull_down_en = (active_level_ ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE);
    io_conf.pull_up_en = (active_level_ ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE);
    gpio_config(&io_conf);

    // 初始化按钮当前稳定状态
    bool current_read = (gpio_get_level(gpio_num_) != 0);
    last_stable_state_ = (current_read == active_level_);

    // 创建并启动定时器进行轮询
    esp_timer_create_args_t timer_conf = {
        .callback = &ButtonDriver::timerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK, // 在esp_timer专用任务中回调
        .name = "button_poll_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_conf, &timer_handle_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle_, poll_interval_ms_ * 1000)); // ms -> us
}

ButtonDriver::~ButtonDriver() {
    // 停止并删除定时器
    esp_timer_stop(timer_handle_);
    esp_timer_delete(timer_handle_);
}

void ButtonDriver::setOnPressCallback(ButtonCallback cb) {
    onPress_ = cb;
}

void ButtonDriver::setOnReleaseCallback(ButtonCallback cb) {
    onRelease_ = cb;
}

void ButtonDriver::timerCallback(void *arg) {
    auto *btn = static_cast<ButtonDriver *>(arg);
    btn->process();
}

void ButtonDriver::process() {
    bool current_read = (gpio_get_level(gpio_num_) != 0);
    // 根据 active_level_ 判断是否是真正"按下" 
    bool is_pressed = (current_read == active_level_);

    // 如果读到的状态和上一次稳定状态不同, 则说明按钮状态有可能发生了变化, 开始或继续消抖计数
    if (is_pressed != last_stable_state_) {
        debounce_counter_ += poll_interval_ms_;
        if (debounce_counter_ >= debounce_time_ms_) {
            // 状态稳定改变, 则更新 last_stable_state_
            last_stable_state_ = is_pressed;
            // 重置消抖计数
            debounce_counter_ = 0;

            // 触发回调
            if (is_pressed) {
                // 按下
                if (onPress_) {
                    onPress_();
                }
            } else {
                // 松开
                if (onRelease_) {
                    onRelease_();
                }
            }
        }
    } else {
        // 状态未变或又变回去了, 就重置消抖计时
        debounce_counter_ = 0;
    }
}
