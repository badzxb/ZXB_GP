//
// Created by badzxb on 25-3-14.
//


#include "press_button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


ButtonHandler::ButtonHandler(gpio_num_t button_gpio) : _button_gpio(button_gpio), _last_state(1) {
    gpio_config_t btn_conf = {
            .pin_bit_mask = (1ULL << _button_gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);
}

bool ButtonHandler::isPressed() {
    uint8_t current_state = gpio_get_level(_button_gpio);
    static int mode = 0;
    if (_last_state == 1 && current_state == 0) {  // 检测下降沿（按下）
        vTaskDelay(pdMS_TO_TICKS(20));  // 消抖
        while (gpio_get_level(_button_gpio) == 0) {  // 等待释放
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        _last_state = 1;

        return true;
    }
    _last_state = current_state;
    return false;
}