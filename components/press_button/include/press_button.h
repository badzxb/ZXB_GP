//
// Created by badzxb on 25-3-14.
//

#ifndef SUBSIDENCE_MONITOR_PRESS_BUTTON_H
#define SUBSIDENCE_MONITOR_PRESS_BUTTON_H

#pragma once
#include "driver/gpio.h"


class ButtonHandler {
public:
    explicit ButtonHandler(gpio_num_t button_gpio);
    bool isPressed();

private:
    gpio_num_t _button_gpio;
    uint8_t _last_state;
};

#endif //SUBSIDENCE_MONITOR_PRESS_BUTTON_H
