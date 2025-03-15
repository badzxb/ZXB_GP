//
// Created by badzxb on 25-3-14.
//

#ifndef SUBSIDENCE_MONITOR_SSD_DISPLAY_H
#define SUBSIDENCE_MONITOR_SSD_DISPLAY_H

#pragma once

#include "ssd1306.h"

class SSDDisplay {
public:
    SSDDisplay(i2c_port_t i2c_port, uint8_t i2c_addr);
    ~SSDDisplay();

    bool init();
    void clear();
    void drawString(int x, int y, const char *text, int size, int color);
    void drawLine(int x1, int y1, int x2, int y2);
    void refresh();

private:
    i2c_port_t _i2c_port;
    uint8_t _i2c_addr;
    ssd1306_handle_t _oled_dev;
};

#endif //SUBSIDENCE_MONITOR_SSD_DISPLAY_H
