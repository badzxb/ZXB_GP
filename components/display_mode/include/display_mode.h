//
// Created by badzxb on 25-3-14.
//

#ifndef SUBSIDENCE_MONITOR_DISPLAY_MODE_H
#define SUBSIDENCE_MONITOR_DISPLAY_MODE_H

#pragma once

#include "ssd_display.h"

class DisplayMode {
public:
    DisplayMode(SSDDisplay &display);        //根据模式选择显示的界面
    void showFSR(int fsr_raw);
    void showWaterLevel(int water_raw);
    void showAltitude(float altitude);
    void showDefault();
    void switch_mode(int fsr_raw, int water_raw, float altitude, int mode);

private:
    SSDDisplay &_display;
};

#endif //SUBSIDENCE_MONITOR_DISPLAY_MODE_H
