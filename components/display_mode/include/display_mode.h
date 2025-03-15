//
// Created by badzxb on 25-3-14.
//

#ifndef SUBSIDENCE_MONITOR_DISPLAY_MODE_H
#define SUBSIDENCE_MONITOR_DISPLAY_MODE_H

#pragma once

#include "ssd_display.h"

class DisplayMode {
public:
    DisplayMode(SSDDisplay &display);
    void showFSR(int fsr_raw);
    void showWaterLevel(int water_raw);
    void showAltitude(float altitude);
    void switch_mode(int fsr_raw, int water_raw, float altitude, int mode);

private:
    SSDDisplay &_display;
};

#endif //SUBSIDENCE_MONITOR_DISPLAY_MODE_H
