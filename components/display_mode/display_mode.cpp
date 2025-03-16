//
// Created by badzxb on 25-3-14.
//


#include "display_mode.h"
#include <cmath>
#include <cstdio>

DisplayMode::DisplayMode(SSDDisplay &display) : _display(display) {}

void DisplayMode::showFSR(int fsr_raw) {
    _display.clear();
    int cx = 83, cy = 52, r = 50, segments = 40;
    double start_deg = 135.0, end_deg = 45.0;
    double step = (start_deg - end_deg) / segments;
    double prev_x = 0, prev_y = 0;

    for (int i = 0; i <= segments; i++) {
        double angle_deg = start_deg - i * step;
        double angle_rad = angle_deg * M_PI / 180.0;
        int x = cx + (int)(r * cos(angle_rad));
        int y = cy - (int)(r * sin(angle_rad));
        if (i > 0) {
            _display.drawLine((int)prev_x, (int)prev_y, x, y);
        }
        prev_x = x;
        prev_y = y;
    }

    double pointer_angle_deg = 135.0 - ((double)fsr_raw / 4095.0) * 90.0;
    double pointer_angle_rad = pointer_angle_deg * M_PI / 180.0;
    int px = cx + (int)(r * cos(pointer_angle_rad));
    int py = cy - (int)(r * sin(pointer_angle_rad));

    _display.drawLine(cx, cy, px, py);
    char buf[32];
    snprintf(buf, sizeof(buf), "FSR:%d", fsr_raw);
    _display.drawString(0, 32, buf, 16, 1);
    _display.refresh();
}

void DisplayMode::showWaterLevel(int water_raw) {
    _display.clear();
    int width = 128, amplitude = 3;
    static int phase = 0;
    phase = (phase + 5) % 360;
    int offset = 50 - (int)((float)water_raw / 2000.0 * 30);
    int prev_x = 0;
    int prev_y = offset + (int)(amplitude * sin((prev_x + phase) * 2 * M_PI / 64.0));

    for (int x = 1; x < width; x++) {
        int y = offset + (int)(amplitude * sin((x + phase) * 2 * M_PI / 64.0));
        _display.drawLine(prev_x, prev_y, x, y);
        prev_x = x;
        prev_y = y;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "Water:%d", water_raw);
    _display.drawString(0, 0, buf, 16, 1);
    _display.refresh();
}

void DisplayMode::showAltitude(float altitude) {
    _display.clear();
    char buf[32];
    snprintf(buf, sizeof(buf), "pressure: %.1f m", altitude);
    _display.drawString(0, 0, buf, 16, 1);
    _display.refresh();
}


void DisplayMode::switch_mode(int fsr_raw, int water_raw, float altitude, int mode) {
    mode = mode % 3;
    switch (mode) {
        case 0: showFSR(fsr_raw); break;
        case 1: showWaterLevel(water_raw); break;
        case 2: showAltitude(altitude); break;
        default: break;;
    }
}