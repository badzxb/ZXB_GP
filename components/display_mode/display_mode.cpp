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

    for (int i = 0; i <= segments; i++) {            //弧线分为40段进行绘制，step为每段跨过的角度，我们需要得到绘制的一小段线相对于圆心的位置
        double angle_deg = start_deg - i * step;
        double angle_rad = angle_deg * M_PI / 180.0; //得出当前度数对应的弧度
        int x = cx + (int)(r * cos(angle_rad));      //从右向左画，每次对线段终点作改变
        int y = cy - (int)(r * sin(angle_rad));
        if (i > 0) {
            _display.drawLine((int)prev_x, (int)prev_y, x, y);
        }
        prev_x = x;
        prev_y = y;
    }

    double pointer_angle_deg = 135.0 - ((double)fsr_raw / 4095) * 90.0;    //根据圆心和fsr_raw的数值得出中间指针的起点和终点
    double pointer_angle_rad = pointer_angle_deg * M_PI / 180.0;
    int px = cx + (int)(r * cos(pointer_angle_rad));
    int py = cy - (int)(r * sin(pointer_angle_rad));
    _display.drawLine(cx, cy, px, py);

    // char buf[32];
    // snprintf(buf, sizeof(buf), "Prs:%.1f kg", ((float)fsr_raw/ 20475.0 ) * 15);
    // _display.drawString(0, 32, buf, 16, 1);
    // _display.refresh();
    /* ---------- 输出内容：按标定表做分段线性插值 ---------- */
    static const float    weight_kg[] = {0.0f, 0.6f, 1.0f, 1.5f, 1.9f, 2.35f, 2.7f,
                                         4.8f, 6.3f, 7.3f, 7.9f, 9.0f, 10.0f, 11.0f,
                                         12.0f, 13.0f, 14.0f, 15.0f};
    static const uint16_t raw_val[]   = {   0,   200,  530, 1100, 2000, 2459, 2650,
                                         3500, 3800, 3850, 3900, 3950, 4000, 4030,
                                         4050, 4065, 4080, 4095};
    constexpr int CAL_POINTS = sizeof(raw_val) / sizeof(raw_val[0]);

    auto raw2kg = [&](uint16_t raw) -> float {
        if (raw <= raw_val[0])              return weight_kg[0];
        if (raw >= raw_val[CAL_POINTS - 1]) return weight_kg[CAL_POINTS - 1];

        for (int i = 1; i < CAL_POINTS; ++i) {
            if (raw < raw_val[i]) {
                float ratio = static_cast<float>(raw - raw_val[i - 1]) /
                              static_cast<float>(raw_val[i] - raw_val[i - 1]);
                return weight_kg[i - 1] + ratio * (weight_kg[i] - weight_kg[i - 1]);
            }
        }
        return 0.0f;   // 理论上不会到这里
    };

    float weight = raw2kg(static_cast<uint16_t>(fsr_raw));

    char buf[32];
    snprintf(buf, sizeof(buf), "Prs:%.2f kg", weight);   // 保留两位小数
    _display.drawString(0, 32, buf, 16, 1);
    _display.refresh();

}

void DisplayMode::showWaterLevel(int water_raw) {
    /* --- 标定表：水位(cm) ↔ ADC 原始值 --- */
    static const float    level_cm[] = {0.0f, 0.1f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 5.5f, 6.0f};
    static const uint16_t raw_val [] = {0, 300, 1000, 1300, 1700, 1800, 1870, 1910, 1950, 1970};
    constexpr int CAL_POINTS = sizeof(raw_val) / sizeof(raw_val[0]);

    /* 把 water_raw 转成 cm 的小函数（分段线性插值） */
    auto raw2cm = [&](uint16_t raw) -> float {
        if (raw <= raw_val[0])               return level_cm[0];
        if (raw >= raw_val[CAL_POINTS - 1])  return level_cm[CAL_POINTS - 1];

        for (int i = 1; i < CAL_POINTS; ++i) {
            if (raw < raw_val[i]) {
                float ratio = (float)(raw - raw_val[i - 1]) /
                              (float)(raw_val[i] - raw_val[i - 1]);
                return level_cm[i - 1] + ratio * (level_cm[i] - level_cm[i - 1]);
            }
        }
        return 0.0f;   // 不会到这里
    };

    float water_cm = raw2cm(water_raw);
    _display.clear();
    int width = 128, amplitude = 3;
    static int phase = 0;
    phase = (phase + 5) % 360;      //x对于2pi的初始偏移，实现水波纹动态效果
    int offset = 50 - (int)((float)water_raw / 2000.0 * 30);   //默认高度偏移量
    int prev_x = 0;
    int prev_y = offset + (int)(amplitude * sin((prev_x + phase) * 2 * M_PI / 64.0));   //每64个像素为一个2pi的图像周期

    for (int x = 1; x < width; x++) {
        int y = offset + (int)(amplitude * sin((x + phase) * 2 * M_PI / 64.0));       //随着x增加高度变化
        _display.drawLine(prev_x, prev_y, x, y);
        prev_x = x;
        prev_y = y;
    }

    char buf[32];
    // snprintf(buf, sizeof(buf), "Water:%.1f cm", (float)water_raw / 2300 * 4);
    snprintf(buf, sizeof(buf), "Water:%.1f cm", water_cm);
    _display.drawString(0, 0, buf, 16, 1);
    _display.refresh();
}

void DisplayMode::showAltitude(float press) {
    _display.clear();
    char buf[32];
    snprintf(buf, sizeof(buf), "air_pressure: %.1f pa", press);
    _display.drawString(0, 0, buf, 16, 1);
    _display.refresh();
}

void DisplayMode::showDefault() {
    _display.clear();
    char buf[64];
    snprintf(buf, sizeof(buf), "land subsidence monitor press button to start");
    _display.drawString(0, 12, buf, 12, 1);
    _display.refresh();
}



void DisplayMode::switch_mode(int fsr_raw, int water_raw, float altitude, int mode) {
    switch (mode) {
        case -1: showDefault();break;
        case 0: showFSR(fsr_raw); break;
        case 1: showWaterLevel(water_raw); break;
        case 2: showAltitude(altitude); break;
        default: break;;
    }
}