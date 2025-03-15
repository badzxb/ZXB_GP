//
// Created by badzxb on 25-3-14.
//

#include "ssd_display.h"
#include "esp_log.h"

#define TAG "SSDDisplay"

SSDDisplay::SSDDisplay(i2c_port_t i2c_port, uint8_t i2c_addr)
        : _i2c_port(i2c_port), _i2c_addr(i2c_addr), _oled_dev(nullptr) {}

SSDDisplay::~SSDDisplay() {
    if (_oled_dev) {
        ssd1306_delete(_oled_dev);
    }
}

bool SSDDisplay::init() {
    ESP_LOGI(TAG, "Initializing SSD1306 display...");

    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 17,
            .scl_io_num = 18,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                    .clk_speed = 100000
            }
    };

    i2c_param_config(_i2c_port, &conf);
    i2c_driver_install(_i2c_port, I2C_MODE_MASTER, 0, 0, 0);

    _oled_dev = ssd1306_create(_i2c_port, _i2c_addr);
    return ssd1306_init(_oled_dev) == ESP_OK;
}

void SSDDisplay::clear() {
    ssd1306_clear_screen(_oled_dev, 0x00);
}

void SSDDisplay::drawString(int x, int y, const char *text, int size, int color) {
    ssd1306_draw_string(_oled_dev, x, y, (uint8_t *)text, size, color);
}

void SSDDisplay::drawLine(int x1, int y1, int x2, int y2) {
    ssd1306_draw_line(_oled_dev, x1, y1, x2, y2);
}

void SSDDisplay::refresh() {
    ssd1306_refresh_gram(_oled_dev);
}
