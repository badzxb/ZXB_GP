//
// Created by badzxb on 25-3-15.
//

#include "rgb_lighter.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include <cstring>
#include "project_conf.h"

static const char *TAG = "RGBLighter";


// 将单个位转为 RMT 项
static inline void ws2812_build_item(rmt_item32_t* item, bool bit_val) {
    if (bit_val) {
        // 1: 高 T1H_TICKS, 低 T1L_TICKS
        item->duration0 = T1H_TICKS;
        item->level0 = 1;
        item->duration1 = T1L_TICKS;
        item->level1 = 0;
    } else {
        // 0: 高 T0H_TICKS, 低 T0L_TICKS
        item->duration0 = T0H_TICKS;
        item->level0 = 1;
        item->duration1 = T0L_TICKS;
        item->level1 = 0;
    }
}

RGBLighter::RGBLighter(rmt_channel_t channel, gpio_num_t gpio)
    : _channel(channel), _gpio(gpio)
{
    // 配置 RMT 用于 WS2812 传输
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = _channel;
    config.gpio_num = _gpio;
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.clk_div = RMT_CLK_DIV;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(_channel, 0, 0));
}

RGBLighter::~RGBLighter() {
    rmt_driver_uninstall(_channel);
}

void RGBLighter::sendColor(uint8_t red, uint8_t green, uint8_t blue) {
    // WS2812 的数据顺序为 GRB
    uint8_t colors[3] = { green, red, blue };
    rmt_item32_t items[LED_RMT_ITEMS];
    int item_index = 0;
    for (int c = 0; c < 3; c++) {
        for (int bit = 7; bit >= 0; bit--) {
            bool bit_val = (colors[c] >> bit) & 0x01;
            ws2812_build_item(&items[item_index], bit_val);
            item_index++;
        }
    }
    ESP_ERROR_CHECK(rmt_write_items(_channel, items, LED_RMT_ITEMS, true));
    rmt_wait_tx_done(_channel, portMAX_DELAY);
}

void RGBLighter::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    sendColor(red, green, blue);
}

void RGBLighter::blink(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        // 打开 LED（例如使用白光）
        setColor(255, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        // 熄灭 LED
        setColor(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void RGBLighter::warn_num_int(int num, int num_now) {
    if (num_now > num) {
        blink(1,500);
        num_now = 0;
    }

}