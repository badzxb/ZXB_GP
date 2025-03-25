#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "project_conf.h"
#include "wifi_mqtt_driver.h"
#include "adc_sensor.h"
#include "bme280_sensor.h"
#include "ssd_display.h"
#include "display_mode.h"
#include "press_button.h"
#include "rgb_lighter.h"
#include "ArduinoJson.hpp"
#include "button_driver.h"

using namespace ArduinoJson;

void activity_monitor(void *arg) {
    /*
     * @brief 任务统计信息
     *
     * 需要打开以下参数在 idf.py menuconfig 中
     * configTASKLIST_INCLUDE_COREID
     * CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS          // 生成运行时间统计
     * CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS   // 使用格式化函数
     * CONFIG_FREERTOS_USE_TRACE_FACILITY               // 使用追踪功能
     */

    // 分配足够大的缓冲区来存储任务统计信息
    char *task_stats_buffer = static_cast<char *>(malloc(4096));
    if (task_stats_buffer == nullptr) {
        printf("无法分配内存来存储任务统计信息\n");
        return;
    }

    while (true) {
        // vTaskList(task_stats_buffer);   // 可以查看任务绑定在哪个CPU上了
        // vTaskGetRunTimeStats(task_stats_buffer); // 可以看CPU占用率
        // printf("任务名\t\t运行时间\t\tCPU使用率 (%%) \n");
        // printf("%s\n", task_stats_buffer);

        size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
        size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);

        ESP_LOGI("MEM_CHECK", "Free Internal RAM : %d KB", free_internal / 1024);
        ESP_LOGI("MEM_CHECK", "Free SPIRAM       : %d KB", free_spiram / 1024);
        ESP_LOGI("MEM_CHECK", "Free DMA-capable  : %d KB", free_dma / 1024);
        ESP_LOGI("MEM_CHECK", "Free 8Bit RAM     : %d KB\n", free_8bit / 1024);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// void main_thread(void *args) {
//     // BME280Sensor bme280(IIC_BME_NUM, IIC_BME_I2C_ADDR);
//     // SSDDisplay ssd1306(IIC_SSD1306_NUM, IIC_SSD1306_I2C_ADDR);
//     // DisplayMode displayMode(ssd1306);
//     // ButtonHandler button(GPIO_NUM_19);
//     // ssd1306.init();
// }

extern "C" void app_main() {
    int mode = 0;
    static WifiMqttDriver wifiMqtt(WIFI_SSID, WIFI_PASSWORD, MQTT_URI, MQTT_USER, MQTT_PASS, MQTT_CLIENT_ID);
    auto *fsrSensor = new ADCSensor(ADC_UNIT, FSR_SENSOR_ADC_CHANNEL, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12);
    auto *waterSensor = new ADCSensor(ADC_UNIT, WATER_SENSOR_ADC_CHANNEL, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12);

    BME280Sensor bme280(IIC_BME_NUM, IIC_BME_I2C_ADDR);
    SSDDisplay ssd1306(IIC_SSD1306_NUM, IIC_SSD1306_I2C_ADDR);
    DisplayMode displayMode(ssd1306);
    // ButtonHandler button(GPIO_NUM_19);
    ButtonDriver button(GPIO_NUM_19);
    RGBLighter led(RMT_CHANNEL_0, GPIO_NUM_48);
    ssd1306.init();
    bme280.init();

    wifiMqtt.connect();
    std::string mqtt_json_string;
    button.setOnPressCallback([&mode, &led]() {
        printf("切换到模式 %d\n", mode);
        led.blink(1, 100);
        mode = (mode + 1) % 3;
    });

    while (true) {
        float altitude, pressure;
        int fsr_raw = fsrSensor->read_raw(), water_raw = waterSensor->read_raw();
        bme280.read(pressure, altitude);
        // if (button.isPressed()) {
        //     printf("切换到模式 %d\n", mode);
        //     led.blink(1, 100);
        //     mode = (mode + 1) % 3;
        // }
        displayMode.switch_mode(fsr_raw, water_raw, altitude, mode);
        auto json = JsonDocument();
        json["method"] = "control";
        json["clientToken"] = MQTT_CLIENT_ID;
        json["params"]["fsrSensor"] = fsr_raw;
        json["params"]["waterSensor"] = water_raw;
        json["params"]["BmeSensor_alt"] = altitude;
        json["params"]["BmeSensor_prs"] = pressure;
        serializeJson(json, mqtt_json_string);

        wifiMqtt.publishJson("/zxb/esp32", mqtt_json_string, 0, false);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
