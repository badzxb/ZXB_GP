#ifndef FOCKNOB_PROJECT_CONF_H
#define FOCKNOB_PROJECT_CONF_H


#define IIC_BME_NUM                  I2C_NUM_0         // IIC port number for master
#define IIC_BME_FREQ_HZ              100000            // IIC master clock frequency
#define IIC_BME_SDA_IO               21
#define IIC_BME_SCL_IO               20
#define IIC_BME_I2C_ADDR             0x76

#define IIC_SSD1306_NUM                  I2C_NUM_1         // IIC port number for master
#define IIC_SSD1306_FREQ_HZ              100000            // IIC master clock frequency
#define IIC_SSD1306_SDA_IO               17
#define IIC_SSD1306_SCL_IO               18
#define IIC_SSD1306_I2C_ADDR             0x3c

#define ADC_UNIT                         ADC_UNIT_1
#define FSR_SENSOR_ADC_CHANNEL           ADC1_CHANNEL_3
#define WATER_SENSOR_ADC_CHANNEL         ADC1_CHANNEL_4

#define SWITCH_BUTTON                    GPIO_NUM_19


#define RMT_CLK_DIV                      2            // WS2812 的 RMT 定时参数，单位为 RMT 时钟 tick
                                                      //（假设 APB 时钟 80MHz，RMT_CLK_DIV = 2 => tick ≈ 25 ns）
#define T0H_TICKS                        14           // 0：高电平 350 ns，低电平 850 ns（约为 14 和 34 个 tick）
#define T0L_TICKS                        34           // 1：高电平 700 ns，低电平 500 ns（约为 28 和 20 个 tick）
#define T1H_TICKS                        28
#define T1L_TICKS                        20

// WS2812 每个 LED 需要 24 个 RMT 项（GRB 顺序）
#define LED_RMT_ITEMS 24


#define WIFI_SSID                       "AirPort-5G"
#define WIFI_PASSWORD                   "###732###"
#define MQTT_URI                        "mqtt://home.kivvi.me:1883"
#define MQTT_USER                       "badzxb"
#define MQTT_PASS                       "badzxb"
#define MQTT_CLIENT_ID                  "esp32s3"

#endif //FOCKNOB_PROJECT_CONF_H
