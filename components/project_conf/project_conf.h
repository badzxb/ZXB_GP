#ifndef FOCKNOB_PROJECT_CONF_H
#define FOCKNOB_PROJECT_CONF_H


#define IIC_MASTER_NUM                  I2C_NUM_0         // IIC port number for master
#define IIC_MASTER_FREQ_HZ              100000            // IIC master clock frequency
#define IIC_MASTER_SDA_IO               GPIO_NUM_15
#define IIC_MASTER_SCL_IO               GPIO_NUM_16

#define ADC_UNIT                        ADC_UNIT_1
#define FSR_SENSOR_ADC_CHANNEL          ADC1_CHANNEL_0
#define WATER_SENSOR_ADC_CHANNEL        ADC1_CHANNEL_1

#define WIFI_SSID                       "AirPort-5G"
#define WIFI_PASSWORD                   "###732###"
#define MQTT_URI                        "mqtt://home.kivvi.me:1883"
#define MQTT_USER                       "badzxb"
#define MQTT_PASS                       "badzxb"
#define MQTT_CLIENT_ID                  "esp32s3"

#endif //FOCKNOB_PROJECT_CONF_H
