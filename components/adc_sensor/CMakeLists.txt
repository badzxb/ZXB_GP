idf_component_register(SRCS "adc_sensor.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "driver" "esp_adc" "esp_timer"
)