idf_component_register(SRCS "bme280_sensor.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "driver" "i2c_bus" "bme280")

