idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos driver esp_driver_gpio esp_driver_i2c log)
