idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES deng_foc esp_driver_gpio esp_driver_uart lowpass_filter pid current_sense
                       INCLUDE_DIRS ".")
