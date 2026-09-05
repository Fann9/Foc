idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES deng_foc esp_driver_gpio lowpass_filter pid current_sense
                       INCLUDE_DIRS ".")
