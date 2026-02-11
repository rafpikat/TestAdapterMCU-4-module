idf_component_register(SRCS "voiltage_current_measuarement.cpp" "ad770x.cpp" "kalman.cpp" "ADCMeasure.cpp" "uartDriver.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver nvs_flash freertos newlib)
