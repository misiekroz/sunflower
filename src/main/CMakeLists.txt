idf_component_register(SRCS "main.cpp"
                            "config/config_main.c"
                            "safety/safety_controller.c"
                            "state/state_controller.c"
                            "state/led_indicator/led_indicator.c"
                            "motion/motor_controller.cpp"
                            "tracking_controller/tracking_controller.cpp"
                            "serial/serial_service.cpp"
                            "sdcard/sd_card_manager.cpp"
                            "wifi/wifi_connect.c"
                            "analog_read/analog_read_service.cpp"
                            "fuzzy_controller/codegen_simulink.cpp"
                            "fuzzy_controller/codegen_simulink_data.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    REQUIRES esp_timer
                    REQUIRES fatfs
                    REQUIRES esp_wifi 
                    REQUIRES nvs_flash
                    PRIV_REQUIRES esp_adc
                    )

