idf_component_register(SRCS "main.cpp" "cp210x_usb.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb esp_driver_gpio esp_driver_uart
                    )