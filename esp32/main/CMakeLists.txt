idf_component_register(SRCS "main.c" "../user_lib/app.cpp" "../user_lib/sys_time.cpp"
                             "../user_lib/hw/sensor.cpp" "../user_lib/hw/motor.cpp"
                             "../user_lib/hw/leg_servo.cpp"
                             "../user_lib/hw/gamepad.cpp"
                             "../user_lib/hw/wifi.cpp"
                             "../user_lib/io/web.cpp"
                             "../user_lib/controller/control.cpp"
                             "../user_lib/controller/balance.cpp"
                             "../user_lib/test.cpp"
                    INCLUDE_DIRS "." "../user_lib"
                    REQUIRES bt nvs_flash esp_wifi esp_netif esp_event
                             esp_http_server esp_driver_i2c esp_driver_gpio
                             esp_driver_mcpwm esp_driver_uart esp_timer esp_rom)
