idf_component_register(SRCS "main.c" "../user_lib/app.cpp" "../user_lib/sys_time.cpp"
                             "../user_lib/hw/sensor.cpp" "../user_lib/hw/motor.cpp"
                             "../user_lib/hw/leg_servo.cpp"
                             "../user_lib/controller/control.cpp"
                             "../user_lib/controller/balance.cpp"
                             "../user_lib/test.cpp"
                    INCLUDE_DIRS "." "../user_lib")
