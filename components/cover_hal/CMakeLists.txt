idf_component_register(SRCS "motor_interlock.c"
                            "motor_driver_gpio2.c"
                            "sim_plant.c"
                            "cover_inputs.c"
                            "pin_guard.c"
                       INCLUDE_DIRS "include"
                       REQUIRES cover_core esp_driver_gpio
                       PRIV_REQUIRES log freertos)
