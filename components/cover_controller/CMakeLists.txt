idf_component_register(SRCS "cover_controller.c"
                       INCLUDE_DIRS "include"
                       REQUIRES cover_core cover_hal
                       PRIV_REQUIRES cover_storage esp_timer log freertos)
