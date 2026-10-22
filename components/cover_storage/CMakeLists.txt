idf_component_register(SRCS "cover_storage.c"
                       INCLUDE_DIRS "include"
                       REQUIRES cover_core nvs_flash
                       PRIV_REQUIRES log freertos)
