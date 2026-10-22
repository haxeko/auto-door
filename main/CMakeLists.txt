idf_component_register(SRCS "app_main.cpp"
                            "app_config.cpp"
                            "app_matter_cover.cpp"
                            "app_console.cpp"
                       PRIV_INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_matter esp_matter_console nvs_flash
                                     cover_core cover_hal cover_controller cover_storage)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
