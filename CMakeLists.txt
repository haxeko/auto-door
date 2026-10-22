# Auto-door firmware (curtain / blind profile) - ESP-IDF v5.5.5 + ESP-Matter release/v1.5
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{ESP_MATTER_PATH})
    message(FATAL_ERROR "Please set ESP_MATTER_PATH to the esp-matter repository (source esp-matter/export.sh)")
endif()

set(PROJECT_VER "0.1.0")
set(PROJECT_VER_NUMBER 1)

set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Loads sdkconfig.defaults (+ sdkconfig.defaults.<target>) the same way as the esp-matter examples.
include(${ESP_MATTER_PATH}/examples/common/cmake_common/components_include.cmake)

# No esp-matter device_hal: this project has its own GPIO layer (components/cover_hal).
set(EXTRA_COMPONENT_DIRS
    "${MATTER_SDK_PATH}/config/esp32/components"
    "${ESP_MATTER_PATH}/components")

project(auto_door)

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
# For RISC-V chips, project_include.cmake sets -Wno-format but does not clear flags that depend on -Wformat
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)
