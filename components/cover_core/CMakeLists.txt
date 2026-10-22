# Pure C motion logic. Built as an ESP-IDF component in the firmware and as a
# plain static library by the host unit tests (test/host).
if(ESP_PLATFORM)
    idf_component_register(SRCS "cover_sm.c"
                           INCLUDE_DIRS "include")
else()
    add_library(cover_core STATIC cover_sm.c)
    target_include_directories(cover_core PUBLIC include)
endif()
