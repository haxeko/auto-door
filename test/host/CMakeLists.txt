cmake_minimum_required(VERSION 3.16)
project(cover_host_tests C)

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra -Werror -fsanitize=address,undefined)
add_link_options(-fsanitize=address,undefined)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../components/cover_core cover_core)

add_executable(test_cover_sm test_cover_sm.c)
target_link_libraries(test_cover_sm cover_core)

enable_testing()
add_test(NAME cover_sm COMMAND test_cover_sm)
