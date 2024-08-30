# Copyright 2024 Zorxx Software. All rights reserved.
if(IDF_TARGET)
    idf_component_register(SRCS "lib/ibus.c" "lib/sys/esp-idf.c"
                           INCLUDE_DIRS "include"
                           PRIV_INCLUDE_DIRS "lib" "include/ibus"
                           PRIV_REQUIRES "driver" "esp_timer")
    return()
endif()

cmake_minimum_required(VERSION 3.5)
set(project ibus)
project(${project} LANGUAGES C VERSION 1.0.0)

add_library(ibus STATIC lib/ibus.c lib/sys/linux.c)
target_compile_definitions(ibus PRIVATE SYS_DEBUG_ENABLE=1)
target_include_directories(ibus PUBLIC include)
target_include_directories(ibus PRIVATE lib include/ibus)
install(TARGETS ibus LIBRARY DESTINATION lib)
install(DIRECTORY include/ibus DESTINATION include)

add_subdirectory(examples/linux)
