set(APP ibus_test)
add_executable(${APP} main.c)
target_link_libraries(${APP} ibus)
target_include_directories(${APP} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)