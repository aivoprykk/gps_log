# CMakeLists.txt for GPS Log Test Example
cmake_minimum_required(VERSION 3.16)

# Set extra component directories to find gps_log and its dependencies
set(EXTRA_COMPONENT_DIRS 
    "${CMAKE_CURRENT_LIST_DIR}/../../../gps_log"
    "${CMAKE_CURRENT_LIST_DIR}/../../../logger_common"
    "${CMAKE_CURRENT_LIST_DIR}/../../../logger_config"
    "${CMAKE_CURRENT_LIST_DIR}/../../../sconfig"
    "${CMAKE_CURRENT_LIST_DIR}/../../../strutil"
    "${CMAKE_CURRENT_LIST_DIR}/../../../logger_ubx"
    "${CMAKE_CURRENT_LIST_DIR}/../../../logger_context"
    "${CMAKE_CURRENT_LIST_DIR}/../../../logger_vfs"
    "${CMAKE_CURRENT_LIST_DIR}/../../../ccan_json"
)

# Include ESP-IDF CMake utilities
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(gps_log_test)

