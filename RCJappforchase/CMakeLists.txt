cmake_minimum_required(VERSION 3.22)

project(RCJappforchase LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Widgets SerialPort QUIET)
if(NOT Qt6_FOUND)
    message(FATAL_ERROR
        "Qt6 development packages are required for RCJappforchase. "
        "On Ubuntu install: sudo apt install qt6-base-dev qt6-serialport-dev")
endif()

add_executable(rcj_appforchase
    src/main_window.cpp
    src/main_window.h
    src/map_widget.cpp
    src/map_widget.h
    src/main.cpp
    src/path_planner.cpp
    src/path_planner.h
    src/protocol.cpp
    src/protocol.h
    src/serial_client.cpp
    src/serial_client.h
)

target_compile_features(rcj_appforchase PRIVATE cxx_std_17)
target_compile_options(rcj_appforchase PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_compile_definitions(rcj_appforchase PRIVATE
    RCJ_APP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(rcj_appforchase PRIVATE
    Qt6::Widgets
    Qt6::SerialPort
)
