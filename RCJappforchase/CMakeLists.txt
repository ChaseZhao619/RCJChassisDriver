cmake_minimum_required(VERSION 3.22)

project(RCJappforchase LANGUAGES CXX)

add_executable(rcj_appforchase
    src/main.cpp
)

target_compile_features(rcj_appforchase PRIVATE cxx_std_17)
target_compile_options(rcj_appforchase PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
