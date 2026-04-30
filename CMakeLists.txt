cmake_minimum_required(VERSION 3.22)

project(RCJChassisDriver LANGUAGES C ASM)

option(RCJ_BUILD_STM32 "Build the STM32 chassis firmware" ON)

if(NOT DEFINED RCJ_BUILD_HOST_APP)
    if(CMAKE_CROSSCOMPILING)
        set(RCJ_BUILD_HOST_APP OFF)
    else()
        set(RCJ_BUILD_HOST_APP ON)
    endif()
endif()
option(RCJ_BUILD_HOST_APP "Build the Ubuntu host application" ${RCJ_BUILD_HOST_APP})

if(RCJ_BUILD_STM32)
    add_subdirectory(RCJchassisdiver)
endif()

if(RCJ_BUILD_HOST_APP AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/RCJappforchase/CMakeLists.txt")
    add_subdirectory(RCJappforchase)
elseif(RCJ_BUILD_HOST_APP)
    message(STATUS "RCJappforchase has no CMakeLists.txt yet; skipping host app.")
else()
    message(STATUS "Host app build is disabled for this preset.")
endif()
