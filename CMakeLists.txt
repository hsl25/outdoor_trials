# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(outdoor_trials C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(outdoor_trials 
               outdoor_trials.c 
               driving.cpp
               buffer.hpp
               vl53l0x/core/src/vl53l0x_api_calibration.c
               vl53l0x/core/src/vl53l0x_api_core.c
               vl53l0x/core/src/vl53l0x_api_ranging.c
               vl53l0x/core/src/vl53l0x_api_strings.c
               vl53l0x/core/src/vl53l0x_api.c
               vl53l0x/platform/src/vl53l0x_platform.c
               )

pico_set_program_name(outdoor_trials "outdoor_trials")
pico_set_program_version(outdoor_trials "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(outdoor_trials 0)
pico_enable_stdio_usb(outdoor_trials 0)

# Add the standard library to the build
target_link_libraries(outdoor_trials
        pico_stdlib
        hardware_pwm
        pico_time
        )

# Add the standard include files to the build
target_include_directories(outdoor_trials PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(outdoor_trials 
        
        )

pico_add_extra_outputs(outdoor_trials)

