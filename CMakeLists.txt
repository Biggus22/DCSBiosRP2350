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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD waveshare_rp2040_pizero CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(DCSBiosRP2350 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# add_executable(DCSBiosRP2350 RS485_test_master.cpp)
# add_executable(DCSBiosRP2350 RS485_test_slave.cpp)
# add_executable(DCSBiosRP2350 Testing_sketch.cpp)
# add_executable(DCSBiosRP2350 ADS1115_example.cpp)
# add_executable(DCSBiosRP2350 PCF8575_example.cpp)
add_executable(DCSBiosRP2350 RP2040_Zero_onboard_RGB_LED_example.cpp)
# add_executable(DCSBiosRP2350 MCP23S17_example.cpp)
# add_executable(DCSBiosRP2350 DCSBiosRP2350.cpp)
# add_executable(DCSBiosRP2350 Fox1_Panel1.cpp)
# add_executable(DCSBiosRP2350 Fox2_Panel1.cpp)



pico_set_program_name(DCSBiosRP2350 "DCSBiosRP2350")
pico_set_program_version(DCSBiosRP2350 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(DCSBiosRP2350 0)
pico_enable_stdio_usb(DCSBiosRP2350 1)

# Compile ws2812.pio into ws2812.pio.h
pico_generate_pio_header(DCSBiosRP2350 ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/ws2812.pio
    ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/i2c.pio
)

# Add the standard library to the build
target_link_libraries(DCSBiosRP2350
        pico_stdlib
        pico_multicore
        pico_time
        hardware_adc
        hardware_i2c
        hardware_pio
        hardware_clocks
        hardware_pwm
        hardware_watchdog
        hardware_spi
        )

# Add the standard include files to the build
target_include_directories(DCSBiosRP2350 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
)
target_sources(DCSBiosRP2350 PRIVATE
    src/internal/Protocol.cpp
    src/internal/aw9523b.cpp
    src/internal/ws2812.cpp
    src/internal/ht16k33a.cpp
    src/internal/Display.cpp
    src/internal/pio_i2c.cpp
    src/internal/heartbeat.cpp
    src/internal/DeviceAddress.cpp
    src/internal/BoardMode.cpp
    src/internal/rs485.cpp
    src/internal/core1_tasks.cpp
    src/internal/Mcp23s17.cpp
    src/internal/PCF8575.cpp

    )
pico_add_extra_outputs(DCSBiosRP2350)

