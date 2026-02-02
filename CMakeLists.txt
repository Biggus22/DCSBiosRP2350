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
set(PICO_BOARD waveshare_rp2350_zero CACHE STRING "Board type")
# Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(DCSBiosRP2350 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# add_executable(DCSBiosRP2350 DCSBiosRP2350.cpp)
# add_executable(DCSBiosRP2350 Fox1_Panel1.cpp)
# add_executable(DCSBiosRP2350 Fox2_Panel1.cpp)
# add_executable(DCSBiosRP2350 Fox2_Demo.cpp)
# add_executable(DCSBiosRP2350 Fox1_Panel_Switch.cpp)

#******  F-4E Examples *********
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_ICS_RP2040-Zero.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/RP2040_Zero_ICS_LED_test.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_AFCS_RP2040-Zero.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_AFCS_RP2040-Zero_MagTest.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_ACFS_RP2040_Pico_testing_light.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_Eng_Start_RP2040_ProMicro.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_EngStart_RP2040_ProMicro_testing_light.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_VORILS_RP2350-Zero.cpp)
# add_executable(DCSBiosRP2350 examples/F-4E/F-4E_Fuel_RP2040-Zero.cpp)
#add_executable(DCSBiosRP2350 examples/F-4E/F-4E_Utility_RP2350-Zero.cpp)
 add_executable(DCSBiosRP2350 examples/F-4E/F-4E_Utility_RP2350-Zero.cpp)
#add_executable(DCSBiosRP2350 examples/F-4E/F-4E_ICS_RP2040-Zero.cpp)

#****** Arduino RS485 Compatibility (Pico Master) *********
# add_executable(DCSBiosRP2350 examples/RS485_Arduino_Master_Pico.cpp)

#****** Test Programs *********
# add_executable(DCSBiosRP2350 test_programs/swarm.cpp)
# add_executable(DCSBiosRP2350 test_programs/LED_test.cpp)
# add_executable(DCSBiosRP2350 test_programs/LED_test_auto.cpp)
# add_executable(DCSBiosRP2350 test_programs/MCP23S17_testing_sketch.cpp)
# add_executable(DCSBiosRP2350 test_programs/ADS1115_example.cpp)
# add_executable(DCSBiosRP2350 test_programs/PCF8575_example.cpp)
# add_executable(DCSBiosRP2350 test_programs/RP2040_Zero_onboard_RGB_LED_example.cpp)
# add_executable(DCSBiosRP2350 test_programs/MCP23S17_example.cpp)
# add_executable(DCSBiosRP2350 test_programs/rs485_arduino_framing_test.cpp)
# add_executable(DCSBiosRP2350 test_programs/X27-168_stepper_RP2350-Zero.cpp)





pico_set_program_name(DCSBiosRP2350 "DCSBiosRP2350")
pico_set_program_version(DCSBiosRP2350 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(DCSBiosRP2350 0)
pico_enable_stdio_usb(DCSBiosRP2350 1)

# Enable Arduino RS485 framing compatibility in this example build
#target_compile_definitions(DCSBiosRP2350 PRIVATE DCSBIOS_RS485_ARDUINO)

# Compile ws2812.pio into ws2812.pio.h
pico_generate_pio_header(DCSBiosRP2350 ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/ws2812.pio
    ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/i2c.pio
)

#pico_generate_pio_header(DCSBiosRP2350 ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/st7789.pio
#)

# Add the standard library to the build
target_link_libraries(DCSBiosRP2350 PRIVATE
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
        hardware_uart
        hardware_dma
        hardware_gpio
        )


target_link_libraries(DCSBiosRP2350 PRIVATE)

# Add the standard include files to the build
target_include_directories(DCSBiosRP2350 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/internal
        # ${CMAKE_CURRENT_LIST_DIR}/src/external/st7789-library-for-pico-main/src/include
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
    src/internal/rs485_arduino.cpp
    src/internal/core1_tasks.cpp
    # src/external/st7789-library-for-pico-main/src/st7789.c
    src/internal/TM1637.cpp
    src/internal/Mcp23s17.cpp
    src/internal/PCF8575.cpp
    src/internal/X27_stepper.cpp
    #src/internal/ADS1115.cpp
    )
pico_add_extra_outputs(DCSBiosRP2350)

