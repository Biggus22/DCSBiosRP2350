#ifndef __DCSBIOS_FOXCONFIG_H
#define __DCSBIOS_FOXCONFIG_H

#define HOST_MODE 0x0
#define SLAVE_MODE_MIN 0x1
#define SLAVE_MODE_MAX 0xD
#define RS485_TERMINAL_MODE 0xE
#define USB_MODE 0xF
#define WATCHDOG_TIMEOUT 5000

// ==== Validate board type ====
#if !defined(FOX1_BOARD) && !defined(FOX2_BOARD) && !defined(PICO_BOARD)
    #error "You must define FOX1_BOARD or FOX2_BOARD before including Fox.h"
#endif

// ==== FOX1 Pinout ====
#if defined(FOX1_BOARD) 
    #define HEARTBEAT_LED 33
    #define ERROR_LED 32
    #define BOARD_LED 31

    #define I2C0_SDA 4
    #define I2C0_SCL 5

    #define I2C1_SDA 6
    #define I2C1_SCL 7

    #define UART0_TX 0
    #define UART0_RX 1
    #define RS485_EN 2

    #define ADDR0 36
    #define ADDR1 37
    #define ADDR2 38
    #define ADDR3 39

    // AW9523B I2C Addresses
    #define AW_DEV_LED     0x58  // Dedicated to LED brightness control
    #define AW_DEV_IO1     0x59  // General purpose I/O
    #define AW_DEV_IO2     0x5B  // General purpose I/O

    #define AW_DEV_LED_RESET 24
    #define AW_DEV_IO1_RESET 26
    #define AW_DEV_IO2_RESET 35

    #define AW_DEV_IO1_INT 25
    #define AW_DEV_IO2_INT 34

    // LED mappings: (address, port, pin)
    #define A0 0 
    #define A1 1
    #define A2 2
    #define A3 3
    #define A4 4
    #define A5 5
    #define A6 6
    #define A7 7
    #define A8 8
    #define A9 9
    #define A10 10
    #define A11 11
    #define A12 12
    #define A13 13
    #define A14 14
    #define A15 15

    // IO1 on AW9523B
    #define B0 0 
    #define B1 1
    #define B2 2
    #define B3 3
    #define B4 4
    #define B5 5
    #define B6 6
    #define B7 7
    #define B8 8
    #define B9 9
    #define B10 10
    #define B11 11
    #define B12 12
    #define B13 13
    #define B14 14
    #define B15 15

    // IO2 on AW9523B
    #define C0 0 
    #define C1 1
    #define C2 2
    #define C3 3
    #define C4 4
    #define C5 5
    #define C6 6
    #define C7 7
    #define C8 8
    #define C9 9
    #define C10 10
    #define C11 11
    #define C12 12
    #define C13 13
    #define C14 14
    #define C15 15


#elif defined(FOX2_BOARD)
    #define ERROR_LED 32
    #define HEARTBEAT_LED 33
    #define BOARD_LED 31

    #define I2C0_SDA 5  // swapped due to error on board
    #define I2C0_SCL 4  // swapped due to error on board

    #define I2C1_SDA 6
    #define I2C1_SCL 7

    #define UART0_TX 0
    #define UART0_RX 1
    #define RS485_EN 2

    #define ADDR0 36
    #define ADDR1 37
    #define ADDR2 38
    #define ADDR3 39

    #define NEO_DRIVE_PIN 34

    #define NEO_LED1 0
    #define NEO_LED2 1
    #define NEO_LED3 2
    #define NEO_LED4 3
    #define NEO_LED5 4
    #define NEO_LED6 5
    #define NEO_LED7 6
    #define NEO_LED8 7
    #define NEO_LED9 8
    #define NEO_LED10 9
    #define NEO_LED11 10
    #define NEO_LED12 11
    #define NEO_LED13 12
    #define NEO_LED14 13
    #define NEO_LED15 14
    #define NEO_LED16 15
    #define NEO_LED17 16
    #define NEO_LED18 17
    #define NEO_LED19 18
    #define NEO_LED20 19
    #define NEO_LED21 20
    #define NEO_LED22 21
    #define NEO_LED23 22
    #define NEO_LED24 23

    #define ENCODER1_A 9
    #define ENCODER1_B 10
    #define ENCODER1_SW 11

    #define ENCODER2_A 12
    #define ENCODER2_B 13
    #define ENCODER2_SW 14

    #define ENCODER3_A 15
    #define ENCODER3_B 16
    #define ENCODER3_SW 17

#elif defined(PICO_BOARD)
    #define HEARTBEAT_LED 29
    #define NEO_DRIVE_PIN 27

#endif

#endif // __DCSBIOS_FOXCONFIG_H