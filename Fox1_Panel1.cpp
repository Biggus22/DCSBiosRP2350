#define FOX1_BOARD

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/aw9523b.h"
#include "internal/ws2812.h"  // Include the WS2812 header


#define FADE_DELAY_MS 1  // Delay between steps

uart_inst_t* rs485_uart = uart0; // Control UART in main


void fadeTest(AW9523B& aw_led) {
    // Fade from off → bright (255 → 0)
    for (int i = 255; i >= 0; i--) {
        aw_led.writeRegister(AW9523B_REG_DIM0, i);
        aw_led.writeRegister(AW9523B_REG_DIM1, i);
        sleep_ms(FADE_DELAY_MS);
    }

    sleep_ms(500);  // Hold full brightness

    // Fade from bright → off (0 → 255)
    for (int i = 0; i <= 255; i++) {
        aw_led.writeRegister(AW9523B_REG_DIM0, i);
        aw_led.writeRegister(AW9523B_REG_DIM1, i);
        sleep_ms(FADE_DELAY_MS);
    }

    sleep_ms(500);  // Hold off
}

void init_i2c0() {
    i2c_init(i2c0, 400 * 1000);  // 400kHz
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
}


AW9523B aw_led(i2c0, AW_DEV_LED);  // Assuming `i2c0` is initialized
AW9523B aw_io1(i2c0, AW_DEV_IO1);
AW9523B aw_io2(i2c0, AW_DEV_IO2);


bool init_external_peripherals() {
    printf("Initializing IO1...\n");
    if (!aw_io1.begin(AW9523B_MODE_GPIO)) return false;
    printf("Initializing LED...\n");
    if (!aw_led.begin(AW9523B_MODE_LED_CURR_100)) return false;
    printf("Initializing IO2...\n");
    if (!aw_io2.begin(AW9523B_MODE_GPIO)) return false;

    return true;
}


int main() {
    stdio_init_all();  // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
    sleep_ms(5000);   // Wait for USB CDC to be ready

    DcsBios::initDeviceAddressPins(ADDR0, ADDR1, ADDR2, ADDR3);
    uint8_t boardAddress = DcsBios::readDeviceAddress();

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    printf("Board address: 0x%X\n", boardAddress);

    switch (board.mode) {
        case DcsBios::BoardModeType::HOST:
            printf("HOST MODE\n");
            break;
        case DcsBios::BoardModeType::SLAVE:
            printf("SLAVE MODE\n");
            break;
        case DcsBios::BoardModeType::USB_ONLY:
            printf("STANDALONE USB MODE\n");
            break;
        case DcsBios::BoardModeType::RS485_TERMINAL:
            printf("RS485 TERMINAL MODE\n");
            break;    
        default:
            printf("INVALID ADDRESS\n");
            break;
    }
    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);  
    printf("Core 1 task launched!\n");


    init_i2c0();  // Initialize I2C0
    sleep_ms(100); 

    printf("Initializing external peripherals...\n");
    init_external_peripherals();  // Initialize external peripherals
    printf("External peripherals initialized!\n");
    



    DcsBios::LED master_AA_LED(FA_18C_hornet_MASTER_MODE_AA_LT_AM, &aw_led, A0, false, 250);
    DcsBios::LED master_AG_LED(FA_18C_hornet_MASTER_MODE_AG_LT_AM, &aw_led, A1, false, 250);
    DcsBios::LED master_AG_LED2(FA_18C_hornet_MASTER_MODE_AG_LT_AM, &aw_led, A2, false, 250);

    DcsBios::Switch2Pos selJettBtn("SEL_JETT_BTN", &aw_io1, B0);

    const uint8_t selJettKnobPins[5] = {B1, B2, B3, B4, B5};
    DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 5> selJettKnob("SEL_JETT_KNOB", &aw_io1, selJettKnobPins);

    DcsBios::Switch2Pos emerJettBtn("EMER_JETT_BTN", 15);

    DcsBios::setup();  // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true) {
        DcsBios::loop(); // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}
