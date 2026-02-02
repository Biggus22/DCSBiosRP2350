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


int main() {
    stdio_init_all();  // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
    sleep_ms(1000);   // Wait for USB CDC to be ready

    
    uint8_t boardAddress = 0xF; // USB mode, set manually. See FoxConfig.h for details.

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

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);  
    printf("Core 1 task launched!\n");

    DcsBios::LED masterModeAaLt(FA_18C_hornet_MASTER_MODE_AA_LT_AM, 1);
    DcsBios::LED masterModeAgLt(FA_18C_hornet_MASTER_MODE_AG_LT_AM, 2);
    DcsBios::Switch2Pos masterArmSw("MASTER_ARM_SW", 3);
    DcsBios::Switch2Pos emerJettBtn("EMER_JETT_BTN", 6);

    DcsBios::setup();  // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true) {
        DcsBios::loop(); // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}
