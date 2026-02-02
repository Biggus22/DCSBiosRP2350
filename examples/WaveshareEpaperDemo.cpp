#if __has_include("pico/stdlib.h") && __has_include("pico/assert.h")
#include "pico/stdlib.h"
#else
#include <chrono>
#include <thread>
inline void stdio_init_all() {}
inline void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#endif

#if __has_include("hardware/spi.h")
#include "hardware/spi.h"
#else
struct spi_inst_t {};
static spi_inst_t* const spi1 = nullptr;
#endif
#include "src/internal/WaveshareEpaper213.h"

using namespace DcsBios;

int main() {
    stdio_init_all();

    WaveshareEpaperConnection connection;
    connection.spi = spi1;
    connection.mosiPin = 11;
    connection.sckPin = 10;
    connection.csPin = 9;
    connection.dcPin = 8;
    connection.rstPin = 12;
    connection.busyPin = 13;
    connection.busyActiveHigh = false;
    connection.baudRateHz = 8 * 1000 * 1000;

    WaveshareEpaperGeometry geometry{WaveshareEpaper213::DefaultWidth,
                                     WaveshareEpaper213::DefaultHeight};

    WaveshareEpaper213 display(connection, geometry);
    display.init();
    display.clear(false);
    display.drawText(10, 40, "Hello World", true);
    display.display();

    while (true) {
        sleep_ms(1000);
    }
}
