#include "I2cBusHwMaster.h"
#include "I2cFrame.h"

namespace DcsBios {

I2cBusHwMaster::I2cBusHwMaster(i2c_inst_t *i2c, uint sda, uint scl, uint32_t baud)
    : i2c_(i2c), sda_(sda), scl_(scl), baud_(baud) {}

void I2cBusHwMaster::init_() {
    i2c_init(i2c_, baud_);
    gpio_set_function(sda_, GPIO_FUNC_I2C);
    gpio_set_function(scl_, GPIO_FUNC_I2C);
    gpio_pull_up(sda_);
    gpio_pull_up(scl_);
    initialized_ = true;
}

bool I2cBusHwMaster::sendFrame(uint8_t addr, uint8_t reg, uint8_t cmd,
                                const uint8_t *data, uint8_t dataLen,
                                uint32_t timeout_ms) {
    if (!initialized_) init_();

    uint8_t buf[I2C_FRAME_MAX_SIZE];
    uint8_t total = i2cFrame_encode(buf, reg, cmd, data, dataLen);
    if (total == 0) return false;

    int ret = i2c_write_blocking_until(i2c_, addr, buf, total, false,
                                       make_timeout_time_ms(timeout_ms));
    return (ret == (int)total);
}

bool I2cBusHwMaster::readFrame(uint8_t addr, uint8_t reg, uint8_t cmd,
                                const uint8_t *writeData, uint8_t writeLen,
                                uint8_t *outReg, uint8_t *outCmd,
                                uint8_t *outData, uint8_t *outDataLen,
                                uint32_t timeout_ms) {
    if (!initialized_) init_();

    uint8_t txBuf[I2C_FRAME_MAX_SIZE];
    uint8_t txLen = i2cFrame_encode(txBuf, reg, cmd, writeData, writeLen);
    if (txLen == 0) return false;

    int ret = i2c_write_blocking_until(i2c_, addr, txBuf, txLen, true,
                                       make_timeout_time_ms(timeout_ms));
    if (ret != (int)txLen) return false;

    uint8_t rxBuf[I2C_FRAME_MAX_SIZE];
    ret = i2c_read_blocking_until(i2c_, addr, rxBuf, sizeof(rxBuf), false,
                                  make_timeout_time_ms(timeout_ms));
    if (ret <= 0) return false;

    return i2cFrame_decode(rxBuf, (uint8_t)ret,
                           outReg, outCmd,
                           (const uint8_t**)&outData, outDataLen);
}

} // namespace DcsBios
