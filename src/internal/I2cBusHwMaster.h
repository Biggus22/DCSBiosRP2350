#ifndef __DCSBIOS_I2CBUSHWMASTER_H
#define __DCSBIOS_I2CBUSHWMASTER_H

#include "I2cBus.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "FoxConfig.h"

namespace DcsBios {

class I2cBusHwMaster : public I2cBus {
private:
    static constexpr uint32_t I2C_DEFAULT_BAUD = 100000;  // default I2C bus speed (100 kHz)

    i2c_inst_t *i2c_;
    uint sda_;
    uint scl_;
    uint32_t baud_;
    bool initialized_ = false;

    void init_();

public:
I2cBusHwMaster(i2c_inst_t *i2c = i2c1,
               uint sda = I2C1_SDA,
               uint scl = I2C1_SCL,
               uint32_t baud = I2C_DEFAULT_BAUD);

    bool sendFrame(uint8_t addr, uint8_t reg, uint8_t cmd,
                   const uint8_t *data, uint8_t dataLen,
                   uint32_t timeout_ms = 100) override;

    bool readFrame(uint8_t addr, uint8_t reg, uint8_t cmd,
                   const uint8_t *writeData, uint8_t writeLen,
                   uint8_t *outReg, uint8_t *outCmd,
                   uint8_t *outData, uint8_t *outDataLen,
                   uint32_t timeout_ms = 100) override;
};

} // namespace DcsBios

#endif
