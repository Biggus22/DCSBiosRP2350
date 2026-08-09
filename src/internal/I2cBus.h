#ifndef __DCSBIOS_I2CBUS_H
#define __DCSBIOS_I2CBUS_H

#include <stdint.h>

namespace DcsBios {

class I2cBus {
public:
    virtual ~I2cBus() = default;

    virtual bool sendFrame(uint8_t addr, uint8_t reg, uint8_t cmd,
                           const uint8_t *data, uint8_t dataLen,
                           uint32_t timeout_ms = 100) = 0;

    virtual bool readFrame(uint8_t addr, uint8_t reg, uint8_t cmd,
                          const uint8_t *writeData, uint8_t writeLen,
                          uint8_t *outReg, uint8_t *outCmd,
                          uint8_t *outData, uint8_t *outDataLen,
                          uint32_t timeout_ms = 100) = 0;
};

} // namespace DcsBios

#endif
