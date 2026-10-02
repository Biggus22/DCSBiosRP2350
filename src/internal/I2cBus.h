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

    // Write a request frame with no STOP, then read exactly rxLen bytes back.
    // readFrame() always requests I2C_FRAME_MAX_SIZE (254) bytes, so it cannot
    // be used against a slave that streams a shorter response from its
    // onRequest handler. Callers here know the exact response size.
    virtual bool readFrameSized(uint8_t addr, const uint8_t *writeData, uint8_t writeLen,
                                uint8_t *rxBuf, uint8_t rxLen,
                                uint32_t timeout_ms = 100) = 0;

    // Clock SCL out and re-issue a STOP, in case a slave that reset mid-
    // transaction left the bus wedged. No-op until the bus has been initialised.
    virtual void recoverBus() = 0;
};

} // namespace DcsBios

#endif
