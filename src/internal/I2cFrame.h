#ifndef __DCSBIOS_I2CFRAME_H
#define __DCSBIOS_I2CFRAME_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t i2cFrame_crc8(const uint8_t *data, uint8_t len);

#define I2C_FRAME_MAX_PAYLOAD 250
#define I2C_FRAME_OVERHEAD    4
#define I2C_FRAME_MAX_SIZE   (I2C_FRAME_MAX_PAYLOAD + I2C_FRAME_OVERHEAD)

uint8_t i2cFrame_encode(uint8_t *outBuf, uint8_t reg, uint8_t cmd,
                         const uint8_t *data, uint8_t dataLen);

bool i2cFrame_decode(const uint8_t *inBuf, uint8_t inLen,
                      uint8_t *outReg, uint8_t *outCmd,
                      const uint8_t **outData, uint8_t *outDataLen);

#ifdef __cplusplus
}
#endif

#endif
