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

// Frame field offsets within the wire buffer [reg][cmd][len][data...][crc8]
#define I2C_FRAME_IDX_REG    0
#define I2C_FRAME_IDX_CMD    1
#define I2C_FRAME_IDX_LEN    2
#define I2C_FRAME_IDX_DATA   3

uint8_t i2cFrame_encode(uint8_t *outBuf, uint8_t reg, uint8_t cmd,
                         const uint8_t *data, uint8_t dataLen);

bool i2cFrame_decode(const uint8_t *inBuf, uint8_t inLen,
                       uint8_t *outReg, uint8_t *outCmd,
                       const uint8_t **outData, uint8_t *outDataLen);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace DcsBios {

// I2C command IDs and register flags. These live here (not in a separate
// command header) because reg/cmd/len are frame-level wire concepts. The
// Arduino slave sketches keep a hand-maintained mirror in
// arduino_sketches/*/i2c_slave_defs.h — keep the copies in sync.
enum I2cCmd : uint8_t {
    I2C_CMD_SET_POSITION    = 0x01,
    I2C_CMD_SET_ANGLE       = 0x02,
    I2C_CMD_HOME_SWEEP      = 0x03,
    I2C_CMD_HOME_SENSOR     = 0x04,
    I2C_CMD_STATUS_REQ      = 0x05,
    I2C_CMD_SET_MODE        = 0x06,
    I2C_CMD_CONFIG_GET      = 0x07,
    I2C_CMD_CONFIG_SET      = 0x08,
    I2C_CMD_SET_BACKLIGHT   = 0x09,
    I2C_CMD_RESET           = 0x7F,
};

constexpr uint8_t I2C_REG_SYSTEM       = 0x00;
constexpr uint8_t I2C_REG_RESPONSE     = 0x80;
constexpr uint8_t I2C_REG_MASK         = 0x7F;
constexpr uint8_t I2C_PROTOCOL_VERSION = 0x01;

} // namespace DcsBios
#endif

#endif
