#ifndef __DCSBIOS_I2CCOMMAND_H
#define __DCSBIOS_I2CCOMMAND_H

#include <stdint.h>

namespace DcsBios {

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

constexpr uint8_t I2C_REG_SYSTEM     = 0x00;
constexpr uint8_t I2C_REG_RESPONSE   = 0x80;
constexpr uint8_t I2C_REG_MASK       = 0x7F;
constexpr uint8_t I2C_PROTOCOL_VERSION = 0x01;

using I2cCmdHandler = bool (*)(uint8_t reg, const uint8_t *data, uint8_t len);

class I2cCommandRegistry {
public:
    static I2cCommandRegistry& instance();

    void registerCommand(uint8_t cmdId, I2cCmdHandler handler);
    bool dispatch(uint8_t cmdId, uint8_t reg, const uint8_t *data, uint8_t len) const;
    bool hasHandler(uint8_t cmdId) const;

private:
    I2cCommandRegistry();
    static constexpr uint8_t MAX_HANDLERS = 128;
    I2cCmdHandler handlers_[MAX_HANDLERS];
};

} // namespace DcsBios

#endif
