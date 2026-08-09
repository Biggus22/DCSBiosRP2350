#include "I2cProtocol.h"

namespace DcsBios {

static uint8_t g_gaugeCount = 0;

static bool handleStatusReq(uint8_t reg, const uint8_t *data, uint8_t len) {
    (void)reg; (void)data; (void)len;
    return true;
}

static bool handleConfigGet(uint8_t reg, const uint8_t *data, uint8_t len) {
    (void)reg;
    if (len < 1) return false;
    uint8_t key = data[0];
    (void)key;
    return true;
}

static bool handleReset(uint8_t reg, const uint8_t *data, uint8_t len) {
    (void)reg; (void)data; (void)len;
    return true;
}

void initI2cProtocolDefaults() {
    auto& reg = I2cCommandRegistry::instance();
    reg.registerCommand(I2C_CMD_STATUS_REQ, handleStatusReq);
    reg.registerCommand(I2C_CMD_CONFIG_GET, handleConfigGet);
    reg.registerCommand(I2C_CMD_RESET, handleReset);
}

void setI2cGaugeCount(uint8_t count) {
    g_gaugeCount = count;
}

} // namespace DcsBios
