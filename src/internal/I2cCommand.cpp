#include "I2cCommand.h"
#include <cstring>

namespace DcsBios {

I2cCommandRegistry::I2cCommandRegistry() {
    std::memset(handlers_, 0, sizeof(handlers_));
}

I2cCommandRegistry& I2cCommandRegistry::instance() {
    static I2cCommandRegistry reg;
    return reg;
}

void I2cCommandRegistry::registerCommand(uint8_t cmdId, I2cCmdHandler handler) {
    if (cmdId < MAX_HANDLERS) {
        handlers_[cmdId] = handler;
    }
}

bool I2cCommandRegistry::dispatch(uint8_t cmdId, uint8_t reg,
                                   const uint8_t *data, uint8_t len) const {
    if (cmdId >= MAX_HANDLERS || handlers_[cmdId] == nullptr) {
        return false;
    }
    return handlers_[cmdId](reg, data, len);
}

bool I2cCommandRegistry::hasHandler(uint8_t cmdId) const {
    return (cmdId < MAX_HANDLERS && handlers_[cmdId] != nullptr);
}

} // namespace DcsBios
