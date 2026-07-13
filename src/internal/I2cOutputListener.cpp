#include "I2cOutputListener.h"
#include "I2cCommand.h"

namespace DcsBios {

I2cOutputListener::I2cOutputListener(unsigned int address,
                                     I2cBus *bus,
                                     uint8_t slaveAddr,
                                     uint8_t regId,
                                     ScaleMode scaleMode)
    : Int16Buffer(address),
      bus_(bus),
      slaveAddr_(slaveAddr),
      regId_(regId),
      scaleMode_(scaleMode),
      lastRawSent_(0xFFFF) {}

bool I2cOutputListener::deadbandPass_(unsigned int raw) {
    if (scaleMode_ == LED_TOGGLE || scaleMode_ == LED_BRIGHTNESS) {
        return true;
    }
    uint16_t diff = (raw > lastRawSent_) ? (raw - lastRawSent_) : (lastRawSent_ - raw);
    if (diff < 128) {
        return false;
    }
    lastRawSent_ = raw;
    return true;
}

void I2cOutputListener::loop() {
    if (!hasUpdatedData()) return;

    unsigned int raw = getData();
    if (!deadbandPass_(raw)) return;
    uint8_t dataBuf[2];
    uint8_t dataLen = 0;
    uint8_t cmd = I2C_CMD_SET_POSITION;

    switch (scaleMode_) {
        case LED_TOGGLE:
            dataBuf[0] = (raw != 0) ? 1 : 0;
            dataLen = 1;
            break;

        case LED_BRIGHTNESS:
            dataBuf[0] = (uint8_t)((raw * 255) / 65535);
            dataLen = 1;
            break;

        case STEPS_16BIT:
            dataBuf[0] = raw & 0xFF;
            dataBuf[1] = (raw >> 8) & 0xFF;
            dataLen = 2;
            break;

        case ANGLE_CENTIDEG: {
            uint16_t angle = (uint16_t)((raw * 31500) / 65535);
            dataBuf[0] = angle & 0xFF;
            dataBuf[1] = (angle >> 8) & 0xFF;
            dataLen = 2;
            cmd = I2C_CMD_SET_ANGLE;
            break;
        }
    }

    sendFrame_(cmd, dataBuf, dataLen);
}

void I2cOutputListener::sendFrame_(uint8_t cmd, const uint8_t *data, uint8_t dataLen) {
    bus_->sendFrame(slaveAddr_, regId_, cmd, data, dataLen, 10);
}

} // namespace DcsBios
