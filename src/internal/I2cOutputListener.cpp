#include "I2cOutputListener.h"
#include "I2cFrame.h"

namespace DcsBios {

I2cOutputListener::I2cOutputListener(unsigned int address,
                                     I2cBus *bus,
                                     uint8_t slaveAddr,
                                     uint8_t regId,
                                     ScaleMode scaleMode,
                                     uint16_t span,
                                     bool reversed,
                                     uint16_t deadband)
    : Int16Buffer(address),
      bus_(bus),
      slaveAddr_(slaveAddr),
      regId_(regId),
      scaleMode_(scaleMode),
      span_(span),
      reversed_(reversed),
      deadband_(deadband),
      lastSteps_(-1) {}

void I2cOutputListener::loop() {
    if (!hasUpdatedData()) return;

    unsigned int raw = getData();
    uint8_t dataBuf[2];
    uint8_t dataLen = 0;
    uint8_t cmd = I2C_CMD_SET_POSITION;

    switch (scaleMode_) {
        case LED_TOGGLE:
            dataBuf[0] = (raw != 0) ? 1 : 0;
            dataLen = 1;
            break;

        case LED_BRIGHTNESS:
            // Scale DCS-BIOS 16-bit value (0-65535) to 8-bit PWM brightness (0-255)
            dataBuf[0] = (uint8_t)((raw * 255) / 65535);
            dataLen = 1;
            break;

        case STEPS_16BIT:
            // Pass 16-bit raw value as little-endian: low byte first, high byte second
            dataBuf[0] = raw & 0xFF;
            dataBuf[1] = (raw >> 8) & 0xFF;
            dataLen = 2;
            break;

        case ANGLE_CENTIDEG: {
            // Scale DCS-BIOS 16-bit value (0-65535) to Switec centidegrees (0-31500)
            uint16_t angle = (uint16_t)((raw * 31500) / 65535);
            dataBuf[0] = angle & 0xFF;
            dataBuf[1] = (angle >> 8) & 0xFF;
            dataLen = 2;
            cmd = I2C_CMD_SET_ANGLE;
            break;
        }

        case BACKLIGHT:
            dataBuf[0] = (uint8_t)((raw * 255) / 65535);
            dataLen = 1;
            cmd = I2C_CMD_SET_BACKLIGHT;
            break;

        case STEPS_SCALED: {
            // The master owns the whole mapping: full-scale span, direction and
            // a deadband that absorbs DCS jitter (so the slave never has to be
            // reflashed to tune the gauge). Endpoints always go out.
            uint16_t steps = (uint16_t)(((uint32_t)raw * span_) / 65535u);
            if (steps > span_) steps = span_;
            if (reversed_) steps = (uint16_t)(span_ - steps);

            const int32_t diff = (int32_t)steps - lastSteps_;
            const bool atEnd = (steps == 0 || steps == span_);
            if (lastSteps_ >= 0 && !atEnd && (diff < 0 ? -diff : diff) < (int32_t)deadband_) {
                return;
            }
            lastSteps_ = steps;

            dataBuf[0] = steps & 0xFF;
            dataBuf[1] = (steps >> 8) & 0xFF;
            dataLen = 2;
            cmd = I2C_CMD_SET_POSITION;
            break;
        }
    }

    sendFrame_(cmd, dataBuf, dataLen);
}

void I2cOutputListener::sendFrame_(uint8_t cmd, const uint8_t *data, uint8_t dataLen) {
    bus_->sendFrame(slaveAddr_, regId_, cmd, data, dataLen, 10);
}

} // namespace DcsBios
