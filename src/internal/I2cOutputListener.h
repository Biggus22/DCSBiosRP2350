#ifndef __DCSBIOS_I2COUTPUTLISTENER_H
#define __DCSBIOS_I2COUTPUTLISTENER_H

#include "ExportStreamListener.h"
#include "I2cBus.h"

namespace DcsBios {

class I2cOutputListener : public Int16Buffer {
public:
    enum ScaleMode : uint8_t {
        LED_TOGGLE     = 0,
        LED_BRIGHTNESS = 1,
        STEPS_16BIT    = 2,
        ANGLE_CENTIDEG = 3,
    };

    I2cOutputListener(unsigned int address,
                      I2cBus *bus,
                      uint8_t slaveAddr,
                      uint8_t regId = 0x01,
                      ScaleMode scaleMode = LED_BRIGHTNESS);

    void loop() override;

private:
    I2cBus *bus_;
    uint8_t slaveAddr_;
    uint8_t regId_;
    ScaleMode scaleMode_;
    uint16_t lastRawSent_;

    bool deadbandPass_(unsigned int raw);
    void sendFrame_(uint8_t cmd, const uint8_t *data, uint8_t dataLen);
};

} // namespace DcsBios

#endif
