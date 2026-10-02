#include "I2cInputPanel.h"
#include "I2cFrame.h"

namespace DcsBios {

I2cInputPanel::I2cInputPanel(I2cBus *bus, uint8_t slaveAddr, uint8_t regId, uint8_t cmd,
                               uint8_t payloadLen, uint32_t pollIntervalMs, uint32_t timeoutMs)
    : bus_(bus), slaveAddr_(slaveAddr), regId_(regId), cmd_(cmd), payloadLen_(payloadLen),
      pollIntervalMs_(pollIntervalMs), timeoutMs_(timeoutMs), lastPollMs_(0),
      state_(0), adc_{0, 0}, online_(false), failCount_(0) {}

bool I2cInputPanel::decode_(const uint8_t *rx, uint8_t rxLen,
                             uint8_t expectedReg, uint8_t expectedCmd,
                             uint8_t expectedPayloadLen,
                             uint8_t *state, uint16_t *adc) {
    uint8_t reg = 0;
    uint8_t cmd = 0;
    uint8_t dataLen = 0;
    const uint8_t *data = nullptr;

    if (!i2cFrame_decode(rx, rxLen, &reg, &cmd, &data, &dataLen)) return false;
    if (dataLen != expectedPayloadLen) return false;
    if (cmd != expectedCmd) return false;

    // The slave flags its own frames with bit 7; the low 7 bits must still match
    // the register the request was addressed to.
    if ((reg & I2C_REG_MASK) != (expectedReg & I2C_REG_MASK)) return false;
    if ((reg & I2C_REG_RESPONSE) == 0) return false;

    if (data == nullptr) return false;

    *state = (expectedPayloadLen > I2C_INPUT_STATE) ? data[I2C_INPUT_STATE] : 0;

    for (uint8_t i = 0; i < MAX_ADC_CHANNELS; i++) {
        const uint8_t off = (uint8_t)(I2C_INPUT_ADC_0 + (i * 2));
        adc[i] = 0;
        if (expectedPayloadLen >= (uint8_t)(off + 2)) {
            adc[i] = (uint16_t)data[off] | ((uint16_t)data[off + 1] << 8);
        }
    }
    return true;
}

void I2cInputPanel::poll() {
    const unsigned long now = to_ms_since_boot(get_absolute_time());
    if (pollIntervalMs_ != 0 && (now - lastPollMs_) < pollIntervalMs_) return;
    lastPollMs_ = now;

    uint8_t req[I2C_FRAME_OVERHEAD];
    const uint8_t reqLen = i2cFrame_encode(req, regId_, cmd_, nullptr, 0);
    if (reqLen == 0) {
        online_ = false;
        return;
    }

    uint8_t rx[I2C_FRAME_MAX_SIZE];
    const uint8_t rxLen = (uint8_t)(payloadLen_ + I2C_FRAME_OVERHEAD);
    if (rxLen > sizeof(rx)) {
        online_ = false;
        return;
    }

    if (!bus_->readFrameSized(slaveAddr_, req, reqLen, rx, rxLen, timeoutMs_)) {
        online_ = false;
        failCount_++;
        // A slave that resets mid-transaction can leave SDA held low, after
        // which every transaction NACKs forever. Poke the bus periodically
        // until the link comes back.
        if ((failCount_ % RECOVER_BUS_EVERY) == 0) bus_->recoverBus();
        return;
    }

    uint8_t state = 0;
    uint16_t adc[MAX_ADC_CHANNELS] = {0, 0};
    if (!decode_(rx, rxLen, regId_, cmd_, payloadLen_, &state, adc)) {
        online_ = false;
        failCount_++;
        return;
    }

    state_ = state;
    adc_[0] = adc[0];
    adc_[1] = adc[1];
    online_ = true;
}

bool I2cInputPanel::readAdc(uint8_t field, uint16_t *out) const {
    if (!online_ || out == nullptr) return false;
    if (field != I2C_INPUT_ADC_0 && field != I2C_INPUT_ADC_1) return false;
    *out = adc_[(field == I2C_INPUT_ADC_0) ? 0 : 1];
    return true;
}

} // namespace DcsBios
