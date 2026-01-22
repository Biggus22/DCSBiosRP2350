#include "rs485_arduino.h"

namespace DcsBios {
namespace ArduinoRs485 {

FrameReceiver::FrameReceiver() {
    reset();
}

void FrameReceiver::reset() {
    state_ = State::WaitAddress;
    working_ = Frame{};
    dataPos_ = 0;
}

bool FrameReceiver::pushByte(uint8_t byte, Frame& outFrame) {
    switch (state_) {
        case State::WaitAddress:
            working_ = Frame{};
            working_.address = byte;
            state_ = State::WaitMsgType;
            break;
        case State::WaitMsgType:
            working_.msgType = byte;
            state_ = State::WaitLength;
            break;
        case State::WaitLength:
            working_.length = byte;
            dataPos_ = 0;
            if (working_.length == 0) {
                state_ = State::WaitChecksum;
            } else if (working_.length > DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                reset();
            } else {
                state_ = State::WaitData;
            }
            break;
        case State::WaitData:
            if (dataPos_ < DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                working_.data[dataPos_++] = byte;
            }
            if (dataPos_ >= working_.length) {
                state_ = State::WaitChecksum;
            }
            break;
        case State::WaitChecksum:
            working_.checksum = byte;
            outFrame = working_;
            reset();
            return true;
    }

    return false;
}

ActiveSlaveSet::ActiveSlaveSet() {
    clear();
}

void ActiveSlaveSet::setConfig(const PollConfig& config) {
    config_ = config;
}

const PollConfig& ActiveSlaveSet::getConfig() const {
    return config_;
}

void ActiveSlaveSet::clear() {
    for (uint8_t i = 0; i < kAddressCount; ++i) {
        active_[i] = false;
        missCount_[i] = 0;
    }
    lastAddress_ = kMinSlaveAddress;
}

void ActiveSlaveSet::markActive(uint8_t address, bool active) {
    if (!isValidAddress(address)) {
        return;
    }
    active_[address] = active;
    missCount_[address] = 0;
}

void ActiveSlaveSet::recordResponse(uint8_t address) {
    if (!isValidAddress(address)) {
        return;
    }
    active_[address] = true;
    missCount_[address] = 0;
}

void ActiveSlaveSet::recordTimeout(uint8_t address) {
    if (!isValidAddress(address)) {
        return;
    }
    if (missCount_[address] < 255) {
        ++missCount_[address];
    }
    if (missCount_[address] >= config_.maxMisses) {
        active_[address] = false;
    }
}

bool ActiveSlaveSet::isActive(uint8_t address) const {
    if (!isValidAddress(address)) {
        return false;
    }
    return active_[address];
}

bool ActiveSlaveSet::hasAnyActive() const {
    for (uint8_t i = kMinSlaveAddress; i <= kMaxSlaveAddress; ++i) {
        if (active_[i]) {
            return true;
        }
    }
    return false;
}

uint8_t ActiveSlaveSet::nextAddress() {
    const bool activeOnly = config_.activeOnly;
    const bool anyActive = hasAnyActive();

    uint8_t probe = lastAddress_;
    for (uint8_t i = 0; i < kAddressCount; ++i) {
        probe = advanceAddress(probe);
        if (!activeOnly || !anyActive || active_[probe]) {
            lastAddress_ = probe;
            return probe;
        }
    }

    // Fallback: if nothing matched, just advance once.
    lastAddress_ = advanceAddress(lastAddress_);
    return lastAddress_;
}

bool ActiveSlaveSet::isValidAddress(uint8_t address) const {
    return address >= kMinSlaveAddress && address <= kMaxSlaveAddress;
}

uint8_t ActiveSlaveSet::advanceAddress(uint8_t address) const {
    if (address < kMinSlaveAddress || address >= kMaxSlaveAddress) {
        return kMinSlaveAddress;
    }
    return static_cast<uint8_t>(address + 1);
}

} // namespace ArduinoRs485
} // namespace DcsBios
