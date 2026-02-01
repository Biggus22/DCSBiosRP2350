#include "rs485_arduino.h"

// =============================================================================
// Arduino DCS-BIOS RS485 Compatibility - Implementation
// =============================================================================
//
// This file implements the frame receiver state machine and active slave
// tracking logic for Arduino RS485 slave compatibility.
//
// See rs485_arduino.h for protocol overview and usage documentation.
// =============================================================================

namespace DcsBios {
namespace ArduinoRs485 {

// ============================================================================
// FrameReceiver Implementation
// ============================================================================

FrameReceiver::FrameReceiver() {
    reset();
}

void FrameReceiver::reset() {
    state_ = State::WaitAddress;
    working_ = Frame{};
    dataPos_ = 0;
}

// State machine for parsing Arduino RS485 binary frames byte-by-byte.
//
// Frame wire format: [address][msgType][length][data...][checksum]
//
// State transitions:
//   WaitAddress -> WaitMsgType (always)
//   WaitMsgType -> WaitLength (always)
//   WaitLength -> WaitData (if length > 0) OR WaitChecksum (if length == 0)
//   WaitData -> WaitChecksum (when dataPos >= length)
//   WaitChecksum -> WaitAddress (frame complete, return true)
//
// Error handling:
//   - If length > FRAME_MAX, reset immediately (prevents buffer overflow)
//   - Data buffer has bounds checking (redundant safety)
bool FrameReceiver::pushByte(uint8_t byte, Frame& outFrame) {
    switch (state_) {
        case State::WaitAddress:
            // Start of new frame: reset working frame and capture address
            working_ = Frame{};
            working_.address = byte;
            state_ = State::WaitMsgType;
            break;

        case State::WaitMsgType:
            // Second byte: message type (currently only 0 is used)
            working_.msgType = byte;
            state_ = State::WaitLength;
            break;

        case State::WaitLength:
            // Third byte: payload length
            working_.length = byte;
            dataPos_ = 0;

            if (working_.length == 0) {
                // Zero-length frame (simple poll): skip data, go to checksum
                state_ = State::WaitChecksum;
            } else if (working_.length > DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                // Invalid length: reset to prevent buffer overflow
                reset();
            } else {
                // Valid length: prepare to receive data bytes
                state_ = State::WaitData;
            }
            break;

        case State::WaitData:
            // Accumulate data bytes into working frame buffer
            if (dataPos_ < DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                working_.data[dataPos_++] = byte;
            }

            // When all data bytes received, move to checksum
            if (dataPos_ >= working_.length) {
                state_ = State::WaitChecksum;
            }
            break;

        case State::WaitChecksum:
            // Final byte: checksum (currently unused, always 0)
            working_.checksum = byte;

            // Frame complete: copy to output and reset for next frame
            outFrame = working_;
            reset();
            return true; // Signal: frame ready
    }

    return false; // Signal: frame not yet complete
}

// ============================================================================
// ActiveSlaveSet Implementation
// ============================================================================

ActiveSlaveSet::ActiveSlaveSet() {
    clear();
}

void ActiveSlaveSet::setConfig(const PollConfig& config) {
    config_ = config;
}

const PollConfig& ActiveSlaveSet::getConfig() const {
    return config_;
}

// Reset all slave state to initial (unknown) condition
void ActiveSlaveSet::clear() {
    for (uint8_t i = 0; i < kAddressCount; ++i) {
        active_[i] = false;      // No slaves known active
        missCount_[i] = 0;       // No timeouts recorded
    }
    lastAddress_ = kMinSlaveAddress; // Start polling from address 1
}

// Manually mark a slave as active or inactive.
// Useful for pre-configuring known slave addresses without waiting for discovery.
void ActiveSlaveSet::markActive(uint8_t address, bool active) {
    if (!isValidAddress(address)) {
        return; // Ignore invalid addresses
    }
    active_[address] = active;
    missCount_[address] = 0; // Reset miss counter
}

// Record a successful response from a slave.
// This marks the slave as active and resets its consecutive miss counter.
void ActiveSlaveSet::recordResponse(uint8_t address) {
    if (!isValidAddress(address)) {
        return;
    }
    active_[address] = true;  // Slave is confirmed active
    missCount_[address] = 0;  // Reset timeout counter
}

// Record a timeout from a slave (no response to poll).
// Increments the miss counter; if it exceeds maxMisses, mark slave inactive.
// This allows temporary issues (noise, busy slave) without immediate deactivation.
void ActiveSlaveSet::recordTimeout(uint8_t address) {
    if (!isValidAddress(address)) {
        return;
    }

    // Increment miss counter (with overflow protection)
    if (missCount_[address] < 255) {
        ++missCount_[address];
    }

    // Deactivate slave if miss threshold exceeded
    if (missCount_[address] >= config_.maxMisses) {
        active_[address] = false;
    }
}

// Check if a specific slave address is currently marked active
bool ActiveSlaveSet::isActive(uint8_t address) const {
    if (!isValidAddress(address)) {
        return false;
    }
    return active_[address];
}

// Check if any slaves are currently marked active.
// Used to determine whether to use active-only polling or fall back to full scan.
bool ActiveSlaveSet::hasAnyActive() const {
    for (uint8_t i = kMinSlaveAddress; i <= kMaxSlaveAddress; ++i) {
        if (active_[i]) {
            return true;
        }
    }
    return false; // No active slaves found
}

// Returns the next slave address to poll (round-robin order).
//
// ALGORITHM:
// 1. If activeOnly mode is disabled: always scan all addresses
// 2. If activeOnly mode is enabled:
//    a. If any active slaves exist: only poll active addresses
//    b. If no active slaves exist: scan all addresses (discovery mode)
//
// This ensures:
// - Minimum latency when slaves are known (active-only polling)
// - Automatic discovery when no slaves are active (full scan fallback)
// - Fair round-robin distribution across active slaves
uint8_t ActiveSlaveSet::nextAddress() {
    const bool activeOnly = config_.activeOnly;
    const bool anyActive = hasAnyActive();

    // Scan from current position to find next valid address
    uint8_t probe = lastAddress_;
    for (uint8_t i = 0; i < kAddressCount; ++i) {
        probe = advanceAddress(probe);

        // Accept this address if:
        // - activeOnly is disabled (poll everything), OR
        // - no active slaves exist (discovery mode), OR
        // - this specific address is active
        if (!activeOnly || !anyActive || active_[probe]) {
            lastAddress_ = probe;
            return probe;
        }
    }

    // Fallback: if nothing matched (shouldn't happen), just advance once
    lastAddress_ = advanceAddress(lastAddress_);
    return lastAddress_;
}

// Check if an address is in the valid slave range (1-126)
bool ActiveSlaveSet::isValidAddress(uint8_t address) const {
    return address >= kMinSlaveAddress && address <= kMaxSlaveAddress;
}

// Advance to the next address in round-robin order (with wraparound)
uint8_t ActiveSlaveSet::advanceAddress(uint8_t address) const {
    if (address < kMinSlaveAddress || address >= kMaxSlaveAddress) {
        return kMinSlaveAddress; // Wrap back to start
    }
    return static_cast<uint8_t>(address + 1);
}

} // namespace ArduinoRs485
} // namespace DcsBios
