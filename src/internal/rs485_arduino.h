#ifndef __DCSBIOS_RS485_ARDUINO_H
#define __DCSBIOS_RS485_ARDUINO_H

#include <stdint.h>

namespace DcsBios {
namespace ArduinoRs485 {

// Arduino RS485 framing parameters (tunable via compile definitions)
#ifndef DCSBIOS_RS485_ARDUINO_FRAME_MAX
#define DCSBIOS_RS485_ARDUINO_FRAME_MAX 64
#endif

// Active-only polling: poll only known responders, with fallback to full scan
#ifndef DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY
#define DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY 1
#endif

// Deactivate a slave after N consecutive misses
#ifndef DCSBIOS_RS485_ARDUINO_MAX_MISSES
#define DCSBIOS_RS485_ARDUINO_MAX_MISSES 3
#endif

// Poll spacing and timing (microseconds)
#ifndef DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US
#define DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US 10000
#endif

#ifndef DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US
#define DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US 5000
#endif

// Required inter-frame quiet time for Arduino RS485 framing
#ifndef DCSBIOS_RS485_ARDUINO_INTERFRAME_GAP_US
#define DCSBIOS_RS485_ARDUINO_INTERFRAME_GAP_US 600
#endif

constexpr uint8_t kMinSlaveAddress = 1;
constexpr uint8_t kMaxSlaveAddress = 126;
constexpr uint8_t kAddressCount = 127; // 0..126

struct PollConfig {
    // When true, only poll known-active slaves; otherwise poll all addresses.
    bool activeOnly = true;

    // Mark a slave inactive after this many consecutive timeouts.
    uint8_t maxMisses = 3;
};

struct Frame {
    uint8_t address = 0;
    uint8_t msgType = 0;
    uint8_t length = 0;
    uint8_t data[DCSBIOS_RS485_ARDUINO_FRAME_MAX]{};
    uint8_t checksum = 0;
};

class FrameReceiver {
public:
    FrameReceiver();

    // Returns true when a full frame has been received into outFrame.
    bool pushByte(uint8_t byte, Frame& outFrame);
    void reset();

private:
    enum class State {
        WaitAddress,
        WaitMsgType,
        WaitLength,
        WaitData,
        WaitChecksum
    };

    State state_ = State::WaitAddress;
    Frame working_{};
    uint8_t dataPos_ = 0;
};

class ActiveSlaveSet {
public:
    ActiveSlaveSet();

    void setConfig(const PollConfig& config);
    const PollConfig& getConfig() const;

    void clear();
    void markActive(uint8_t address, bool active = true);
    void recordResponse(uint8_t address);
    void recordTimeout(uint8_t address);

    bool isActive(uint8_t address) const;
    bool hasAnyActive() const;

    // Returns the next address to poll.
    // If activeOnly is enabled and no active slaves are known, it falls back to a full scan.
    uint8_t nextAddress();

private:
    bool isValidAddress(uint8_t address) const;
    uint8_t advanceAddress(uint8_t address) const;

    PollConfig config_{};
    bool active_[kAddressCount]{};
    uint8_t missCount_[kAddressCount]{};
    uint8_t lastAddress_ = kMinSlaveAddress;
};

} // namespace ArduinoRs485
} // namespace DcsBios

#endif
