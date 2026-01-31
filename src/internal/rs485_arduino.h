#ifndef __DCSBIOS_RS485_ARDUINO_H
#define __DCSBIOS_RS485_ARDUINO_H

#include <stdint.h>

// =============================================================================
// Arduino DCS-BIOS RS485 Compatibility Layer
// =============================================================================
//
// This module provides compatibility with Arduino-based DCS-BIOS slaves that
// use the DcsBios::RS485Slave class from the Arduino DCS-BIOS library.
//
// PROTOCOL OVERVIEW:
// -----------------
// The Arduino RS485 protocol uses a polled, binary frame-based design:
//
// 1. MASTER-TO-SLAVE POLL:
//    Frame: [address][msgType][length][data...][checksum]
//    - address: Slave address (1-126)
//    - msgType: 0 for normal poll
//    - length: Number of data bytes (0 for simple poll)
//    - data: DCS-BIOS broadcast data from PC
//    - checksum: Currently 0x00 (reserved for future use)
//
// 2. SLAVE-TO-MASTER RESPONSE:
//    Frame: [address][msgType][length][data...][checksum]
//    - address: Echo of slave's address
//    - msgType: 0 for normal response
//    - length: Number of response bytes
//    - data: Events/updates from slave (e.g., button presses)
//    - checksum: Currently 0x00
//
// 3. TIMING REQUIREMENTS:
//    - Inter-frame gap: 600µs minimum quiet time between frames
//    - Poll interval: Time between successive polls to same/different slaves
//    - Response timeout: Max wait time for slave to respond
//
// ACTIVE-ONLY POLLING:
// -------------------
// To minimize latency, the system tracks which slave addresses have responded
// and only polls those addresses. If a slave times out repeatedly (exceeds
// maxMisses threshold), it's marked inactive and polling is suspended. When
// no active slaves are known, the system falls back to scanning all addresses
// to discover new slaves.
//
// TUNING PARAMETERS:
// -----------------
// All timing and behavior parameters can be overridden at compile time by
// defining the corresponding macro before including this header.
//
// =============================================================================

namespace DcsBios {
namespace ArduinoRs485 {

// Maximum frame payload size (bytes).
// Arduino DCS-BIOS slaves typically use small buffers. 64 bytes provides
// headroom for most use cases without excessive memory consumption.
#ifndef DCSBIOS_RS485_ARDUINO_FRAME_MAX
#define DCSBIOS_RS485_ARDUINO_FRAME_MAX 64
#endif

// Enable active-only polling optimization.
// When 1: Only poll slaves that have previously responded (faster, lower bus traffic)
// When 0: Always scan all addresses 1-126 (slower, guaranteed discovery)
#ifndef DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY
#define DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY 1
#endif

// Number of consecutive poll timeouts before marking a slave as inactive.
// Higher values = more tolerant of intermittent issues but slower to detect failures.
// Lower values = faster failure detection but may spuriously deactivate busy slaves.
#ifndef DCSBIOS_RS485_ARDUINO_MAX_MISSES
#define DCSBIOS_RS485_ARDUINO_MAX_MISSES 3
#endif

// Time interval between successive polls (microseconds).
// This determines the overall polling rate. For N active slaves, each slave
// is polled approximately once every (N × POLL_INTERVAL_US) microseconds.
//
// Current value: 25000µs (25ms) = 40Hz polling rate per slave
//
// TUNING GUIDANCE:
// - Arduino Nano with SSD1306 OLED: 25-50ms works well (20-40Hz)
// - Faster MCUs or simpler sketches: Can go as low as 5-10ms (100-200Hz)
// - Multiple slaves: Increase interval to avoid overwhelming bus
#ifndef DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US
#define DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US 25000
#endif

// Maximum time to wait for a slave response before timing out (microseconds).
// Should be less than POLL_INTERVAL_US to allow time for processing.
// Must account for:
// - RS485 turnaround time (~100µs)
// - Slave processing time (varies by sketch complexity)
// - Frame transmission time at 250000 baud (~50µs per byte)
//
// Current value: 8000µs (8ms)
#ifndef DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US
#define DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US 8000
#endif

// Minimum quiet time required between frames (microseconds).
// The Arduino DCS-BIOS library expects 600µs of bus silence to reset its
// frame receiver state machine. This prevents byte misalignment errors.
#ifndef DCSBIOS_RS485_ARDUINO_INTERFRAME_GAP_US
#define DCSBIOS_RS485_ARDUINO_INTERFRAME_GAP_US 600
#endif

// Valid slave address range: 1-126
// Address 0 is reserved for broadcast messages (master-to-all)
// Address 127 is reserved for future use
constexpr uint8_t kMinSlaveAddress = 1;
constexpr uint8_t kMaxSlaveAddress = 126;
constexpr uint8_t kAddressCount = 127; // Array size to cover 0..126

// Configuration for active-only polling behavior
struct PollConfig {
    // When true, only poll known-active slaves; otherwise poll all addresses.
    // Active-only mode reduces latency and bus traffic when slave count is known.
    bool activeOnly = true;

    // Mark a slave inactive after this many consecutive timeouts.
    // Higher values tolerate intermittent issues; lower values detect failures faster.
    uint8_t maxMisses = 3;
};

// Arduino RS485 binary frame structure
// Wire format: [address][msgType][length][data...][checksum]
//
// MASTER POLL FRAME:
//   address = slave address to poll (1-126) or 0 for broadcast
//   msgType = 0 for normal poll
//   length  = number of DCS-BIOS bytes to broadcast to slave
//   data    = DCS-BIOS data from PC (aircraft state updates)
//   checksum = 0 (reserved for future use)
//
// SLAVE RESPONSE FRAME:
//   address = slave's own address (echoed back)
//   msgType = 0 for normal response
//   length  = number of response bytes
//   data    = events from slave (button presses, switch changes, etc.)
//   checksum = 0 (reserved for future use)
struct Frame {
    uint8_t address = 0;   // Slave address or 0 for broadcast
    uint8_t msgType = 0;   // Message type (currently only 0 is used)
    uint8_t length = 0;    // Payload length in bytes
    uint8_t data[DCSBIOS_RS485_ARDUINO_FRAME_MAX]{}; // Frame payload
    uint8_t checksum = 0;  // Checksum (currently unused, always 0)
};

// State machine for receiving and parsing Arduino RS485 binary frames.
//
// USAGE:
//   FrameReceiver rx;
//   Frame frame;
//   while (uart_has_data()) {
//       uint8_t byte = uart_read();
//       if (rx.pushByte(byte, frame)) {
//           // Complete frame received, process frame.data
//       }
//   }
//
// The receiver automatically handles:
// - Sequential byte assembly into frame structure
// - Frame length validation
// - Automatic state reset after frame completion
// - Overflow protection (resets on invalid length)
//
// NOTE: This receiver expects properly framed data with correct inter-frame
// gaps. Byte misalignment will cause frame errors until the next gap.
class FrameReceiver {
public:
    FrameReceiver();

    // Feed one byte from RS485 RX into the state machine.
    // Returns true when a complete frame has been assembled into outFrame.
    // The receiver automatically resets after returning true.
    bool pushByte(uint8_t byte, Frame& outFrame);

    // Manually reset the receiver to wait for a new frame.
    // Useful for recovering from framing errors or timeouts.
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

// Tracks active slaves and determines the next address to poll.
//
// PURPOSE:
// Maintains a set of "known active" slave addresses to optimize polling.
// Instead of polling all 126 possible addresses, this class tracks which
// slaves have responded and only polls those addresses.
//
// BEHAVIOR:
// 1. Initially, no slaves are marked active (unknown state)
// 2. When a slave responds, it's marked active
// 3. When a slave times out, its miss counter increments
// 4. After maxMisses consecutive timeouts, slave is marked inactive
// 5. nextAddress() returns the next active slave in round-robin order
// 6. If no active slaves exist, falls back to full address scan
//
// BENEFITS:
// - Reduced latency: Only polls known-responsive slaves
// - Lower bus traffic: Skips inactive/non-existent addresses
// - Automatic discovery: Falls back to full scan when needed
// - Fault tolerance: Handles slave disconnection/reconnection gracefully
//
// EXAMPLE:
//   ActiveSlaveSet slaves;
//   slaves.setConfig({.activeOnly = true, .maxMisses = 3});
//
//   while (true) {
//       uint8_t addr = slaves.nextAddress();
//       send_poll(addr);
//       if (wait_for_response(addr)) {
//           slaves.recordResponse(addr);  // Mark active, reset miss counter
//       } else {
//           slaves.recordTimeout(addr);   // Increment miss counter
//       }
//   }
class ActiveSlaveSet {
public:
    ActiveSlaveSet();

    // Set the polling configuration (activeOnly mode, miss threshold)
    void setConfig(const PollConfig& config);
    const PollConfig& getConfig() const;

    // Reset all slaves to inactive state
    void clear();

    // Manually set a slave's active status (useful for pre-configuration)
    void markActive(uint8_t address, bool active = true);

    // Record a successful response from a slave.
    // Marks the slave active and resets its miss counter to 0.
    void recordResponse(uint8_t address);

    // Record a timeout from a slave.
    // Increments miss counter; marks inactive if maxMisses is exceeded.
    void recordTimeout(uint8_t address);

    // Check if a specific slave is currently marked active
    bool isActive(uint8_t address) const;

    // Check if any slaves are currently active
    bool hasAnyActive() const;

    // Returns the next address to poll.
    //
    // ALGORITHM:
    // - If activeOnly is true AND active slaves exist: poll only active slaves (round-robin)
    // - If activeOnly is false OR no active slaves: poll all addresses 1-126 (round-robin)
    //
    // This ensures new slaves can be discovered even when no slaves are active.
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
