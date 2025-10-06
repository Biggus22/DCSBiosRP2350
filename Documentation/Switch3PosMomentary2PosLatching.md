# Switch3PosMomentary2PosLatching Component Documentation

## Overview
The `Switch3PosMomentary2PosLatching` component is designed to interface a 3-position momentary switch with DCS-BIOS as a 2-position latching toggle switch. This is commonly used for cockpit controls that have a spring-return action but need to function as a toggle in the simulation.

## Switch Positions
The component recognizes three physical positions:
- Position 0: Down/First position (pin 0 activated)
- Position 1: Middle/Neutral position (neither pin activated) 
- Position 2: Up/Second position (pin 1 activated)

## Behavior
The switch now behaves as a proper latching toggle:

1. When pressed to position 0 (Down), it sends "0" to DCS-BIOS and latches that state
2. When released back to position 1 (Middle), it remains in the latched state
3. When pressed to position 0 (Down) again, it sends "0" again (allows repeated presses)
4. When pressed to position 2 (Up), it sends "1" to DCS-BIOS and latches that state
5. When released back to position 1 (Middle), it remains in the new latched state
6. When pressed to position 2 (Up) again, it sends "1" again (allows repeated presses)

The key improvement is that you can press the same direction multiple times without needing to press the opposite direction first, but it still maintains the latching behavior where the switch remembers its last activated state.

## Wiring
Connect your 3-position momentary switch as follows:
- Common terminal to ground
- Down position terminal to the first specified pin
- Up position terminal to the second specified pin
- Middle position does not need a connection

## Usage Example
```cpp
const uint8_t switchPins[2] = {8, 9}; // Pin 8 for Down, Pin 9 for Up
DcsBios::Switch3PosMomentary2PosLatching mySwitch("SWITCH_NAME", switchPins);
```

## Constructor Parameters
1. `msg`: The DCS-BIOS message identifier
2. `pins`: An array of two pin numbers (Down pin, Up pin)
3. `debounceDelay`: Optional debounce delay in milliseconds (default: 50ms)

## Notes
- The Up position (position 2) is automatically mapped to value "1" for DCS-BIOS compatibility
- The Down position (position 0) is sent as value "0" 
- The Middle position (position 1) is used internally for debouncing and does not send messages
- The component includes built-in debouncing to prevent false triggers
- The component remembers its last activated state (latching behavior)
- You can press the same direction multiple times without needing to press the opposite direction first