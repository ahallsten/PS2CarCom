# PS2CarCom

PS2CarCom is a PlatformIO/Arduino firmware project for a two-microcontroller RC-style vehicle control system. One Adafruit Feather 32u4/RFM95 board acts as the transmitter and reads a PlayStation 2 controller. A second Feather 32u4/RFM95 board acts as the receiver on the vehicle, receives LoRa packets, and drives steering plus four BTS7960 motor drivers.

The project builds the transmitter and receiver as separate PlatformIO environments. Use `-e transmitter` for the remote and `-e receiver` for the vehicle-side controller.

## Current Architecture

```text
PS2 controller
    |
    | bit-banged PsxNewLib interface
    v
Transmitter Feather 32u4 + RFM95
    |
    | RadioHead RH_RF95 LoRa packets
    v
Receiver Feather 32u4 + RFM95
    |
    | MCP23017 GPIO expander + software PWM + direct MCU pins
    v
BTS7960 motor drivers, steering PWM, pedal input, battery sense
```

The active source files are:

| Path | Purpose |
| --- | --- |
| `include/RoleConfig.h` | Compile-time guard that requires exactly one role macro. Roles are selected by PlatformIO environments. |
| `include/RadioConfig.h` | Shared RFM95 chip-select, reset, interrupt pins, and RF frequency. |
| `src/main.cpp` | Compile-time guard that rejects invalid role selection. |
| `src/tx_main.cpp` | Transmitter firmware, compiled only when `TRANSMITTER` is defined. |
| `src/rx_main.cpp` | Receiver firmware, compiled only when `RECEIVER` is defined. |
| `lib/Protocol/Protocol.h` | Shared packet format and encode/decode helpers. |
| `lib/Controller/` | Shared controller state, button edge tracking, and serial debug printing. |
| `lib/AxisMap/` | Analog stick dead-zone and signed PWM mapping helpers. |
| `lib/DriveSystem/` | Four-motor drive abstraction around BTS7960 drivers. |
| `lib/BTS7960/` | Low-level BTS7960 motor driver wrapper. |
| `lib/SoftwarePWM/` | Software PWM channels for MCU or MCP23017 pins. |

`lib/Joystick`, `lib/MCPPWM`, `lib/Utils`, and `sketchPad.txt` are present but are not referenced by the active transmitter or receiver firmware on this branch.

## Toolchain

The project uses PlatformIO:

```ini
[env]
platform = atmelavr
board = feather32u4
framework = arduino
monitor_speed = 115200

[env:transmitter]
build_flags = -DTRANSMITTER

[env:receiver]
build_flags = -DRECEIVER
```

Declared library dependencies in `platformio.ini` are:

- Adafruit SleepyDog Library
- RadioHead
- PsxNewLib
- LibPrintf
- Adafruit MCP23017 Arduino Library

Active code directly uses RadioHead, PsxNewLib, DigitalIO, SPI, Wire, and the Adafruit MCP23X17 API. Some declared dependencies may be historical or indirectly used.

## Build, Upload, And Monitor

Install PlatformIO through the VS Code extension or the PlatformIO CLI. Select the firmware role with the PlatformIO environment name.

Build the transmitter:

```sh
pio run -e transmitter
```

Build the receiver:

```sh
pio run -e receiver
```

Upload the transmitter:

```sh
pio run -e transmitter -t upload
```

Upload the receiver:

```sh
pio run -e receiver -t upload
```

Open the serial monitor at the configured speed:

```sh
pio device monitor -e transmitter -b 115200
pio device monitor -e receiver -b 115200
```

`include/RoleConfig.h` should not be edited to choose a role. It only checks that exactly one role macro was provided by the selected PlatformIO environment.

## Radio Configuration

`include/RadioConfig.h` defines the shared RFM95 configuration:

| Setting | Value in code |
| --- | --- |
| `RFM95_CS` | `8` |
| `RFM95_RST` | `4` |
| `RFM95_INT` | `7` |
| `RF95_FREQ` | `915.0f` |

Both boards must use matching radio frequency and packet protocol. If you change the region, hardware, or radio pins, update this file and verify that the selected frequency is legal for your location and radio module.

## Transmitter Firmware

`src/tx_main.cpp` reads a PS2 controller through `PsxControllerBitBang` using these pins:

| PS2 signal | Pin |
| --- | --- |
| ATT | `10` |
| CMD | `11` |
| DAT | `12` |
| CLK | `13` |

The transmitter:

1. Initializes the RFM95 radio.
2. Searches for a PS2 controller.
3. Enters PS2 config mode and enables analog sticks/buttons when possible.
4. Reads the controller button word and left/right analog sticks.
5. Sends a control packet when the controller state changes or at least every 50 ms.
6. Receives receiver status packets and tracks receiver link health, RSSI, receiver battery millivolts, and motor percentages.
7. Sends its own status heartbeat every 100 ms.

`readTransmitterBatteryMilliVolts()` is currently a TODO and returns `0`, so transmitter battery telemetry is not implemented yet.

## Receiver Firmware

`src/rx_main.cpp` receives control packets and applies vehicle outputs. It uses:

- `Adafruit_MCP23X17 mcp` with `mcp.begin_I2C()` and the library default I2C address.
- `SoftwarePWMX pwmx(8, &mcp)` for eight software PWM channels, two per motor driver.
- Four `BTS7960` objects wrapped by `DriveSystem`.
- `analogRead(GAS_PEDAL_PIN)` for the pedal input.
- `analogRead(RX_BATTERY_PIN)` for receiver battery telemetry.
- `analogWrite(STEER_PWM, value)` for steering output.

Important receiver pins and defaults:

| Function | Pin/source in code |
| --- | --- |
| Gas pedal | `A7` unless `GAS_PEDAL_PIN` is defined before this file |
| Receiver battery sense | `A6` unless `RX_BATTERY_PIN` is defined before this file |
| Steering PWM | MCU pin `3` |

Motor pin definitions as currently wired in code:

| Motor | RPWM | LPWM | REN | LEN | RIS | LIS |
| --- | --- | --- | --- | --- | --- | --- |
| Front left | MCP `12` | MCP `13` | MCP `0` | MCP `1` | MCU `A0` | MCU `A1` |
| Front right | MCP `11` | MCP `10` | MCP `2` | MCP `3` | MCU `A2` | MCU `A3` |
| Rear left | MCP `7` | MCP `6` | MCP `4` | MCP `5` | MCU `A4` | MCU `A5` |
| Rear right | MCP `5` | MCP `4` | MCP `6` | MCP `7` | MCU `6` | MCU `9` |

The code treats MCP values as `Adafruit_MCP23X17` pin indexes. Confirm the physical package pin mapping against your MCP23017 board before wiring.

## Controls

The receiver maps the PS2 button word with this enum:

| Bit | Button | Receiver action on rising edge |
| --- | --- | --- |
| 0 | Select | Toggle parking brake |
| 1 | L3 | No active action |
| 2 | R3 | No active action |
| 3 | Start | Toggle tank mode |
| 4 | Up | Increase maximum speed by 25, capped at 255 |
| 5 | Right | Set maximum speed to 255 |
| 6 | Down | Decrease maximum speed by 25, floored at 25 |
| 7 | Left | Set maximum speed to 25 |
| 8 | L2 | No active action |
| 9 | R2 | Toggle drive enabled |
| 10 | L1 | No active action |
| 11 | R1 | No active action |
| 12 | Triangle | No active action |
| 13 | Circle | No active action |
| 14 | Cross | No active action |
| 15 | Square | No active action |

Analog stick handling:

- Left stick Y controls the left-side drive command.
- In normal mode, left stick Y also controls the right-side drive command.
- In tank mode, right stick Y controls the right-side drive command.
- Right stick X controls steering PWM.
- Left stick X is mapped and printed in debug output, but it is not currently used for drive mixing.

The gas pedal scales motor output from `0` to `255`. A pedal reading of `0` suppresses drive output even if the remote is commanding speed. A full-scale pedal reading allows the full clamped remote command.

`maximumSpeed` starts at `255`; the minimum speed clamp is `25`. Nonzero stick commands are constrained between the current minimum and maximum speed before pedal scaling.

## Communication Protocol

The packet format is implemented in `lib/Protocol/Protocol.h`. Both packet types start with:

| Byte | Meaning |
| --- | --- |
| `0` | Protocol version, currently `1` |
| `1` | Packet type: `1` for control, `2` for status |

Control packet, 10 bytes total:

| Byte(s) | Meaning |
| --- | --- |
| `0` | Protocol version |
| `1` | `PACKET_CONTROL` |
| `2` | Sequence number |
| `3` | Flags, bit `0x01` means controller present |
| `4..5` | `buttonWord`, little-endian |
| `6` | Left stick X |
| `7` | Left stick Y |
| `8` | Right stick X |
| `9` | Right stick Y |

Status packet, 13 bytes total:

| Byte(s) | Meaning |
| --- | --- |
| `0` | Protocol version |
| `1` | `PACKET_STATUS` |
| `2` | Flags: bit `0x01` link OK, bit `0x02` controller present |
| `3..4` | Raw RSSI, signed 16-bit little-endian |
| `5..6` | Smoothed RSSI, signed 16-bit little-endian |
| `7..8` | Battery millivolts, unsigned 16-bit little-endian |
| `9..12` | Motor command percentages in front-left, front-right, rear-left, rear-right order |

The transmitter sends control packets immediately on state changes, otherwise at least every 50 ms. Both sides send status heartbeats every 100 ms. The receiver considers the control link stale after 500 ms without a fresh control packet from a present controller.

## Failsafe And Safety Behavior

The receiver has important software failsafes:

- If the control link is stale or the transmitter reports no controller, the receiver resets the controller state to neutral.
- Link loss clears button edge state, marks button resync needed, turns `driveEnabled` off, disables the drive system, and commands zero drive.
- Motors only run when the link is fresh, the remote controller is present, `driveEnabled` has been toggled on with R2, the parking brake is off, and the pedal input is nonzero.
- Parking brake causes `DriveSystem` to call its brake path and record zero motor commands.

This code can move a real vehicle. Test changes with motor power disabled, the vehicle lifted, or motor drivers disconnected until the behavior is understood.

## Calibration And Tuning

Places to tune behavior:

- Radio pins/frequency: `include/RadioConfig.h`
- Role build flags/environments: `platformio.ini`
- PS2 pins: `src/tx_main.cpp`
- Motor, steering, pedal, and battery pins: `src/rx_main.cpp`
- Stick dead zones and stick-to-PWM mapping: the `AxisMap` constants in `src/rx_main.cpp`
- Minimum/maximum speed behavior: `kMinimumSpeed`, `maximumSpeed`, and `clampSpeed()` in `src/rx_main.cpp`
- Link and heartbeat timing: `CONTROL_PERIOD_MS`, `HEARTBEAT_PERIOD_MS`, and `LINK_TIMEOUT_MS`

Receiver battery telemetry currently assumes a full-scale ADC reading corresponds to `48000` mV:

```cpp
(raw * 48000UL) / 1023UL
```

The resistor divider or sensing circuit that justifies this scale is not documented in the repo. Calibrate this before relying on battery telemetry.

The steering output is `analogWrite()` PWM on pin `3`; the active code does not use the Arduino `Servo` library or servo pulse widths. Confirm that the steering hardware expects this signal before connecting it.

## Troubleshooting

- `RFM95 initialization failed`: check RFM95 power, SPI wiring, CS/reset/interrupt pins in `include/RadioConfig.h`, and that the board matches the Feather 32u4 PlatformIO environments.
- `mcp begin error`: check MCP23017 power, SDA/SCL wiring, and I2C address. The code does not pass an explicit address to `mcp.begin_I2C()`.
- Controller repeatedly appears and disappears: check PS2 wiring, controller power level, and the ATT/CMD/DAT/CLK pins in `src/tx_main.cpp`.
- Receiver link never becomes OK: confirm both boards are built from the same protocol version, have the same radio frequency, and that one board is built as transmitter while the other is built as receiver.
- Motors do not move: verify R2 has toggled drive enabled, Select has not enabled the parking brake, the pedal input is above zero, the receiver has a fresh link, and the BTS7960 enable/PWM pins match the wiring.
- Steering does not behave like a hobby servo: the code uses `analogWrite()`, not servo pulses. Confirm the steering actuator interface.

## Known Limitations And Open Questions

- Transmitter and receiver are separate PlatformIO environments, but upload ports are not pinned in `platformio.ini`; select the correct connected board when uploading.
- There are no project tests in `test/` beyond the default PlatformIO README.
- Transmitter battery measurement is stubbed and always reports `0` mV.
- Receiver battery scaling is hard-coded but the sensing circuit is not documented.
- The exact PS2 button bit ordering is assumed by the receiver enum and should be confirmed against `PsxNewLib` and the actual controller.
- `Joystick`, `MCPPWM`, `Utils`, and `sketchPad.txt` look like older experiments or unused helpers.
- No wiring diagram, enclosure notes, RF module variant notes, or hardware revision information are included.
- The protocol has a version byte and strict packet lengths, but no application-level checksum, authentication, or compatibility shim for future protocol versions.
