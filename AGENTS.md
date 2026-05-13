# AGENTS.md

Operational notes for future Codex/agent work in this repository.

## Project Summary

PS2CarCom is a PlatformIO Arduino project for two Adafruit Feather 32u4/RFM95 boards:

- Transmitter: reads a PS2 controller with PsxNewLib and sends control packets over RadioHead `RH_RF95`.
- Receiver: receives control packets, reads a pedal ADC input, drives a steering PWM output, and controls four BTS7960 motor drivers through an MCP23017 GPIO expander plus software PWM.

This branch has one PlatformIO environment, `feather32u4`. The active role is selected manually in `include/RoleConfig.h`; there are not separate transmitter/receiver environments.

## First Files To Inspect

- `include/RoleConfig.h`: current firmware role. Exactly one of `TRANSMITTER` or `RECEIVER` must be defined.
- `include/RadioConfig.h`: shared RFM95 pins and frequency.
- `lib/Protocol/Protocol.h`: wire protocol, packet sizes, version, flags, and encode/decode logic.
- `src/tx_main.cpp`: PS2 controller handling and transmitter heartbeats.
- `src/rx_main.cpp`: receiver pin map, control decisions, stick mapping, pedal scaling, link-loss failsafe, and steering/motor output.
- `lib/DriveSystem/`, `lib/BTS7960/`, `lib/SoftwarePWM/`: motor driver behavior.
- `lib/AxisMap/` and `lib/Controller/`: stick mapping, button edge tracking, and debug printing.

`lib/Joystick`, `lib/MCPPWM`, `lib/Utils`, and `sketchPad.txt` are currently unused by the active TX/RX firmware. Treat them as legacy or experimental unless a task says otherwise.

## Build And Validation Commands

With a working PlatformIO install:

```sh
pio run -e feather32u4
pio run -e feather32u4 -t upload
pio device monitor -e feather32u4 -b 115200
```

To validate both firmwares, edit `include/RoleConfig.h`, build the receiver, switch the role, then build the transmitter. Do not claim both roles compile unless you actually built both role selections.

This repo currently has no meaningful tests under `test/`. A build is the minimum software validation; hardware behavior needs bench testing with motors made safe.

## Coding Style

- Arduino C++ with PlatformIO private libraries under `lib/<Name>/`.
- Newer code uses two-space indentation, include guards, file-local `static` constants/functions, and Arduino `F()` strings for serial output.
- Keep changes small and explicit. Avoid broad refactors around motor control, protocol, or pin mapping unless the task requires them.
- Prefer shared protocol/types in `lib/Protocol` and `lib/Controller` over duplicating structs in TX/RX code.

## Hardware-Sensitive Areas

Edit these carefully:

- `src/rx_main.cpp` motor pin constants and `MotorPins` source selections (`MCP_PIN` vs `MCU_PIN`).
- `src/rx_main.cpp` `GAS_PEDAL_PIN`, `RX_BATTERY_PIN`, `STEER_PWM`, and battery scaling.
- `include/RadioConfig.h` radio pins/frequency.
- `src/tx_main.cpp` PS2 ATT/CMD/DAT/CLK pins.
- `lib/BTS7960` and `lib/DriveSystem` enable, brake, coast, and PWM behavior.
- `lib/SoftwarePWM`, because the receiver allocates exactly eight channels for four motor drivers.

Always preserve link-loss behavior unless explicitly changing failsafe design: stale link or missing controller should neutralize controller state, disable drive, and command zero motor output.

## Protocol Guidance

`lib/Protocol/Protocol.h` is compatibility-critical. Current protocol version is `1`.

If changing packet contents:

- Update both encoder and decoder paths.
- Update `CONTROL_MESSAGE_SIZE`, `STATUS_MESSAGE_SIZE`, and `MAX_WIRE_PACKET_SIZE`.
- Keep little-endian integer handling consistent.
- Build both transmitter and receiver roles.
- Prefer backward-compatible decoding or a protocol version bump when deployed devices may be mixed.
- Update `README.md` packet tables in the same change.

## Control/Safety Notes

Receiver button actions are edge-triggered. R2 toggles drive enable, Select toggles parking brake, Start toggles tank mode, and D-pad changes maximum speed. On link resync, button edges are reset to avoid replaying held buttons as new presses.

The pedal input scales all motor drive. A zero pedal reading suppresses drive even when the remote commands speed. Steering is currently `analogWrite()` PWM on pin `3`, not Arduino `Servo` pulses.

When testing motor or steering changes, keep the vehicle lifted or motor power disconnected until serial output and PWM behavior are verified.

## Known Uncertainties

- No wiring diagram or hardware revision file is present.
- Receiver battery scaling assumes ADC full scale is `48000` mV, but the sensing circuit is not documented.
- Transmitter battery telemetry is a TODO and returns `0`.
- MCP23017 address is not explicit in code; `mcp.begin_I2C()` uses the library default.
- PS2 button bit ordering is assumed by the enum in `src/rx_main.cpp`; confirm against PsxNewLib before changing button behavior.
- Existing generated VS Code files reference a Windows user path and should not be treated as portable source of truth.
