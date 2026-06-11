# AGENTS.md

Operational notes for future Codex/agent work in this repository.

## Project Summary

PS2CarCom is a PlatformIO Arduino project for two Adafruit Feather 32u4/RFM95 boards:

- Transmitter: reads a PS2 controller with PsxNewLib and sends control packets over RadioHead `RH_RF95`.
- Receiver: receives control packets, reads a gas pedal ADC input, drives four wheel BTS7960 channels through a PCA9685 PWM expander, controls BTS7960 enable pins through an MCP23017, runs a closed-loop BTS7960 steering actuator with AS5600 feedback, reads BTS7960 current-sense pins on native analog inputs, and reads pack battery telemetry from an ADS1115.

Roles are selected by PlatformIO environments: `transmitter` and `receiver`. Do not edit `include/RoleConfig.h` to switch roles.

## First Files To Inspect

- `platformio.ini`: role environments, dependencies, build-version script.
- `include/RoleConfig.h`: compile-time guard requiring exactly one role macro.
- `include/RadioConfig.h`: shared RFM95 pins/frequency.
- `lib/Protocol/Protocol.h`: protocol version, packet sizes, flags, and encode/decode offsets.
- `lib/VehicleLayout/VehicleLayout.h`: motor and current-sense array ordering.
- `src/tx_main.cpp`: PS2 handling, control send cadence, status receive logging.
- `src/rx_main.cpp`: receiver hardware map, ADS1115 scaling, PCA9685 setup, steering service command gates, control decisions, link failsafe, telemetry cadence.
- `lib/BTS7960/`, `lib/DriveSystem/`, `lib/Pca9685Pwm/`: four-wheel motor output behavior.
- `lib/SteeringActuator/`: steering BTS7960, AS5600 unwrap, homing, mechanical calibration, EEPROM store, and current protection.
- `lib/CurrentProtection/`: reusable raw-ADC current trip/cooldown helper.
- `src/wheel_encoder_slave_main.cpp` and `lib/WheelEncoderProtocol/`: optional Nano wheel-speed slave using TCA9548A plus four AS5600 sensors.
- `archive/SoftwarePWM/`: old software PWM implementation, preserved but inactive.

## Build And Validation Commands

With a working PlatformIO install:

```sh
pio run -e transmitter
pio run -e receiver
pio run -e transmitter -t upload
pio run -e receiver -t upload
pio device monitor -e transmitter -b 115200
pio device monitor -e receiver -b 115200
```

In this workspace, `/usr/bin/pio` may be broken with the system Python/click version. The working command during this refactor was:

```sh
~/.platformio/penv/bin/pio run -e transmitter
~/.platformio/penv/bin/pio run -e receiver
~/.platformio/penv/bin/pio run -e wheel_encoder_slave
```

There are no meaningful automated tests under `test/`; building both environments is the minimum validation. Hardware behavior needs bench testing with motor power made safe.

## Current Hardware Map

PCA9685 runs at 50 Hz and owns all BTS7960 PWM inputs:

- FL: PCA `0/1` = RPWM/LPWM
- FR: PCA `2/3`
- RL: PCA `4/5`
- RR: PCA `6/7`
- Steering: PCA `8/9`

MCP23017 owns BTS7960 enables:

- FL: REN/LEN `10/11`
- FR: `12/13`
- RL: `8/9`
- RR: `14/15`
- Steering: `0/1`

Native analog current-sense ordering is:

- `FL_L=A1`, `FL_R=A0`
- `FR_L=A3`, `FR_R=A2`
- `RL_L=A5`, `RL_R=A4`
- `RR_L=9`, `RR_R=6`
- `STEER_L=A6`, `STEER_R=A8`

Gas pedal remains `A7`. ADS1115 channel `0` is total pack voltage; channel `1` is midpoint voltage. Steering AS5600 uses default I2C address `0x36`. Steering home switch is MCP `2`, active-low with pullup. MCP23017 `INTA` is expected on MCU pin `0`.

The `Pca9685Pwm` wrapper uses direct `Wire` register writes instead of linking the Adafruit PWM driver object because the receiver firmware is close to the Feather 32u4 flash limit. Keep the Adafruit PCA9685 dependency declared unless intentionally revisiting that tradeoff.

The wheel encoder slave environment targets `nanoatmega328`. It reads four AS5600s at address `0x36` behind a TCA9548A at address `0x70`, channels `0..3` = FL/FR/RL/RR, and exposes a 28-byte I2C slave packet at address `0x12`. Receiver polling is scaffolded behind `RX_ENABLE_WHEEL_ENCODER_SLAVE_POLL` and defaults off to preserve flash.

## Protocol Guidance

`lib/Protocol/Protocol.h` is compatibility-critical. Current protocol version is `5`.

Status packets are 58 bytes and include:

- flags and ACK sequence
- raw/smoothed RSSI
- battery total and midpoint millivolts
- `motorPwm[5]`: FL, FR, RL, RR, STEER
- `currentSense[10]`: FL_L, FL_R, FR_L, FR_R, RL_L, RL_R, RR_L, RR_R, STEER_L, STEER_R
- steering AS5600 raw angle, relative position counts, target counts, state, flags, fault flags, and max steering current raw

When changing protocol fields:

- Update both encoder and decoder.
- Update packet size constants and documented offsets.
- Build both transmitter and receiver.
- Update `README.md` packet tables.
- Prefer a version bump unless backward-compatible decoding is deliberately implemented.

## Safety-Sensitive Behavior

Preserve link-loss behavior unless a task explicitly changes failsafe design. Stale link or missing controller should neutralize controller state, reset button-edge state, disable drive, and command zero output.

`DriveSystem` gates only the four wheel BTS7960 outputs. `SteeringActuator` owns the steering BTS7960 and must coast on stale link, missing controller, disabled drive, parking brake, missing AS5600, invalid calibration, missing homing, and current-protection trips.

Steering service commands are held-button combos: L3+R3+Select for 2 seconds homes steering; L3+R3+Start for 3 seconds runs full mechanical-limit calibration and saves EEPROM. Select/Start edge actions are suppressed while L3+R3 are held so these combos do not accidentally toggle parking brake/tank mode.

EEPROM calibration at address `0` uses magic `STR1`, version `1`, struct size, count limits, raw angle at save, flags, and Fletcher16 checksum. Treat layout changes as protocol-like: bump version or reject old data.

Motor direction changes use a short break-before-make delay in `BTS7960`. Be careful editing `drive()`, `brake()`, `coast()`, or PCA9685 channel writes.

## Coding Style

- Arduino C++ with two-space indentation.
- Prefer file-local `static` constants/functions for receiver pin maps and scaling constants.
- Use shared layout/protocol constants instead of duplicated magic array sizes.
- Keep abstractions thin. `Pca9685Pwm` is intentionally a small adapter around PCA9685 register writes.
- Receiver flash is tight on Feather 32u4. `RX_PRINT_DECODED_CONTROL` and `RX_PRINT_DRIVE_DEBUG` default to `0`; do not add large serial formatters without checking receiver program size.
- Add comments only for hardware assumptions, protocol scaling, or non-obvious timing/safety behavior.

## High-Caution Changes

Use extra care for:

- `src/rx_main.cpp` pin/channel maps.
- ADS1115 divider constants and gain assumptions.
- `lib/Protocol/Protocol.h` offsets, sizes, and version.
- `lib/WheelEncoderProtocol/WheelEncoderProtocol.h` wheel packet size/checksum if enabling wheel-speed telemetry.
- `lib/BTS7960` direction/brake/coast behavior.
- `lib/DriveSystem` enable, parking brake, and telemetry command recording.
- `lib/SteeringActuator` homing/calibration/current-protection state transitions.
- LoRa modem config or timing constants.

Do not move `archive/SoftwarePWM/` back into active `lib/` unless intentionally reviving that implementation. It was archived so PlatformIO will not use it for receiver motor control.

## Known Uncertainties

- No wiring diagram or hardware revision file is present.
- Battery divider ratios are placeholders and must be calibrated.
- Current sense telemetry is raw ADC counts; BTS7960 current-to-ADC scaling is not documented.
- Feather 32u4 analog availability for `A6` and `A8` should be confirmed on the exact board/wiring.
- Steering current thresholds, homing direction, motor direction, travel limits, and software margins are placeholders that require bench calibration.
- AS5600 raw counts are `0..4095`; one steering output revolution is assumed to be `8192` unwrapped counts because the reference actuator sketch used a 2:1 encoder-to-output ratio.
- The Nano wheel slave uses the same `Wire` bus as an I2C slave to the receiver and as I2C master to the TCA/AS5600 chain. Reads are scheduled outside callbacks, but bench-test this carefully; use separate buses or a private software bus if it locks up.
- Transmitter battery telemetry is a TODO and reports `0`.
- PS2 button bit ordering is assumed by the enum in `src/rx_main.cpp`; confirm against PsxNewLib before changing button behavior.
