# PS2CarCom

PS2CarCom is a PlatformIO/Arduino firmware project for a two-microcontroller RC-style vehicle control system. One Adafruit Feather 32u4/RFM95 board is the transmitter and reads a PlayStation 2 controller. A second Feather 32u4/RFM95 board is the vehicle receiver and drives BTS7960 motor controllers over LoRa commands.

The project builds the two roles as separate PlatformIO environments:

```sh
pio run -e transmitter
pio run -e receiver
```

## Architecture

```text
PS2 controller
    |
    | PsxNewLib bit-banged controller bus
    v
Transmitter Feather 32u4 + RFM95
    |
    | RadioHead RH_RF95 LoRa packets
    v
Receiver Feather 32u4 + RFM95
    |
    | I2C: MCP23017 enables, PCA9685 PWM, ADS1115 battery ADC
    v
Five BTS7960 drivers: FL, FR, RL, RR, steering
```

The receiver uses the PCA9685 for all BTS7960 RPWM/LPWM inputs at 50 Hz. The MCP23017 is still used for BTS7960 enable pins. Steering is treated as another signed BTS7960 motor channel, not as a hobby servo or `analogWrite()` output.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `platformio.ini` | PlatformIO environments and dependencies. |
| `include/RoleConfig.h` | Compile-time role guard. Do not edit this to switch roles. |
| `include/RadioConfig.h` | Shared RFM95 pins and frequency. |
| `src/tx_main.cpp` | Transmitter firmware. |
| `src/rx_main.cpp` | Receiver firmware, hardware map, control decisions, telemetry sampling. |
| `lib/Protocol/Protocol.h` | Shared control/status packet format. |
| `lib/VehicleLayout/VehicleLayout.h` | Shared motor/current-sense ordering constants. |
| `lib/BTS7960/` | Low-level BTS7960 wrapper. |
| `lib/DriveSystem/` | Five-driver vehicle output coordinator. |
| `lib/Pca9685Pwm/` | Thin PCA9685 PWM adapter. |
| `lib/AxisMap/` | Stick dead-zone and PWM mapping helpers. |
| `lib/Controller/` | Controller state, button edges, and serial debug helpers. |
| `archive/SoftwarePWM/` | Previous software PWM implementation, preserved but inactive. |

## Toolchain And Dependencies

The active target is `atmelavr`, board `feather32u4`, framework `arduino`.

Declared library dependencies:

- Adafruit SleepyDog Library
- RadioHead
- PsxNewLib
- LibPrintf
- Adafruit MCP23017 Arduino Library
- Adafruit PWM Servo Driver Library
- Adafruit ADS1X15

Build, upload, and monitor:

```sh
pio run -e transmitter
pio run -e receiver
pio run -e transmitter -t upload
pio run -e receiver -t upload
pio device monitor -e transmitter -b 115200
pio device monitor -e receiver -b 115200
```

If multiple boards are attached, choose the upload port explicitly in PlatformIO or add a temporary `upload_port` locally.

## Radio Configuration

`include/RadioConfig.h` currently defines:

| Setting | Value |
| --- | --- |
| `RFM95_CS` | `8` |
| `RFM95_RST` | `4` |
| `RFM95_INT` | `7` |
| `RF95_FREQ` | `915.0f` |

Both firmwares use `RH_RF95::Bw500Cr45Sf128`. The comment in code estimates about 11 ms on-air time per packet with this modem config. Keep the transmitter and receiver radio settings matched.

## Transmitter Firmware

`src/tx_main.cpp` reads the PS2 controller through `PsxControllerBitBang`:

| PS2 signal | Pin |
| --- | --- |
| ATT | `10` |
| CMD | `11` |
| DAT | `12` |
| CLK | `13` |

The transmitter:

1. Initializes the RFM95 radio.
2. Finds and configures the PS2 controller.
3. Reads buttons and analog sticks.
4. Sends control packets immediately when state changes or at least every 50 ms.
5. Receives receiver telemetry and prints ACK/status lines.
6. Sends transmitter status heartbeats every 100 ms.

Transmitter battery measurement is still a TODO and currently reports `0` mV.

## Receiver Firmware

The receiver:

1. Receives control packets from the transmitter.
2. Applies link-loss and controller-present failsafes.
3. Converts stick values into signed drive and steering commands.
4. Scales drive motor output by the gas pedal on native analog pin `A7`.
5. Sends five BTS7960 signed PWM commands through the PCA9685.
6. Samples current sense on native analog inputs.
7. Samples pack total and midpoint voltage through ADS1115 channels.
8. Sends status telemetry every 250 ms.

### Receiver Hardware Map

PCA9685 PWM channel mapping:

| Output | RPWM | LPWM |
| --- | --- | --- |
| Front left | PCA `0` | PCA `1` |
| Front right | PCA `2` | PCA `3` |
| Rear left | PCA `4` | PCA `5` |
| Rear right | PCA `6` | PCA `7` |
| Steering | PCA `8` | PCA `9` |

MCP23017 enable mapping:

| Output | REN | LEN |
| --- | --- | --- |
| Front left | MCP `10` | MCP `11` |
| Front right | MCP `12` | MCP `13` |
| Rear left | MCP `8` | MCP `9` |
| Rear right | MCP `14` | MCP `15` |
| Steering | MCP `0` | MCP `1` |

Current-sense mapping, reported as raw native ADC counts:

| Output | L_IS | R_IS |
| --- | --- | --- |
| Front left | `A1` | `A0` |
| Front right | `A3` | `A2` |
| Rear left | `A5` | `A4` |
| Rear right | `9` | `6` |
| Steering | `A6` | `A8` |

Other receiver inputs:

| Function | Pin/channel |
| --- | --- |
| Gas pedal | `A7` |
| Battery total voltage | ADS1115 channel `0` |
| Battery midpoint voltage | ADS1115 channel `1` |
| RX packet LED | MCU pin `13` |

The code assumes the Feather 32u4 board variant exposes `A6` and `A8`. Confirm this against the exact Feather and wiring before depending on steering current telemetry.

### Battery Scaling

ADS1115 battery scaling is integer fixed-point. The code configures `GAIN_ONE`, treated as 125 uV/count, and applies divider ratios:

```cpp
BATTERY_TOTAL_DIVIDER_NUM / BATTERY_TOTAL_DIVIDER_DEN
BATTERY_MIDPOINT_DIVIDER_NUM / BATTERY_MIDPOINT_DIVIDER_DEN
```

Both ratios currently default to `11:1` placeholders. Replace these constants in `src/rx_main.cpp` with the measured resistor divider ratios before relying on battery voltage telemetry. Telemetry reports total and midpoint millivolts; individual batteries can be derived as:

```text
battery_1_mv = midpoint_mv
battery_2_mv = total_mv - midpoint_mv
```

## Controls

Receiver button actions are edge-triggered:

| Bit | Button | Action |
| --- | --- | --- |
| `0` | Select | Toggle parking brake |
| `3` | Start | Toggle tank mode |
| `4` | Up | Increase maximum speed by 25, capped at 255 |
| `5` | Right | Set maximum speed to 255 |
| `6` | Down | Decrease maximum speed by 25, floored at 25 |
| `7` | Left | Set maximum speed to 25 |
| `9` | R2 | Toggle drive enabled |

Analog stick behavior:

- Left stick Y controls left-side drive.
- In normal mode, left stick Y also controls right-side drive.
- In tank mode, right stick Y controls right-side drive.
- Right stick X controls the steering BTS7960 command.
- Left stick X is mapped and printed for debug, but is not currently used for mixing.

The gas pedal scales the four drive motors from zero to full command. Steering is not scaled by the gas pedal, but it is still gated by link health, drive enable, and parking brake through `DriveSystem`.

## Protocol

`lib/Protocol/Protocol.h` defines protocol version `4`. Both packet types begin with:

| Byte | Meaning |
| --- | --- |
| `0` | Protocol version |
| `1` | Packet type: `1` control, `2` status |

Control packet, 10 bytes:

| Byte(s) | Meaning |
| --- | --- |
| `0` | Protocol version |
| `1` | `PACKET_CONTROL` |
| `2` | Sequence number |
| `3` | Flags, bit `0x01` means controller present |
| `4..5` | Button word, little-endian |
| `6` | Left stick X |
| `7` | Left stick Y |
| `8` | Right stick X |
| `9` | Right stick Y |

Status packet, 42 bytes:

| Byte(s) | Meaning |
| --- | --- |
| `0` | Protocol version |
| `1` | `PACKET_STATUS` |
| `2` | Flags: link OK, controller present, parking brake, drive enabled, tank mode |
| `3` | ACK echo: last control sequence decoded by receiver |
| `4..5` | Raw RSSI, signed i16 little-endian |
| `6..7` | Smoothed RSSI, signed i16 little-endian |
| `8..9` | Battery total millivolts, u16 little-endian |
| `10..11` | Battery midpoint millivolts, u16 little-endian |
| `12..21` | `motorPwm[5]`, signed i16: FL, FR, RL, RR, STEER |
| `22..41` | `currentSense[10]`, raw u16 ADC counts: FL_L, FL_R, FR_L, FR_R, RL_L, RL_R, RR_L, RR_R, STEER_L, STEER_R |

The transmitter and receiver must be built from matching protocol code. There is no compatibility shim for older v2/v3 packets.

## Timing And Failsafes

- Transmitter control packets: on change or at least every 50 ms.
- Transmitter status packets: every 100 ms.
- Receiver status packets: every 250 ms.
- Link timeout: 500 ms.

On stale link or missing controller, the receiver resets controller state, clears button edge state, disables drive, and commands zero output. The receive LED on pin 13 stays on for 100 ms per received packet with a 100 ms off gap so activity is visible by eye.

LoRa sends still block in `rfm.waitPacketSent()`. The PCA9685 removes software PWM timing pressure, but radio transmission remains the largest intentional blocking operation in the loop.

## Troubleshooting

- `RFM95 initialization failed`: check radio power, SPI wiring, and `include/RadioConfig.h`.
- `mcp begin error`: check MCP23017 power, SDA/SCL wiring, and default I2C address.
- ADS1115 warning at boot: battery telemetry will report `0` mV until ADS1115 wiring/address is fixed.
- Controller repeatedly appears/disappears: check PS2 wiring and ATT/CMD/DAT/CLK pins.
- Receiver link never becomes OK: confirm matching protocol version, frequency, modem config, and opposite firmware roles.
- Motors do not move: confirm R2 drive enable, Select parking brake state, gas pedal reading, fresh link, PCA9685 channels, MCP enable wiring, and BTS7960 power.
- Steering does not move: confirm the steering BTS7960 is wired to PCA channels `8/9`, MCP enables `0/1`, and the receiver link/drive gates are active.

## Known Limitations

- No wiring diagram or hardware revision file is present.
- ADS1115 and PCA9685 use library default I2C addresses.
- Battery divider ratios are placeholders and require calibration.
- Current telemetry is raw ADC counts, not amps. The BTS7960 board current-sense scaling is not documented here.
- Transmitter battery telemetry is stubbed.
- Upload ports are not pinned in `platformio.ini`.
- There are no automated tests beyond successful PlatformIO builds.
