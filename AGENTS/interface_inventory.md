# Interface Inventory Draft

Created: 2026-06-29

This is a draft. Do not freeze connector pinouts until the exact modules and mechanical layout are selected.

## Transmitter interfaces

### Battery / power

| Interface | Pins | Notes |
|---|---:|---|
| 1S Li-ion/LiPo battery | 2 | Use keyed connector, polarity marking, protection strategy. |
| Charge input | TBD | Depends on PowerBoost/module choice. |
| Power switch/latch | TBD | Manual on/off plus firmware shutdown/wake options. |
| Battery sense | 1 ADC + GND ref | Use high-value divider and optional MOSFET disconnect to reduce sleep drain. |

### RFM9x radio

Existing firmware pin intent:

| Signal | MCU pin |
|---|---:|
| RFM95_CS | 8 |
| RFM95_RST | 4 |
| RFM95_INT | 7 |
| SPI SCK/MOSI/MISO | Hardware SPI |

### PS2 controller

Existing firmware pin intent:

| Signal | MCU pin |
|---|---:|
| PS2_ATT | 10 |
| PS2_CMD | 11 |
| PS2_DAT | 12 |
| PS2_CLK | 13 |
| Controller-present debug | A1 |
| Button-press debug | A0 |

Confirm actual PS2 connector pinout, voltage, and whether level shifting is required.

### Nextion display

Typical interface:

| Pin | Signal |
|---:|---|
| 1 | 5 V |
| 2 | GND |
| 3 | MCU TX -> Nextion RX |
| 4 | MCU RX <- Nextion TX |

Confirm display model and current draw.

## Receiver interfaces

### Battery and power

| Interface | Signals | Notes |
|---|---|---|
| Battery input | BAT+, BAT- | Use fuse, keyed connector, high-current rating. |
| Battery midpoint | BAT_MID | Needed if monitoring two 12 V batteries in series. Protect ADC path. |
| Reverse protection | MOSFET path | Low-loss protection target. |
| Master latch | SET, RESET, SENSE | Manual and firmware control. |
| Charger/daughterboard | TBD | Keep modular until charger topology is finalized. |

### RFM9x radio

Existing firmware pin intent:

| Signal | MCU pin |
|---|---:|
| RFM95_CS | 8 |
| RFM95_RST | 4 |
| RFM95_INT | 7 |
| SPI SCK/MOSI/MISO | Hardware SPI |

### BTS7960 motor control (current firmware map)

Five channels: FL, FR, RL, RR, STEER. PWM comes from the PCA9685 (50 Hz), enables come from the MCP23017, and current sense goes to native Feather ADC pins. The authoritative map is `src/rx_main.cpp`, mirrored in `AGENTS/AGENTS.md`.

| Channel | RPWM / LPWM (PCA9685 ch) | R_EN / L_EN (MCP23017 pin) | R_IS / L_IS (Feather pin) |
|---|---|---|---|
| FL | 0 / 1 | 10 / 11 | A0 / A1 |
| FR | 2 / 3 | 12 / 13 | A2 / A3 |
| RL | 4 / 5 | 8 / 9 | A4 / A5 |
| RR | 6 / 7 | 14 / 15 | D6 / D9 ⚠ |
| STEER | 8 / 9 | 0 / 1 | A10 (D10) / A11 (D12) |

⚠ D6 is also `GAS_PEDAL_PIN A7`. D9 carries the Feather's VBAT divider. Resolve before pin lock (`AGENTS/project_context.md` §8).

Planned DF11 harness per driver (Rev A logic PCB, one connector ×5): RPWM, LPWM, R_EN, L_EN, R_IS, L_IS, VCC (logic), GND. Use the 2×5 DF11 so the two spare positions can carry extra GND. Every R_EN/L_EN gets a pull-down on the PCB so the motors default off.

### Driver cooling (planned)

| Interface | Signals | Notes |
|---|---|---|
| Heatsink thermistor | NTC + GND | 10k NTC against the BTS7960 heatsinks. Proposed input: ADS1115 ch3. |
| Fan output | FAN+, FAN− | Low-side MOSFET on the logic PCB, driven from a Feather/MCP pin. Flyback diode. Fan voltage TBD (likely pack voltage). |

### ESC-style motor outputs for future brushless upgrade

Recommended 3-pin header per ESC:

| Pin | Signal | Notes |
|---:|---|---|
| 1 | GND | Common command reference. |
| 2 | +5 V optional | Use DNP jumper if ESC has BEC. Do not short multiple BECs. |
| 3 | PWM | Add pulldown; safe at reset. |

### AS5600 / encoder sensor port draft

Flexible 6-pin DF11-compatible sensor header:

| Pin | Signal | Notes |
|---:|---|---|
| 1 | GND | Ground reference. |
| 2 | VCC_SENSOR | 3.3 V or 5 V; choose per sensor board. |
| 3 | SDA / ENC_A | I2C SDA or quadrature A. |
| 4 | SCL / ENC_B | I2C SCL or quadrature B. |
| 5 | OUT / PWM / INDEX | Optional absolute output or index. |
| 6 | DIR / ID / spare | Direction, ID resistor, or spare GPIO. |

Candidate sensor ports:

| Port | Purpose | Rev A priority |
|---|---|---|
| WHEEL_FL | Front-left wheel speed/position | High |
| WHEEL_FR | Front-right wheel speed/position | High |
| WHEEL_RL | Rear-left wheel speed/position | Medium |
| WHEEL_RR | Rear-right wheel speed/position | Medium |
| PEDAL | Pedal position | High |
| STEER_ANGLE | Steering feedback | High |

### Lighting and accessory outputs

| Output | Driver type | Notes |
|---|---|---|
| Headlights | Protected MOSFET switch | PWM optional. |
| Tail lights | Protected MOSFET switch | PWM optional. |
| Left blinker | Protected MOSFET switch | Firmware pattern. |
| Right blinker | Protected MOSFET switch | Firmware pattern. |
| Horn | MOSFET/relay | Inductive suppression if needed. |
| Stereo/songs | Switched auxiliary rail | Verify voltage/current. |
| Spare AUX 1 | TBD | Include fuse/current limit. |
| Spare AUX 2 | TBD | Include fuse/current limit. |

### Debug/programming

Each MCU needs accessible programming and debug.

| Interface | Signals | Notes |
|---|---|---|
| UART debug | TX, RX, GND | 3.3 V or 5 V logic clearly labeled. |
| ISP/SWD/UPDI/etc. | MCU-specific | Depends on selected MCU. |
| I2C debug | SDA, SCL, GND | Useful during sensor bring-up. |
| Test points | All rails, reset, enables | Must be reachable. |
