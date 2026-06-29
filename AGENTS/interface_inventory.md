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

### Existing BTS7960-style motor control

Current firmware defines four motor channels.

| Channel | RPWM | LPWM | R_EN | L_EN | R_IS | L_IS |
|---|---:|---:|---:|---:|---:|---:|
| Front left | 12 | 13 | MCP/B0? | MCP/B1? | A0 | A1 |
| Front right | 11 | 10 | MCP/B2? | MCP/B3? | A2 | A3 |
| Rear left | 7? | 6? | MCP/B4? | MCP/B5? | A4 | A5 |
| Rear right | 5? | 4? | MCP/B6? | MCP/B7? | 6 | 9 |

Caution: comments and implementation need review. Some signals are commented as MCP23017 pins while the `BTS7960` class uses Arduino `digitalWrite()` and `analogWrite()` directly. Resolve before schematic pin lock.

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
