# Transmitter UI Specification Draft

Created: 2026-06-29

## 1. Target

This draft assumes a Nextion display on the transmitter, connected to the transmitter MCU over UART.

Final values depend on the chosen display model and resolution.

## 2. Main UI states

| State | Meaning | Display behavior |
|---|---|---|
| BOOT | Transmitter is starting | Show radio/controller/display initialization |
| SAFE | Transmitter is connected but vehicle is not armed | Big SAFE status |
| ARMED | Vehicle commands may be active | Big ARMED status plus speed limit |
| LINK_LOST | Radio/receiver not healthy | Full-width warning, block arm indication |
| CONTROLLER_LOST | PS2 controller not healthy | Full-width warning, no drive commands |
| LOW_TX_BAT | Transmitter battery low | Persistent battery warning |
| LOW_RX_BAT | Receiver battery low | Persistent receiver warning |
| FAULT | Receiver fault or local fault | Fault page available, fault banner visible |

## 3. Page: Boot

Objects:

| Name | Type | Purpose |
|---|---|---|
| `p_boot` | page | Boot page |
| `t_title` | text | Project/device name |
| `t_fw` | text | Firmware version/build role |
| `t_radio` | text | Radio init status |
| `t_ps2` | text | Controller init status |
| `t_disp` | text | Display init/status |

## 4. Page: Drive

Objects:

| Name | Type | Purpose |
|---|---|---|
| `p_drive` | page | Main page |
| `t_link` | text | Radio link status |
| `t_arm` | text | SAFE/ARMED |
| `t_brake` | text | Parking brake status |
| `t_tank` | text | Tank mode status |
| `n_spdlim` | number | Speed limit numeric value |
| `j_txbat` | progress | Transmitter battery percent |
| `j_rxbat` | progress | Receiver battery percent |
| `t_fault` | text | Most important active fault |
| `b_telem` | button | Go to telemetry page |
| `b_settings` | button | Go to settings page |

## 5. Page: Telemetry

Objects:

| Name | Type | Purpose |
|---|---|---|
| `p_telem` | page | Telemetry page |
| `n_rssi` | number | RSSI if available |
| `n_pkt` | number | Packet rate or count |
| `n_lx` | number | Left stick X |
| `n_ly` | number | Left stick Y |
| `n_rx` | number | Right stick X |
| `n_ry` | number | Right stick Y |
| `n_vtx` | number | Transmitter battery millivolts or percent |
| `n_vrx1` | number | Receiver battery 1 voltage scaled value |
| `n_vrx2` | number | Receiver battery 2 voltage scaled value |
| `b_drive` | button | Back to drive page |

## 6. Page: Settings

Objects:

| Name | Type | Purpose |
|---|---|---|
| `p_settings` | page | Settings page |
| `h_bright` | slider | Brightness setting |
| `b_spd_up` | button | Increase speed limit |
| `b_spd_dn` | button | Decrease speed limit |
| `b_timeout` | button | Cycle idle timeout |
| `b_drive` | button | Back to drive page |

## 7. Page: Faults

Objects:

| Name | Type | Purpose |
|---|---|---|
| `p_faults` | page | Fault page |
| `t_fault0` | text | Highest priority fault |
| `t_fault1` | text | Secondary fault |
| `t_fault2` | text | Secondary fault |
| `b_ack` | button | Acknowledge displayed fault |
| `b_drive` | button | Back to drive page |

## 8. Event map draft

| Event ID | Event name | Source object | MCU action |
|---|---|---|---|
| 1 | `EVT_GOTO_DRIVE` | `b_drive` | Show drive page |
| 2 | `EVT_GOTO_TELEM` | `b_telem` | Show telemetry page |
| 3 | `EVT_GOTO_SETTINGS` | `b_settings` | Show settings page |
| 4 | `EVT_SPEED_UP` | `b_spd_up` | Clamp and increase speed limit |
| 5 | `EVT_SPEED_DOWN` | `b_spd_dn` | Clamp and decrease speed limit |
| 6 | `EVT_BRIGHTNESS` | `h_bright` | Set brightness after clamp |
| 7 | `EVT_ACK_FAULT` | `b_ack` | Acknowledge visible message only |

## 9. Open decisions

- Exact Nextion model and resolution.
- Whether the display is 5 V only or tolerant of the transmitter power architecture.
- UART baud rate.
- Whether to use an Arduino library or direct serial command helpers.
- Whether touch events use built-in component IDs or custom printable command tokens.
- Whether settings persist in MCU EEPROM, Nextion variables, or not at all for Rev A.

