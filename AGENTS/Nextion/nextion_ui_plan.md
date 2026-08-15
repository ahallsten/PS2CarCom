# Nextion Transmitter UI Plan

Created: 2026-06-29

## 1. Purpose

The transmitter-side Nextion display is a status and configuration interface for the handheld controller. It should not be the safety authority. The controller must remain safe if the display is disconnected, frozen, rebooting, or showing stale data.

## 2. Design principle

Use the Nextion as a local HMI:

- The display owns static layout, pages, fonts, pictures, gauges, and touch widgets.
- The MCU owns vehicle/control truth, radio state, safety state, battery state, and command validation.
- The MCU sends concise state updates to the display over UART.
- The display sends touch events or simple commands back to the MCU.
- The hard status LEDs remain independent of the display.

## 3. Recommended first UI revision

Rev A should be simple and reliable.

Pages:

| Page | Purpose | Must show |
|---|---|---|
| Boot | Startup self-check | Firmware role, display version, radio init, controller init |
| Drive | Main operating screen | Link status, controller status, transmitter battery, vehicle armed state, speed limit, parking brake, tank mode |
| Telemetry | Detailed values | RSSI/SNR if available, packet rate, joystick values, receiver battery voltages, wheel/steering data when implemented |
| Settings | Safe configuration | Speed limit, control mode, screen brightness, timeout settings |
| Faults | Clear fault display | Radio lost, controller lost, low battery, receiver fault, display comm timeout |
| Diagnostics | Bring-up tools | Raw button word, analog stick values, last packet age, loop timing |

Avoid making the first revision visually fancy. Use large text, strong contrast, and few controls.

## 4. Suggested component naming convention

Use predictable object names so firmware and AI can reason about them.

Examples:

| UI item | Nextion name | MCU variable intent |
|---|---|---|
| Main page | `p_drive` | Active drive page |
| Link status text | `t_link` | `LINK OK` / `LINK LOST` |
| Armed state text | `t_arm` | `SAFE` / `ARMED` |
| Parking brake text | `t_brake` | `PARK` / `OFF` |
| Tank mode text | `t_tank` | `TANK` / `NORMAL` |
| Speed limit number | `n_spdlim` | 0 to 250 or percent |
| Transmitter battery gauge | `j_txbat` | 0 to 100 percent |
| Receiver battery gauge | `j_rxbat` | 0 to 100 percent |
| Fault text | `t_fault` | Last/most important fault |
| Brightness slider | `h_bright` | Display brightness control |

Rules:

- Prefix text with `t_`, numbers with `n_`, progress bars/gauges with `j_`, buttons with `b_`, sliders with `h_`, variables with `v_`, pages with `p_`.
- Do not rename objects casually once firmware depends on them.
- Keep a generated object map in the repo.

## 5. Display update strategy

Do not redraw everything every loop. Use a small UI state cache in firmware:

1. Keep the last value sent to each component.
2. Send an update only when the value changes, or at a slow heartbeat interval.
3. Rate-limit noncritical telemetry.
4. Prioritize fault and safety status above cosmetic values.
5. Keep the UART protocol nonblocking.

Suggested update rates:

| Signal | Rate |
|---|---|
| Armed/fault/link state | Immediate on change plus heartbeat |
| Battery percent | 1 to 2 Hz |
| RSSI/packet telemetry | 1 to 5 Hz |
| Joystick diagnostics | 5 to 10 Hz on diagnostics page only |
| Decorative animations | Avoid in Rev A |

## 6. MCU to Nextion command style

A minimal direct-command approach is usually enough:

- `page p_drive`
- `t_link.txt="LINK OK"`
- `t_arm.txt="SAFE"`
- `n_spdlim.val=125`
- `j_txbat.val=82`

Every Nextion command must be terminated by three bytes: `0xFF 0xFF 0xFF`.

For maintainability, do not scatter raw strings across the firmware. Create helper functions such as:

- `nextionSetText(objectName, value)`
- `nextionSetNumber(objectName, value)`
- `nextionSetGauge(objectName, value)`
- `nextionShowPage(pageName)`

## 7. Nextion to MCU event style

For Rev A, keep touch events boring:

- Button press sends an event ID or command token.
- MCU validates whether the action is allowed.
- MCU updates its own state.
- MCU sends updated state back to the display.

Do not let the Nextion directly become the source of truth for arming, speed limit, or power shutdown.

Suggested events:

| Event | Meaning | MCU rule |
|---|---|---|
| `EVT_PAGE_DRIVE` | User selected drive page | Always allowed |
| `EVT_PAGE_TELEM` | User selected telemetry | Always allowed |
| `EVT_SPEED_UP` | Increase speed limit | Clamp to safe max |
| `EVT_SPEED_DOWN` | Decrease speed limit | Clamp to safe min |
| `EVT_BRIGHTNESS` | Brightness changed | Clamp and persist if desired |
| `EVT_ACK_FAULT` | Acknowledge displayed fault | Only clear display acknowledgement, not fault source |
| `EVT_POWER_OFF_REQ` | User requested controller off | Require safe state before power-latch action |

## 8. Assets and visual design

Recommended asset workflow:

- Keep source assets in `ui/assets/source/` as SVG, PNG, or editable files.
- Export display-ready bitmaps to `ui/assets/nextion/`.
- Use consistent dimensions and filenames.
- Keep a screenshot of every page in the repo for AI review and visual diffing.
- Prefer simple shapes and text over large full-screen bitmaps for Rev A.

## 9. Firmware integration tasks

Add a display module to firmware:

- `NextionDisplay.h/.cpp`
- A `DisplayState` struct for values the UI needs.
- A display scheduler that sends updates without blocking radio/control work.
- A parser for touch events and return frames.
- A display-present timeout so the transmitter works without the display.

Add `NEXTION_ENABLED` build flag or compile-time option.

## 10. Safety rules

The display must never be required for safe operation.

- If the Nextion fails, the controller still sends safe radio commands.
- If the MCU fails, the display must not keep the vehicle armed by itself.
- If the radio link is lost, the receiver must fail safe regardless of what the transmitter screen says.
- Use physical status LEDs for power, radio, controller-present, armed/safe, and fault.
- Do not place an on-screen button where accidental touch can arm or power motors without a hardware or firmware interlock.

## 11. AI use cases

AI is useful for:

- Page-flow design.
- Naming conventions.
- Component inventory tables.
- Touch-event map.
- Serial protocol definitions.
- Firmware helper classes.
- Screenshot review.
- Text/icon copywriting.
- Consistency checking between HMI object names and firmware constants.

AI is not reliable for:

- Blindly editing a binary `.hmi` file.
- Verifying that a compiled `.tft` is correct without testing.
- Approving safety behavior without hardware testing.
- Knowing the exact current draw of a display without the selected model datasheet.

