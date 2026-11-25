# Project Context and Design Intent

Consolidated 2026-11-25 from the external ChatGPT project handoff and the current repository state. This file holds design intent, requirements, decisions, and open work that are **not** already covered elsewhere. For the current firmware/hardware map, read `AGENTS/AGENTS.md` first. For PCB architecture, read `AGENTS/pcb_design_plan.md`.

Rule: the code in this repo is the source of truth. Anything below marked "intent" or "open" is not implemented until the code says so.

## 1. End-state goal

A robust embedded controller for a kid's Power Wheels-style Jeep, with:

- PS2 remote control over a 915 MHz RFM95 link.
- Local child driving (gas pedal, eventually the steering wheel).
- 4 independently driven wheels, plus a steering actuator.
- Closed-loop steering, plus wheel speed/position feedback from AS5600 encoders.
- Safety interlocks, telemetry, a Nextion UI on the transmitter, custom PCBs, and low-drain auto power-off.

## 2. Status of historical defects (checked against code 2026-11-25)

Older handoff notes list defects from a pre-refactor `main.cpp`. Current status:

| Historical issue | Status | Where |
|---|---|---|
| TX only sent on state change, so no heartbeat | **Fixed.** TX sends a control heartbeat every 100 ms; RX link timeout is 500 ms. | `src/tx_main.cpp`, `src/rx_main.cpp` |
| No RX packet-age failsafe | **Fixed.** `hasFreshControlLink()` neutralizes state, disables drive, and commands zero. | `controlsDecision()` |
| Link recovery auto-arms | **Fixed.** Link loss forces `driveEnabled = false`; the user must re-enable with R2. | `controlsDecision()` |
| `previousButtonWord` local/uninitialized; toggles fired on both edges | **Fixed.** `ButtonEdgeTracker` acts on the rising edge only, with a 200 ms per-button debounce. | `lib/Controller/` |
| BTS7960 reversal without break-before-make | **Fixed.** 200 µs dead-time. | `lib/BTS7960/BTS7960.cpp` |
| Native GPIO vs MCP23017 pin numbers mixed | **Fixed.** `PinSource::MCU_PIN` / `MCP_PIN`; PWM is entirely on the PCA9685. | `lib/BTS7960/`, `src/rx_main.cpp` |
| Parking brake / disable overridden by later `drive()` calls | **Fixed.** `DriveSystem` gates every output at the last stage. | `lib/DriveSystem/` |
| Unversioned raw-struct protocol | **Fixed.** Versioned (v4) encode/decode with sequence/ACK and a status packet. | `lib/Protocol/Protocol.h` |
| Speed limit not applied | **Open.** `clampSpeed()` exists but its call is commented out. `applyDrive()` gets raw `leftYPWM`/`rightYPWM`. | `controlsDecision()` |
| Tank vs normal mixing | **Open.** The `tankMode` toggle exists but isn't used. RX always drives left-Y on the left wheels and right-Y on the right wheels (effectively tank). | `controlsDecision()` |
| Gas pedal not applied | **Open.** `pedalDriveValue` is read but unused. Remote/local authority is undefined (see §5). | `controlsDecision()` |
| Watchdog | **Open.** SleepyDog is a dependency, but no `Watchdog.enable()` exists in either role. | both roles |
| Init failure loops | **Open (review).** `while(1)` on MCP/PCA/RFM init failure. Confirm that outputs are safe in each loop, and feed or disable the watchdog once it's added. | `setup()` |
| Axis-mapping midpoint reversal/decay bug seen earlier | **Watch.** Trace the full chain (raw → normalize → deadband → mix → limit → gate) before changing anything. Don't stack more remaps. | `lib/AxisMap/`, `controlsDecision()` |

## 3. Target command pipeline and state machine (intent)

Keep commands signed internally. Convert to unsigned only at the driver.

```text
Raw stick → calibrated signed normalize → deadband → mode mixer (normal/tank)
  → speed limit → optional slew limit → per-wheel inversion → safety gate → BTS7960
```

Make vehicle states explicit, not scattered booleans:

```text
BOOT → SELF_TEST → DISARMED
DISARMED → ARMED_REMOTE  (explicit arm + fresh link + controls neutral)
ARMED_REMOTE ↔ ARMED_LOCAL  (only if explicitly allowed)
any critical fault / link timeout → FAULT → deliberate recovery → DISARMED
```

Orthogonal flags: parking brake, tank mode, speed limit, accessory states. Critical conditions override every actuator command. Separate policy from hardware access: the safety logic decides, and `BTS7960` never decides whether driving is allowed.

When changing safety behavior, document in the PR: the failure mode, the safe state, how it's detected, and the recovery/re-arm path.

## 4. Safety requirements (intent unless already in the table above)

- Heartbeat about 0.1 s; link-loss safe state at about 0.5 s (implemented).
- Watchdog in both roles (open).
- **Hardware** default-off: pull-downs on every BTS7960 R_EN/L_EN, so reset, the bootloader, an unplugged MCU, or a high-Z GPIO can never enable a motor. This is a PCB requirement.
- Make motor outputs safe before initializing the radio, display, or optional peripherals.
- No automatic arm after boot, reset, or link recovery.
- An independent physical E-stop / hard motor-enable inhibit, outside firmware.
- Brownout or reset of the RX must leave the motors disabled.
- Keep serial logging out of timing-critical paths.
- Bench-test with the wheels off the ground before child use. Fault-injection list: radio unplugged, controller unplugged, encoder failure, MCU reset, brownout, stuck joystick, motor-driver fault, over-temperature.

## 5. Inputs and authority (open)

- The local gas pedal and the remote PS2 sticks must not both write throttle in arbitrary loop order. Pick an authoritative mode (`REMOTE_CONTROL` / `LOCAL_CHILD_CONTROL`) and define which inputs are valid in each.
- Current button map (RX): SELECT = parking brake, START = tank mode, R2 = drive enable, D-pad = max speed (UP/DOWN ±25 within 25–255, LEFT = min, RIGHT = max). The other face/shoulder buttons are free for accessories or modes.
- The speed limit is a child-safety feature. The receiver must enforce limits on its own and never trust the transmitter alone.

## 6. Encoders and steering (intent)

- Sensors: multiple AS5600 encoders (12-bit, 4096 counts/rev, fixed I2C address `0x36`).
- **Decision:** use the TCA9548A I2C mux the user already owns, one AS5600 per mux channel (wheels and steering). The option table in `pcb_design_plan.md` §6 is now resolved toward the mux unless cable length forces remote sensor boards.
- Wheel position/speed: unwrap raw angle across 0/4095, accumulate a signed multi-turn count, take speed from Δcounts/Δt, and filter lightly so low-speed response survives.
- Steering gearbox: the encoder turns **2 revolutions per 1 output revolution**. A home/limit switch is active for about half of the output revolution and gives the phase. Combine switch state with AS5600 angle for an unambiguous position.
- Homing must define behavior for: boot with the switch active, boot with it inactive, boot near the transition, encoder magnet fault, obstructed motion, and home-search timeout.
- Steering control: remote command → desired position → limits → position controller → BTS7960. Fail safe on radio loss, invalid encoder, impossible position jump, homing timeout, stall/saturation, or watchdog reset. Calibrate gains and travel limits on hardware; don't guess them.
- A donor `.ino` servo/AS5600 implementation exists as reference only. Use an off-the-shelf AS5600 library, and don't copy the donor code wholesale.
- Candidate steering actuator: a salvaged 12 V car window motor (Bosch FPG or VW) driven by the steering BTS7960 at 24 V with current limiting.

## 7. Power, protection, and auto-off (intent)

- **Verify the battery pack before finalizing** dividers, TVS voltages, protection thresholds, or the charger. History: two 12 V SLA in series (the current firmware assumes total + midpoint sensing), with a Li-ion pack also considered.
- Logic DC-DC: earlier discussion mentions a 5–36 V input buck module. **The exact part is not recorded yet. Get it from the user before locking its footprint.**
- Soft-latch auto power-off: a momentary ON button brings up the logic rails, and the MCU asserts HOLD. After about 30 min with no command/activity (or on request), the MCU releases HOLD. Off-state drain must be very low. Prefer an engineered low-Iq latch (load switch/MOSFET) over a 555 timer or a plain relay. Traction power can be a separate contactor/MOSFET stage under safety control.
- Protection checklist: battery fuse plus branch fuses; MOSFET (ideal-diode) reverse polarity; TVS; flyback suppression on inductive loads; bulk capacitance at the motor drivers; local decoupling; deliberate ground/return routing; ESD on exposed connectors; brownout resilience.
- Keep traction current off the Rev A logic PCB. The PCB commands external BTS7960 modules.

## 8. Electrical gotchas

- The Feather 32u4 is **3.3 V logic**. If an MCP23017 (or any I2C breakout) is powered from 5 V with 5 V pull-ups, the bus is not automatically safe for the Feather. Power the I2C breakouts from 3.3 V or level-shift.
- **Pin conflicts found 2026-11-25 (resolve before PCB pin lock):**
  - `GAS_PEDAL_PIN A7` is Feather 32u4 **D6**, and so is `MOTOR_RR_RIS = 6`. The pedal and RR right current sense read the same pin.
  - `MOTOR_RR_LIS = 9` sits on D9/A9, which has the Feather's onboard VBAT 100k/100k divider. The reading will be corrupted.
  - The 32u4 Feather has no spare clean analog input for the planned thermistor. Proposed fix: move the pedal to ADS1115 ch2, put the NTC thermistor on ADS1115 ch3, and move RR_LIS off D9 (second ADS1115 at `0x49`, or a freed native ADC pin).
- The old `AGENTS.md` listed the steering IS pins as A6/A8. Those are D4/D8, which collide with RFM95 RST/CS. The code actually uses A10 (D10) / A11 (D12).

## 9. Current hardware work (decided 2026-11-25)

- **Receiver logic PCB Rev A in KiCad** (not Altium) so an agent can generate and edit it as text. Altium can import it later if desired. Path: `hardware/kicad/receiver_logic/`.
  - Carrier board: socketed Feather 32u4 RFM95, plus Adafruit MCP23017, PCA9685, and ADS1115 breakouts, plus the DC-DC module.
  - One Hirose DF11 harness connector per BTS7960 (×5: FL, FR, RL, RR, STEER): RPWM, LPWM, R_EN, L_EN, R_IS, L_IS, VCC, GND.
  - Pull-downs on all enables. MOSFET low-side fan output. Thermistor input.
- **Cooling: the Feather controls the fan.** A thermistor against the driver heatsinks feeds the logic PCB; firmware switches a MOSFET fan output with hysteresis (default proposal: on at 50 °C, off at 42 °C). Over-temperature should also be reported in telemetry and be able to derate or fault the drive.
- **One 3D-printed enclosure** holds all 5 BTS7960 modules, the fan, the temperature sensor, and the logic PCB. Parametric CAD with STEP + STL output at `hardware/cad/driver_enclosure/`, printed on a Bambu X2D. BTS7960 module dimensions should be measured, not assumed.

## 10. PCB tooling notes (Altium path, if used)

- The SnapEDA Adafruit Feather Altium library has a pinout that doesn't match the user's Feather. Extract its source into a project-owned library and fix the symbol/pinout; don't use the compiled library as-is.
- A dual-MCU receiver (radio MCU + sensor/control MCU) is an option, not a decision.

## 11. Telemetry wish list (intent)

Beyond the current v4 status packet: armed/fault state and fault bitfield, control mode, wheel speeds/positions, steering target vs actual plus home status, pedal position, reset reason, temperatures, and transmitter battery. Command and failsafe traffic always has priority over telemetry airtime.

## 12. Target code organization (intent)

`main` is already split into `tx_main.cpp` / `rx_main.cpp` plus `lib/` modules. As the code grows, extract toward: radio link/protocol, input (PS2, pedal, buttons), control (state machine, mixer, steering controller, safety manager, speed limiter), motor (BTS7960, PWM backends), sensors (AS5600, TCA9548A, current, battery, temperature), power manager, and Nextion UI. Keep small, testable modules; don't keep growing `rx_main.cpp`.

## 13. Open questions (answer from hardware, don't guess)

1. Installed battery chemistry/configuration and the motor stall currents.
2. Exact DC-DC converter module (part/link).
3. Final motor PWM frequency. 50 Hz is a PCA9685 default carried over from servo use, not a choice based on motor or acoustic needs.
4. AS5600 count and roles: 4 wheels + steering? Pedal?
5. Steering actuator, travel, and safe center.
6. Remote vs local authority rules.
7. Nextion model/size (see `AGENTS/Nextion/`).
8. Measured BTS7960 module outline/hole pattern, plus fan size and voltage, for the enclosure.

## 14. Out of scope

Don't merge the user's other projects into this repo: the RCBS RS5 reloading press / Teknic ClearPath servo, reloading automation, 3D-printer control, the precision scale, vehicle troubleshooting, HVAC, or laser/CNC work. They share technologies, not requirements.
