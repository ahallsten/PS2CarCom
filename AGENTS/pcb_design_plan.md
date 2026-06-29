# PCB Design Plan

Created: 2026-06-29

## 1. Goal

Design one or more PCBs for a remote-controlled Power Wheels / small vehicle project using Altium Designer Student Edition, with JLCPCB as the likely fabrication and assembly vendor.

The project should be designed in stages. The first PCB revision should reduce wiring mess, improve reliability, and expose good test points without trying to integrate every high-current subsystem immediately.

## 2. Board set

### Board A: Transmitter / handheld controller

Purpose: portable control unit with radio, PS2 controller interface, display, power management, and status LEDs.

Candidate subsystems:

| Subsystem | Candidate implementation | Notes |
|---|---|---|
| Battery | 1-cell Li-ion/LiPo | Add protection/low-battery handling. |
| Charger/boost | PowerBoost 1000C or equivalent | Must pass current budget. Avoid paralleling boost outputs. |
| Radio | Adafruit RFM9x / RFM95 915 MHz | Existing firmware uses RH_RF95 at 915 MHz. |
| Controller input | PS2 controller via PsxNewLib | Confirm controller voltage and connector pinout. |
| Display | Nextion UART display | Confirm model, current, mounting, and 5 V rail budget. |
| Power latch | Bistable relay or MOSFET latch | Must support manual on/off and firmware shutdown. |
| Status LEDs | GPIO LEDs | Include screen-fault/status fallbacks. |
| Debug | UART/programming/test pads | Bring-up needs visible test points. |

### Board B: Receiver / vehicle controller

Purpose: vehicle electronics hub. It should handle radio command reception, local sensor acquisition, motor/ESC command generation, safety interlocks, lighting/accessories, and power monitoring.

Candidate subsystems:

| Subsystem | Candidate implementation | Notes |
|---|---|---|
| Master power | Bistable relay, contactor, or MOSFET power path | Include manual kill and software shutdown. |
| Reverse polarity | Low-loss MOSFET ideal-diode style protection | Size for battery current and fault conditions. |
| Battery monitor | Divider + filtering + ADC, or dedicated monitor IC | Monitor both batteries and midpoint if using 2x 12 V series. |
| Charger | Modular per-battery charger or advanced 24 V charger | Keep charger modular until battery topology is final. |
| Radio MCU | Existing Feather/RFM9x or integrated module | Keep radio physically away from motor noise. |
| Control MCU | Secondary MCU for sensors/motors | Use deterministic loop and watchdog. |
| Wheel sensors | AS5600 or optical encoders | AS5600 multi-sensor topology must be planned. |
| Pedal | AS5600, encoder, or analog Hall/pot | Use plausibility checks and input filtering. |
| Steering angle | AS5600 or encoder | Useful for closed-loop steering. |
| Steering actuator | Servo/PWM output or motor driver | Confirm actuator current and feedback. |
| Motor drivers | BTS7960 headers now, ESC PWM later | Do not integrate high current too early. |
| Lights/accessories | Protected low-side or high-side outputs | Add flyback/TVS where loads are inductive. |
| Connectors | Hirose DF11 where practical | Use keyed/polarized headers for battery/high-current. |

## 3. Recommended hardware architecture for Rev A

### Transmitter Rev A

A practical Rev A transmitter board should be a carrier board, not a custom RF/power design from scratch.

Recommended approach:

1. Use known-good modules for risky blocks:
   - Feather/MCU module if possible.
   - Adafruit RFM9x module or Feather RFM9x.
   - PowerBoost 1000C or a validated power module.
2. Put connectors, LEDs, power switch/latch, battery sense, display connector, and PS2 connector on the PCB.
3. Add labeled test pads for 5 V, 3.3 V, battery, EN, radio reset, SPI, UART, and ground.
4. Leave mechanical room for the battery and display cable.
5. Confirm standby/off current. A battery-powered transmitter can be ruined by a few mA of accidental sleep current.

### Receiver Rev A

A practical Rev A receiver board should separate logic, sensing, and control from raw motor current.

Recommended approach:

1. Use a protected logic board with connectors to external motor drivers/ESCs.
2. Put battery sense, reverse-protected logic power, radio module, MCU(s), sensor headers, lights/accessory drivers, and motor driver command headers on the PCB.
3. Use external fuses and external high-current distribution for motors.
4. Use external charger modules or a charger daughterboard for the first revision unless the charger is the main project focus.
5. Provide a real manual E-stop/master disconnect outside firmware control.

## 4. Power architecture

### Transmitter power tree

```text
1S Li-ion/LiPo battery
  -> charger/load-share/boost module
    -> 5 V rail
      -> Nextion display
      -> PS2 controller if it requires 5 V
      -> Feather/logic regulator input if using a 5 V tolerant module
    -> 3.3 V rail where needed
      -> MCU logic
      -> raw RFM9x module if not using Adafruit breakout level shifting
```

Required measurements before schematic freeze:

| Load | Voltage | Current estimate | Verified? |
|---|---:|---:|---|
| MCU/Feather | TBD | TBD | No |
| RFM9x TX peak | TBD | TBD | No |
| PS2 controller | TBD | TBD | No |
| Nextion display/backlight | TBD | TBD | No |
| LEDs | TBD | TBD | No |
| Buzzer/haptics/accessory | TBD | TBD | No |

### Receiver power tree

```text
Battery pack / two lead-acid batteries
  -> fuse / master disconnect
  -> reverse polarity protection
  -> master power latch / contactor / MOSFET switch
  -> high-current motor distribution outside logic PCB
  -> protected DC/DC converter(s)
    -> 12 V accessory rail if needed
    -> 5 V logic/accessory rail
    -> 3.3 V MCU/radio/sensor rail
```

Receiver rails to define:

| Rail | Purpose | Notes |
|---|---|---|
| VBAT_RAW | Battery before protection | Sense only through high impedance/protection. |
| VBAT_PROT | After reverse protection | Main protected input. |
| VMOTOR | Motor/ESC supply | High-current; likely off-board distribution. |
| 12 V AUX | Lights, horn, stereo, accessories | Switch/protect each output. |
| 5 V | Servos, Nextion-like accessories, some sensors | Size for servo stall if used. |
| 3.3 V | MCU, radio, logic sensors | Keep clean; filter from motor noise. |

## 5. Battery and charger planning

Current candidate charger ICs from prior research:

| IC/module | Best use | Risk/complexity |
|---|---|---|
| TI UC3906 | One sealed lead-acid charger per 12 V battery | Linear pass design can dissipate heat. Older but prototype-friendly. |
| TI BQ24450 | Standalone lead-acid charger controller | Still requires external pass/power design and thermal analysis. |
| ADI LTC4020 | Integrated 24 V-ish buck-boost multi-chemistry charger/power-path | Excellent but complex; better after eval-board study. |
| External charger | Early prototype | Simplest and safest while vehicle control is under development. |

Recommended path:

1. Rev A receiver: include battery monitor and charge connector access, but use external chargers or charger daughterboards.
2. Rev B: add one 12 V lead-acid charger per battery if the pack remains 2x 12 V SLA.
3. Rev C or charger daughterboard: consider LTC4020-style integrated 24 V charging/power path if needed.
4. Lithium upgrade: treat as a separate pack/BMS/charger design. Do not make a jumper-selectable lead-acid/lithium charger unless the exact battery chemistry, cell count, BMS, and safety requirements are fully specified.

## 6. Sensor topology

### AS5600 planning

AS5600 is attractive for pedal, steering, and wheel rotation because it is contactless and absolute. However, many AS5600 devices cannot simply share one I2C bus without planning because the common address is fixed.

Options:

| Option | Pros | Cons |
|---|---|---|
| I2C mux such as TCA9548A | Simple central MCU wiring | Adds part and software complexity. |
| Separate I2C buses | Clean if MCU has enough I2C peripherals | Pin-limited on small MCUs. |
| AS5600 analog/PWM outputs | Avoids address conflict | More ADC/timer inputs; may be noisier over long wires. |
| Local sensor microcontrollers | Robust for distributed sensors | More boards/firmware. |
| Optical quadrature encoders | Good velocity/direction | Not absolute unless indexed/homed. |

Default recommendation:

- Use local connector pinouts that can support both digital and analog/PWM outputs.
- For wheel sensors, strongly consider remote sensor boards or differential/conditioned encoder signals if cable runs are long.

### Candidate DF11 sensor connector

Do not freeze until the actual sensor module is selected. A flexible 6-pin version is:

| Pin | Signal | Notes |
|---:|---|---|
| 1 | GND | Cable shield/drain strategy TBD. |
| 2 | VCC_SENSOR | 3.3 V or 5 V, keyed per board revision. |
| 3 | SDA / A | I2C SDA or encoder A. |
| 4 | SCL / B | I2C SCL or encoder B. |
| 5 | OUT / PWM / INDEX | Optional analog/PWM/index. |
| 6 | ID / DIR / spare | Sensor ID resistor, direction, or spare GPIO. |

## 7. Motor and actuator planning

### Brushed DC/BTS7960 path

Existing firmware uses BTS7960-style objects. For PCB Rev A, expose headers for external BTS7960 modules:

| Signal | Purpose |
|---|---|
| RPWM | Forward/reverse PWM side 1 |
| LPWM | Forward/reverse PWM side 2 |
| R_EN | Enable side 1 |
| L_EN | Enable side 2 |
| R_IS | Current sense/fault if used |
| L_IS | Current sense/fault if used |
| VLOGIC | Driver logic supply if needed |
| GND | Common reference |

Add series resistors/pulldowns on control signals as appropriate. Confirm the exact BTS7960 module pinout before laying out.

### Brushless ESC path

For future brushless motor/ESC use, provide servo-style PWM outputs:

| Pin | Signal | Notes |
|---:|---|---|
| 1 | GND | Must share command ground with ESC. |
| 2 | +5 V/BEC optional | Usually do not tie multiple BEC outputs together. Consider DNP link. |
| 3 | PWM command | Add pulldown and maybe series resistor. |

Include a firmware-safe startup sequence so ESCs never arm unexpectedly.

## 8. Lighting and accessory outputs

Use protected switching. Do not connect unknown legacy loads directly to MCU pins.

Candidate output types:

| Load | Driver style | Notes |
|---|---|---|
| Headlights | Low-side MOSFET or high-side switch | Add current limit/fuse. |
| Tail lights | Low-side MOSFET | PWM dimming optional. |
| Blinkers | Low-side MOSFET | Firmware pattern control. |
| Horn | MOSFET/relay | Inductive suppression required. |
| Stereo/songs | Switched auxiliary power | Verify voltage/current first. |

## 9. Schematic page plan

Suggested Altium schematic pages:

### Transmitter

1. Cover/symbols/revision notes.
2. MCU/radio module.
3. Power/charger/boost/latch.
4. PS2 controller interface.
5. Nextion display interface.
6. LEDs/buttons/debug/test points.
7. Mounting/mechanical notes.

### Receiver

1. Cover/symbols/revision notes.
2. Power entry, fuse notes, reverse protection, power latch.
3. DC/DC regulators and rail monitoring.
4. Radio/primary MCU.
5. Secondary MCU and inter-MCU link.
6. Sensor connectors.
7. Motor/ESC control headers.
8. Lighting/accessory outputs.
9. Battery monitoring and charger/daughterboard interface.
10. Programming/debug/test points.
11. Mounting/mechanical notes.

## 10. JLCPCB design-for-assembly checklist

Before ordering assembled boards:

- Every assembled component has an MPN and JLCPCB/LCSC part number.
- BOM uses consistent designators and packages.
- CPL/pick-place file includes correct top/bottom side and rotations.
- Fiducials are present if using SMT assembly.
- Parts with polarity have clear silkscreen and assembly drawing marks.
- Connectors are not so close that hand assembly/rework is painful.
- SMD spacing follows JLCPCB assembly spacing guidance.
- Parts not assembled by JLCPCB are marked DNP or hand-assemble.
- Heavy or through-hole connectors have mechanical strain relief strategy.
- Board outline, mounting holes, keepouts, and enclosure constraints are verified.
- Gerbers, drill files, BOM, CPL, schematic PDF, and assembly drawings are exported together from a controlled OutJob.

## 11. Altium project conventions

Suggested naming:

```text
hardware/altium/transmitter/Transmitter.PrjPCB
hardware/altium/transmitter/Transmitter_Power.SchDoc
hardware/altium/transmitter/Transmitter_UI.SchDoc
hardware/altium/transmitter/Transmitter.PcbDoc

hardware/altium/receiver/Receiver.PrjPCB
hardware/altium/receiver/Receiver_Power.SchDoc
hardware/altium/receiver/Receiver_MCU.SchDoc
hardware/altium/receiver/Receiver_IO.SchDoc
hardware/altium/receiver/Receiver.PcbDoc
```

Net names should be descriptive:

- `VBAT_RAW`
- `VBAT_PROT`
- `VMOTOR`
- `PWR_LATCH_SET`
- `PWR_LATCH_RESET`
- `MOTOR_FL_RPWM`
- `MOTOR_FL_LPWM`
- `SENS_WHEEL_FL_SDA`
- `SENS_WHEEL_FL_SCL`
- `RADIO_SPI_SCK`
- `RADIO_DIO0_INT`

## 12. First AI/Codex tasks

Give Codex small, checkable tasks:

1. Build a `docs/pcb/current_budget.md` table for transmitter and receiver rails.
2. Build a `docs/pcb/interface_inventory.md` from firmware pins plus desired sensors/accessories.
3. Build a component candidate table with JLCPCB/LCSC numbers for passives, MOSFETs, regulators, LEDs, connectors, TVS parts, and common logic.
4. Review firmware pin usage and flag conflicts before schematic pin lock.
5. Generate Altium naming conventions and OutJob checklist.
6. Draft schematic review checklist.
7. Draft bring-up checklist.

## 13. Critical open questions

Answer these before Rev A schematic freeze:

- Exact transmitter display model and max backlight current?
- Exact PS2 controller connector/pinout and operating voltage?
- Exact battery capacity and chemistry for transmitter?
- Exact receiver battery arrangement and current draw of stock motors?
- Are two 12 V lead-acid batteries in series, and is the midpoint accessible?
- Are existing motors brushed DC only, and what is stall current?
- Which motor drivers will be used for Rev A?
- Which MCU should be used for the secondary controller?
- How many wheel sensors are required in Rev A: 2 or 4?
- Are AS5600 sensors on custom boards or off-the-shelf breakout boards?
- Steering: hobby servo, DC motor with feedback, linear actuator, or other?
- Does the board need environmental sealing, conformal coating, or automotive-style connectors?
