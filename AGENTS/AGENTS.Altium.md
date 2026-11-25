# AGENTS.Altium.md — Hardware/PCB Agent Guide

Hardware and PCB rules for this repository. For the current firmware and hardware map, read `AGENTS/AGENTS.md`. For design intent, decisions, and open questions, read `AGENTS/project_context.md`.

## Project identity

This is a C++/Arduino + PCB (Altium/KiCad) project for a remote-controlled Power Wheels / small vehicle platform. The project has two major electronics assemblies:

1. Transmitter / handheld controller board.
2. Receiver / vehicle control board.

Firmware is PlatformIO/Arduino on Adafruit Feather 32u4 RFM95 boards, with separate `transmitter` and `receiver` environments. The receiver logic PCB Rev A is being done in KiCad (`hardware/kicad/receiver_logic/`); see `project_context.md` §9.

## Current firmware facts

The firmware facts that used to live here were out of date. The authoritative versions are now:

- Build environments, pin/channel map, and protocol: `AGENTS/AGENTS.md`.
- Defect status and pin conflicts: `AGENTS/project_context.md` §2 and §8.

## Hardware vision

### Transmitter board

The transmitter board should be simple and portable.

Core requirements:

- Single lithium battery pack.
- Adafruit PowerBoost 1000C or a functionally similar lithium charger + 5 V boost/load-share solution.
- Adafruit RFM9x radio.
- PS2 controller interface.
- Nextion display interface.
- Latching/bistable relay or MOSFET-based power latch so the controller can power on/off manually and potentially shut down under firmware control after an idle timeout.
- Wake/sleep strategy for low standby current.
- Auxiliary GPIO LEDs for hard status when the screen fails or is disconnected.
- Battery measurement and low-battery warning.

Important design notes:

- Do a real current budget before committing to PowerBoost 1000C. Include Feather/MCU, RFM9x TX peaks, Nextion backlight, PS2 controller, LEDs, and any haptic/accessory loads.
- Do not assume the PowerBoost can power everything until the display current is known.
- Avoid paralleling two PowerBoost 1000C outputs directly. If more current is required, choose a properly rated single regulator/charger architecture or isolate/load-share rails intentionally.

### Receiver board

The receiver board is the complex assembly. It should be designed as a safety-critical vehicle controller, not just an Arduino carrier.

Core requirements:

- Manual and software-controllable master power latch using bistable relay, contactor, or MOSFET power-path circuit.
- Low-drop reverse-polarity protection, preferably MOSFET based, because batteries may be connected backwards.
- Battery monitoring for two lead-acid batteries, with future accommodation for lithium-ion/LiPo upgrade.
- Charger support:
  - UC3906 or BQ24450 style controller if building one 12 V lead-acid charger per battery.
  - LTC4020-style integrated buck-boost charger/power-path if building a more advanced 24 V system.
  - Do not use the same charger configuration for lead-acid and lithium without explicit chemistry-specific design review.
- Secondary microcontroller for real-time sensing and motor/actuator control.
- RFM9x/primary MCU can remain responsible for radio communications and command parsing.
- Ports for wheel rotation/position sensors, likely AS5600 or optical encoder channels.
- Pedal sensor port, likely encoder or AS5600.
- Steering wheel/steering angle sensor port, likely AS5600.
- Steering servo or actuator output.
- Ports for external switches, headlights, tail lights, blinkers, horn/stereo/kids legacy accessories, and auxiliary power.
- Hirose DF11 headers are preferred where practical because the project already has tools and spare connectors.
- Motor driver strategy must support current BTS7960-style DC motor drivers and future ESC/PWM brushless drivers.

Important design notes:

- Keep high-current motor paths off the logic PCB unless the board is intentionally designed as a power PCB with copper weight, thermal modeling, fusing, and creepage/clearance review.
- Provide logic/control headers to external motor drivers first; integrate high-current power later if needed.
- For multiple AS5600 sensors on one controller, check the addressing/topology. The common AS5600 I2C address is fixed, so multiple identical sensors usually need an I2C mux, separate buses, analog/PWM outputs, or local sensor microcontrollers.
- Long vehicle wiring is noisy. For cables leaving the board, prefer protected interfaces, series resistors, ESD/TVS where appropriate, filtering, keyed connectors, and clear ground strategy.

## Safety and reliability rules

Treat the vehicle receiver as safety critical.

Non-negotiables:

- Default outputs to safe/off at reset, bootloader time, firmware crash, and radio loss.
- Include a hardware-accessible E-stop or master disconnect.
- Fuse batteries and major downstream branches.
- Add watchdog and radio-loss timeout in firmware.
- Never enable motors until command validity, battery sanity, and safety interlocks are checked.
- Provide physical pull-downs/pull-ups so motor/ESC enable lines cannot float active.
- Separate logic ground routing from high-current return routing; connect intentionally at the power architecture point.
- Use conservative connectors and traces for current. Do not route battery/motor current through small headers.
- Add test points for power rails, reset, SWD/ISP/programming, UART debug, I2C, motor enables, and battery sense nodes.
- Add clear labels and polarity markings on all external connectors.

## Altium and AI workflow rules

Codex/AI should be used heavily for planning, review, and automation, but not treated as an electrical signoff authority.

Good AI tasks:

- Extract requirements from notes and datasheets.
- Build pin budgets and interface tables.
- Build current-budget spreadsheets/tables.
- Compare charger, regulator, MCU, connector, and protection options.
- Create BOM candidate tables with manufacturer part numbers, packages, LCSC/JLCPCB part numbers, lifecycle/stock notes, and datasheet links.
- Write Altium scripting helpers, outjob templates, rule checklists, naming conventions, and release checklists.
- Review schematics from exported PDFs/images and call out missing decoupling, weak protection, ambiguous nets, or unverified footprints.
- Review generated Gerbers, BOM, CPL, and DRC/erc reports.

Bad AI tasks:

- Blindly editing native binary `.SchDoc` or `.PcbDoc` files.
- Creating or modifying footprints without datasheet dimensional verification.
- Approving battery charging circuits without checking datasheet equations, thermal limits, and actual battery specs.
- Assuming JLCPCB assembly availability without checking the current JLCPCB/LCSC part entry.
- Routing high-current power without current, copper, connector, and thermal calculations.

Altium file guidance:

- Keep Altium projects under `hardware/altium/` once created.
- Keep generated manufacturing outputs under `hardware/releases/<board>/<rev>/`.
- Prefer text-friendly companion files in Git:
  - Markdown design notes.
  - CSV BOMs.
  - Pin maps.
  - Interface tables.
  - Net naming tables.
  - Output job notes.
  - Screenshots or PDFs for review.
- If Altium ASCII exports are used, treat them as review artifacts unless the team confirms round-trip behavior in Altium.
- Always validate AI-generated scripts and edits in Altium before committing.

## Suggested repository structure

```text
/
  CLAUDE.md                 pointer to AGENTS/
  platformio.ini
  src/ include/ lib/        firmware
  AGENTS/                   agent and design docs (this folder)
  hardware/
    kicad/receiver_logic/   receiver logic PCB Rev A
    cad/driver_enclosure/   3D-printed BTS7960/fan/PCB enclosure (STEP + STL)
    altium/                 if/when Altium boards are created
    datasheets/
    releases/<board>/<rev>/ fab outputs (Gerbers, BOM, CPL, PDFs)
```

## Firmware verification commands

Use PlatformIO as the source of truth.

```sh
pio run -e transmitter
pio run -e receiver
```

Before changing PCB pin assignments, update firmware pin maps and interface tables together.

## PCB planning workflow

Use this order:

1. Requirements capture.
2. System block diagram.
3. Power tree and current budget.
4. Safety concept and power-off/failsafe concept.
5. Connector inventory and cable plan.
6. MCU pin budget.
7. Component selection with JLCPCB/LCSC assembly status.
8. Schematic page partitioning.
9. ERC and design review.
10. PCB placement with mechanical constraints.
11. Routing and DRC.
12. Manufacturing package: Gerbers, drill files, BOM, CPL, assembly drawings, schematic PDF, fabrication notes.
13. Bring-up plan and test checklist.

## JLCPCB-oriented component rules

For any part intended for assembly by JLCPCB, record:

- Reference designator group.
- Value.
- Function.
- Manufacturer part number.
- Package.
- Footprint name.
- LCSC/JLCPCB part number.
- JLCPCB category if known: basic, extended, pre-order, global sourcing, or customer-supplied.
- Substitution constraints.
- Datasheet source.
- Critical layout notes.

Prefer JLCPCB basic/common parts for passives, small MOSFETs, LDOs, LEDs, connectors when reasonable. Do not let JLCPCB convenience override safety-critical part quality.

## Board partitioning preference

Recommended first hardware split:

- Transmitter PCB: integrate portable UI/control/power functions.
- Receiver logic/power-distribution PCB: controller, radio, sensor ports, protected low-current outputs, driver control headers, battery sensing.
- External high-current motor drivers/ESCs and external charger/power modules for early revisions.

Only integrate high-current H-bridges or battery charging on the main receiver PCB after the first prototype proves the control architecture.

## Known firmware issues to review before PCB pin lock

See `AGENTS/project_context.md` §2 (defect status) and §8 (pin conflicts). Most historical issues are fixed. The open ones that affect the PCB are the pin conflicts on D6 and D9, the lack of a spare analog input for the thermistor, and the hardware pull-downs on enables.

## Open decisions

Do not freeze schematic symbols, footprints, or PCB until these are answered:

- Exact transmitter MCU/module: Feather 32u4 with RFM9x, separate Feather + RFM9x breakout, or custom MCU + radio?
- Exact receiver MCU(s): Feather 32u4, newer 3.3 V MCU, dual MCU, or CAN/RS-485 distributed nodes?
- Battery topology: two 12 V lead-acid in series, separately accessible midpoint, or parallel/other arrangement?
- Charger topology: one charger per 12 V battery, 24 V integrated charger, external charger only, or modular charger daughterboard?
- Future lithium topology: pack voltage, BMS choice, charge port, and whether the board should support both chemistries with jumpers/daughterboards.
- Motor topology: brushed DC with BTS7960 modules, integrated H-bridge, or brushless ESC PWM.
- Steering actuator type and current.
- Nextion display model and current.
- PS2 controller voltage and connector pinout.
- Exact AS5600 breakout/module pinout or custom AS5600 PCB.
- Whether wheel sensors are local to the main board or remote sensor boards.
