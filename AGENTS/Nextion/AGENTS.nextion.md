# AGENTS.md

This file gives Codex and other AI assistants durable project context for this repository.

## Project identity

This is a C++/Arduino + Altium PCB project for a remote-controlled Power Wheels / small vehicle platform. The project has two major electronics assemblies:

1. Transmitter / handheld controller board.
2. Receiver / vehicle control board.

The current firmware is a PlatformIO Arduino project targeting an Adafruit Feather 32u4-style environment. The existing firmware has conditional roles for TRANSMITTER and RECEIVER, with RECEIVER currently enabled in `main.cpp`.

## Current firmware facts

- PlatformIO environment: `feather32u4`.
- Platform: `atmelavr`.
- Framework: Arduino.
- Serial monitor: 115200 baud.
- Current dependencies include:
  - Adafruit SleepyDog Library
  - RadioHead
  - PsxNewLib
  - LibPrintf
  - Adafruit MCP23017 Arduino Library
- Radio currently uses an RH_RF95 object with RFM95 pins:
  - CS: 8
  - RST: 4
  - INT: 7
  - Frequency: 915.0 MHz
- Transmitter firmware intent:
  - Read PS2 controller state.
  - Serialize `ControllerState`.
  - Send packets over RFM9x / RH_RF95.
- Receiver firmware intent:
  - Receive `ControllerState` packets.
  - Interpret button edges and joystick axes.
  - Control motor drivers and steering.
  - Current receiver code references MCP23017 expansion and four BTS7960 motor driver objects: front-left, front-right, rear-left, rear-right.
- Check for missing/local headers before building, especially `Joystick.h` and `AxisMap.h`.

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


## Nextion UI workflow rules

The transmitter may use a Nextion UART HMI display. Treat the display as an HMI, not as the safety authority.

Core display principles:

- The MCU owns truth for radio state, controller state, safety state, battery state, speed limit, arming, and faults.
- The Nextion owns layout, pages, fonts, pictures, touch widgets, and local visual behavior.
- The MCU sends concise UART updates to the display.
- The display sends touch events or simple event IDs back to the MCU.
- The transmitter must remain safe and functional if the display is disconnected, frozen, or rebooting.
- Hard status LEDs are still required for power/radio/controller/fault indication.

Good AI tasks for Nextion:

- Design page flows, wireframes, object naming conventions, event maps, and serial protocol specs.
- Generate Markdown/CSV companion files such as object maps, event maps, and UI state tables.
- Generate Arduino/C++ helper classes for sending Nextion commands and parsing events.
- Review Nextion page screenshots for readability, safety hierarchy, missing states, and inconsistent names.
- Check consistency between firmware constants and documented Nextion object names.

Bad AI tasks for Nextion:

- Blindly editing binary `.hmi` or compiled `.tft` files.
- Treating a Git diff of `.hmi`/`.tft` as a meaningful design review.
- Making display touch events directly arm motors or override safety interlocks.
- Assuming display current draw, voltage tolerance, or mounting dimensions without the selected model datasheet.

Recommended Nextion files:

```text
  docs/
    nextion/
      nextion_ui_plan.md
      nextion_ai_workflow.md
      transmitter_ui_spec.md
      nextion_serial_protocol.md
      object_map.csv
      event_map.csv
  ui/
    nextion/
      transmitter.hmi
      transmitter.tft
      exports/
        screenshots/
        compile_logs/
    assets/
      source/
      nextion/
```

Record the exact Nextion model, screen resolution, Nextion Editor version, UART baud rate, and display current budget before freezing the transmitter PCB connector and power supply.

## Suggested repository structure

```text
/
  AGENTS.md
  platformio.ini
  src/ or firmware/
  include/
  lib/
  docs/
    pcb/
      pcb_design_plan.md
      ai_altium_workflow.md
      interface_inventory.md
      current_budget.md
      component_selection.md
    nextion/
      nextion_ui_plan.md
      nextion_ai_workflow.md
      transmitter_ui_spec.md
      nextion_serial_protocol.md
  hardware/
    altium/
      transmitter/
      receiver/
    datasheets/
    releases/
      transmitter/
      receiver/
```

The current uploaded files are flat in the project root. If reorganizing, update PlatformIO paths carefully.

## Firmware verification commands

Use PlatformIO as the source of truth.

```sh
pio run -e feather32u4
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

- `detectButtonWordStateChange()` currently declares `previousButtonWord` locally without initialization/persistence. It should likely be `static` or a global initialized to zero.
- The `map()` calls intended to limit max speed do not assign the return value.
- Receiver currently drives all four motors from `leftPWM`; verify steering/tank/mixing logic before final pin allocation.
- Some comments mention MCP23017 pins while some `BTS7960` methods use Arduino `digitalWrite()` directly. Confirm which signals are on the MCU versus MCP23017 before board design.
- BTS7960 header declares `disable()` and `stop()` but implementation currently does not define them.
- Resolve missing headers and build warnings before freezing electrical interfaces.

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
