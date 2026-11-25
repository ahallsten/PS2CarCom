# AGENTS.nextion.md — Transmitter Nextion UI Guide

Nextion-specific rules only. General hardware/PCB rules are in `AGENTS/AGENTS.Altium.md`, the firmware map is in `AGENTS/AGENTS.md`, and design intent is in `AGENTS/project_context.md`. (This file used to duplicate all of `AGENTS.Altium.md`; that copy was removed.)

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

Sibling docs in this folder: `nextion_ui_plan.md`, `nextion_ai_workflow.md`, `transmitter_ui_spec.md`, `nextion_serial_protocol.md`.
