# AI and Nextion Workflow

Created: 2026-06-29

## 1. Practical answer

Use AI heavily for planning and firmware around Nextion. Use the Nextion Editor as the canonical tool for the actual `.hmi` project and `.tft` build.

This is analogous to Altium:

- Altium owns the schematic/layout design files.
- Nextion Editor owns the HMI design file.
- Git and AI own the companion source-of-truth documents, tables, object maps, protocol specs, generated firmware constants, screenshots, and checklists.

## 2. Recommended repo structure

```text
/
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
  firmware/
    display/
      NextionDisplay.h
      NextionDisplay.cpp
      NextionObjects.h
```

## 3. Files AI can safely maintain

Excellent targets for Codex/AI:

- Markdown UI requirements.
- CSV object map.
- CSV event map.
- Serial protocol spec.
- Firmware constants for component names and page IDs.
- C++ helper classes for writing Nextion commands.
- Test sketches for display bring-up.
- Asset manifest.
- Screenshot review notes.
- Release checklist.

Poor targets for blind AI editing:

- `.hmi` project file.
- `.tft` compiled output.
- Generated build folders from Nextion Editor.
- Any reverse-engineered HMI/TFT edits not round-tripped through Nextion Editor.

## 4. AI-assisted design loop

1. Write UI requirements in Markdown.
2. Ask AI to create a page list and page-wireframe table.
3. Create the pages manually in Nextion Editor.
4. Export screenshots or take screenshots of each page.
5. Ask AI to review screenshots for readability, safety, missing states, and naming consistency.
6. Export or maintain `object_map.csv` and `event_map.csv`.
7. Ask AI to generate firmware constants and helper code from the maps.
8. Build and upload `.tft` using Nextion Editor or microSD workflow.
9. Test serial communication with a simple display test sketch.
10. Integrate with transmitter firmware.

## 5. Prompt patterns that work well

Use prompts that provide exact constraints.

Examples:

```text
Given this transmitter project, design a 4-page Nextion UI for a 3.5 inch 480x320 screen.
Use large text, child-safe status emphasis, and no more than 6 touch targets per page.
Return: page list, component names, displayed values, and MCU update frequency.
```

```text
Here is my object_map.csv and event_map.csv. Generate a C++ NextionDisplay class for Arduino.
Requirements: nonblocking writes, helper functions for text/number/gauge updates, triple-0xFF termination, and a cached last-value layer.
```

```text
Review this Nextion page screenshot. Look for ambiguous labels, status hierarchy problems, accidental-arm risks, unreadable small text, missing fault state, and inconsistent naming.
```

## 6. Version control strategy

Commit these:

- `.hmi` source file.
- Known-good `.tft` output for each release.
- Screenshots of each page.
- Object/event maps.
- Firmware constants generated from maps.
- Release notes saying which Nextion Editor version built the `.tft`.

Do not rely on Git diffs of `.hmi` or `.tft` for meaningful review. Treat screenshots and exported text/maps as the reviewable layer.

## 7. Release checklist

Before calling a display release good:

- Exact Nextion model recorded.
- Nextion Editor version recorded.
- `.hmi` committed.
- `.tft` committed or attached as release artifact.
- Page screenshots committed.
- Object map and event map reviewed.
- Firmware object names match HMI object names.
- Touch events tested.
- Display disconnected test passes.
- Display reboot during transmitter operation test passes.
- MCU reboot while display remains powered test passes.
- Brightness/current budget checked.
- Low battery and radio loss visible without relying only on the screen.

