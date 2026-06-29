# AI + Altium Workflow

Created: 2026-06-29

## 1. Direct answer

AI can help a lot with Altium PCB design, but the most reliable workflow is not to let AI blindly edit native Altium schematic/layout files.

Use AI for:

- Requirements capture.
- Datasheet extraction.
- Component comparison.
- Pin maps.
- Connector tables.
- Current budgets.
- JLCPCB BOM fields.
- Schematic review.
- DRC/DFM review.
- Altium script generation.
- Release package checklists.

Use Altium itself for:

- Final schematic capture.
- Footprint placement.
- Rule setup.
- Routing.
- ERC/DRC.
- 3D/mechanical checks.
- Manufacturing exports.

## 2. File-format strategy

Codex works best on text and code. Altium projects include some text-friendly files and some native design documents that are not ideal for automated blind editing.

Recommended repo approach:

```text
hardware/altium/              Native Altium projects
hardware/releases/            Exported manufacturing packages
hardware/datasheets/          Datasheets and notes
hardware/parts/               BOM candidate CSVs and library notes
docs/pcb/                     Human/AI readable design intent
```

Track these in Git:

- `*.PrjPCB`
- `*.OutJob`
- Markdown docs.
- CSV BOMs.
- Pin tables.
- Datasheet notes.
- Altium scripts.
- Release ZIPs or release manifests.
- Schematic PDFs for review.

Be cautious with:

- `*.SchDoc`
- `*.PcbDoc`
- Altium binary library files.

If using Altium ASCII exports, test round-trip behavior. Do not assume every feature survives ASCII export/import perfectly.

## 3. How to use Codex effectively

### Good Codex prompt pattern

Give Codex a bounded task, clear inputs, and a required output format.

Example:

```text
Read docs/pcb/pcb_design_plan.md and the uploaded datasheets in hardware/datasheets.
Create docs/pcb/component_selection.md with candidate JLCPCB-assemblable parts for:
- 0603 resistors/capacitors
- status LEDs
- low-side MOSFET outputs up to 2 A
- TVS diode for 24 V battery input
- reverse-polarity MOSFET candidate
For each part include MPN, package, LCSC/JLCPCB number if found, voltage/current ratings, and why it is suitable.
Do not invent unavailable JLCPCB part numbers. Mark unknown values TBD.
```

### Bad Codex prompt pattern

Avoid vague prompts such as:

```text
Design the PCB for me.
```

That is too broad and encourages unverified assumptions.

## 4. Suggested AI-assisted PCB workflow

### Phase 1: Requirements

Outputs:

- System block diagram.
- Board partitioning decision.
- Power budget.
- Connector inventory.
- Pin budget.
- Safety checklist.

AI role:

- Convert notes into tables.
- Identify missing specifications.
- Propose block diagrams and design alternatives.

Human/Altium role:

- Confirm actual modules, connectors, and mechanical constraints.

### Phase 2: Component selection

Outputs:

- Candidate BOM.
- JLCPCB part status.
- Datasheet notes.
- Footprint notes.

AI role:

- Find candidate parts.
- Summarize key ratings.
- Build comparison tables.

Human/Altium role:

- Verify datasheet equations, pinouts, package land patterns, and thermal ratings.

### Phase 3: Schematic capture

Outputs:

- Schematic pages.
- ERC report.
- PDF schematic review package.

AI role:

- Review exported PDFs/images.
- Check for missing decoupling, pull-ups, programming headers, test points, polarity labels, and protection.
- Cross-check pin maps against firmware.

Human/Altium role:

- Place symbols and wire nets.
- Run ERC.
- Resolve all warnings intentionally.

### Phase 4: PCB layout

Outputs:

- Placement screenshots.
- Routing screenshots.
- DRC report.
- 3D model screenshot.

AI role:

- Review placement/routing screenshots for obvious risks.
- Check connector orientation and serviceability.
- Review DRC report text.

Human/Altium role:

- Place and route.
- Define rules.
- Check clearances, current paths, thermal reliefs, copper pours, and mechanical constraints.

### Phase 5: Manufacturing release

Outputs:

- Gerbers or Gerber X2.
- NC drill files.
- BOM.
- CPL/pick-place.
- Assembly drawings.
- Schematic PDF.
- Fab notes.
- Release checklist.

AI role:

- Review output package completeness.
- Check BOM/CPL reference designators against schematic.
- Flag DNP/customer-supplied parts.

Human/Altium/JLCPCB role:

- Upload to JLCPCB.
- Check their preview carefully.
- Verify orientation of every polarized part before ordering.

## 5. JLCPCB BOM tips

For each BOM line, maintain these columns in a local CSV even if Altium uses additional fields:

| Column | Purpose |
|---|---|
| Designator | R1,R2,R3 or U1 etc. |
| Quantity | Placement count. |
| Value | Electrical value. |
| Footprint | Package/land pattern. |
| Manufacturer | Actual manufacturer. |
| MPN | Manufacturer part number. |
| LCSC/JLCPCB PN | Assembly matching. |
| Assembly | JLCPCB, hand, DNP, customer-supplied. |
| Notes | Voltage/current/tolerance/substitution rules. |

Rules:

- Do not use generic symbols without final MPNs for assembly parts.
- Keep DNP parts in a clear variant or separate BOM field.
- Check stock shortly before ordering.
- Prefer common/basic passives, but do not compromise ratings.
- For connectors, verify JLCPCB can place the exact part or plan hand assembly.

## 6. Altium tips for this project

- Use hierarchical or multi-sheet schematics. Do not put the whole receiver on one sheet.
- Use consistent net labels and harness/ports for repeated motor and sensor channels.
- Make one channel carefully, review it, then copy for FL/FR/RL/RR.
- Use rooms/classes for repeated wheel sensor and motor driver sections.
- Define net classes early:
  - battery/high current
  - motor power
  - 12 V accessory
  - 5 V logic/accessory
  - 3.3 V logic
  - radio/SPI
  - I2C sensors
  - noisy outputs
- Define component classes for JLCPCB assembly, hand-assembly, DNP, and test points.
- Put test points on every rail and every critical control signal.
- Put mounting holes and board outline in early, not after routing.
- Add fiducials and tooling/assembly marks early.
- Use Altium variants for optional charger circuits, lithium vs lead-acid experiments, and hand/DNP options.
- Export schematic PDF at every review checkpoint.
- Treat ERC warnings as design decisions. Do not just suppress them.
- Run DRC before every commit that changes PCB layout.

## 7. AI review checklist for schematics

Ask AI to check:

- Does every IC have local decoupling near power pins?
- Are regulator input/output capacitors per datasheet?
- Are all enable/reset/boot pins pulled to safe states?
- Are off-board signals protected or filtered?
- Are I2C pull-ups sized and connected to the correct rail?
- Are there address conflicts on shared buses?
- Are connector pinouts labeled and keyed?
- Are batteries fused and reverse-polarity protected?
- Are high-voltage ADC dividers safe at max voltage?
- Do MOSFET gates have pulldowns and reasonable gate resistors?
- Do inductive loads have flyback/suppression?
- Can firmware program/debug both MCUs after assembly?
- Are motor/ESC outputs safe during reset and bootloader time?
- Are test points accessible after enclosure assembly?

## 8. AI review checklist for PCB layout

Ask AI to review screenshots/PDFs for:

- Power entry path clarity.
- Connector orientation and labels.
- Separation between noisy motor/power paths and radio/analog/sensor areas.
- Crystal/oscillator placement if used.
- Regulator loop area and thermal relief.
- Ground return paths.
- Antenna keepout for RFM9x.
- Adequate copper for current.
- Mechanical clearance for headers/cables.
- Silkscreen polarity marks.
- Fiducial and mounting hole placement.
- Test point access.

## 9. Suggested release checklist

Before ordering:

```text
[ ] Schematic PDF reviewed.
[ ] ERC reviewed; every warning resolved or documented.
[ ] Firmware pin map matches schematic.
[ ] Mechanical outline/mounting checked.
[ ] Board stackup chosen.
[ ] Net classes and rules set.
[ ] DRC clean or documented.
[ ] All assembled parts have MPN + LCSC/JLCPCB PN.
[ ] BOM exported and checked.
[ ] CPL/pick-place exported and checked.
[ ] Polarity/orientation checked in JLCPCB preview.
[ ] Gerber/drill package checked in viewer.
[ ] Assembly drawing generated.
[ ] Bring-up plan written.
[ ] Spare parts and hand-assembly parts identified.
```

## 10. Bring-up strategy

Bring up boards in safe stages:

1. Visual inspection before power.
2. Continuity check for power rails and battery inputs.
3. Current-limited bench supply power-up, not batteries first.
4. Verify regulators with no modules installed if possible.
5. Program MCU and run blink/UART test.
6. Verify radio only after logic rails are stable.
7. Verify sensors one at a time.
8. Verify motor/ESC command outputs with drivers disconnected.
9. Use dummy loads before real motors.
10. Use wheels off the ground for first powered motor tests.
11. Verify radio-loss timeout, watchdog, and E-stop before letting a child ride.
