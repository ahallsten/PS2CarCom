# CLAUDE.md

Agent instructions for PS2CarCom live in `AGENTS/`. Start here:

1. `AGENTS/AGENTS.md`: current firmware/hardware map, build commands, safety-sensitive code. **Always read this first.**
2. `AGENTS/project_context.md`: design intent, defect status, pin conflicts, hardware decisions, open questions. Read it before any design, safety, PCB, or CAD work.
3. The rest of `AGENTS/` is indexed in the "Doc Index" table at the top of `AGENTS/AGENTS.md`.

Ground rules:

- The code is the source of truth. Docs describing intent are not implemented features.
- This drives a child's ride-on vehicle. Don't silently change failsafe, enable, parking-brake, or motor-direction behavior. For any safety change, state the failure mode, safe state, detection, and re-arm path.
- Build both roles after firmware changes: `pio run -e transmitter` and `pio run -e receiver`.
- `lib/Protocol/Protocol.h` is shared by both roles. Change the encoder and decoder together and bump the version.
- Hardware sources go under `hardware/` (`kicad/`, `cad/`, `datasheets/`, `releases/`).
- When you learn something durable about the hardware or a design decision, update `AGENTS/project_context.md`. Don't start a new notes file.
