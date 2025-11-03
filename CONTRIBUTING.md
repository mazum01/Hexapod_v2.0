# Contributing to MARS Hexapod v2.0

Thanks for helping improve the MARS Hexapod firmware. This project targets a Teensy 4.1 and prioritizes deterministic timing, small footprint, and clear operator UX.

## Ground rules
- Keep the main sketch minimal: core setup/loop and tick logic in `MARS_Hexapod.ino`; helpers in `functions.ino`.
- Avoid dynamic allocation, exceptions, RTTI, and heavy STL. Prefer fixed-size buffers and stack/static storage.
- Use integers for telemetry (mV, 0.1°C, centidegrees) and keep prints in `F()` PROGMEM where possible.
- Maintain the 166 Hz tick budget; keep IO non-blocking. Defer noncritical work if over budget.

## Coding style
- Follow `docs/STYLE.md`.
- Prefer `constexpr`, `inline`, narrow types, and simple helpers over deep abstractions.
- No `String`. Use `char[]` + `snprintf`.
- Minimize public surface and compilation units to keep flash usage low.

## Versioning & changelogs (mandatory)
- After completing any TODO item or user-facing behavior change:
  - Bump `FW_VERSION` (SemVer; patch by default) in `MARS_Hexapod.ino`.
  - Update the short change log block at the top of `MARS_Hexapod.ino`.
  - Update `CHANGELOG.md` with a concise entry.
  - Ensure splash/STATUS print the new version.

## Docs & TODO discipline
- Keep `docs/TODO.md` as the source of truth for open tasks.
  - Update it when adding, changing, or completing tasks.
- Mirror important behavior/interface changes in `docs/PROJECT_SPEC.md`.
- Keep `HELP` output in sync with implemented serial commands.

## Branching & PRs
- Work in topic branches; open PRs into `main`.
- This repo requires PR review (1 approval) and disallows force pushes/deletions on `main`.
- Explain:
  - What changed and why
  - Acceptance criteria and how you verified it (build/tests/device-level checks)
  - Any follow-ups or risk areas

## Commit messages
- Use imperative mood; include the area and a brief summary, e.g.,
  - `TEST: add OVERLAP and persist to config`
  - `STATUS: uptime wrap-safe accumulator`

## Testing
- Prefer tiny, fast, device-level checks where feasible.
- Validate build under Arduino 1.8.19 + Teensyduino.
- If behavior changes, add or update minimal tests or validation notes in the PR.

## Config & persistence
- Only add config keys actually used by code.
- Parse `key=value` pairs; ignore blanks and comments starting with `#`.
- Provide safe defaults; never crash on bad config.

## Safety
- Enforce soft limits, collision lockout (when implemented), and E‑stop rules.
- Motion commands should be ignored when disabled; clamp or reject out-of-bounds.

Thanks for contributing and keeping the robot upright and predictable!