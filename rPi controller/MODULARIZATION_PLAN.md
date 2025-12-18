# Controller.py Modularization Plan

**Created**: 2025-12-18
**Completed**: 2025-12-18
**Status**: ✅ COMPLETE (7 phases done)
**Original size**: ~4844 lines
**Final size**: ~3825 lines (~21% reduction)

## Overview

The `controller.py` file was reduced from ~4844 to ~3825 lines through a phased modularization approach. Six new modules were created with clear responsibilities.

---

## Final Module Structure

```
rPi controller/
├── controller.py         # Main entry point (~3825 lines — Controller class + main loop)
├── telemetry.py          # Telemetry parsing, dataclasses, S1-S5 handlers (385 lines)
├── display_thread.py     # DisplayThread class, eye updates, LCD rendering (609 lines)
├── input_handler.py      # Gamepad/keyboard event processing (407 lines)
├── posture.py            # Posture commands, gait integration helpers (290 lines)
├── config_manager.py     # INI loading/saving, config validation (662 lines)
├── mars_state.py         # State container dataclasses (382 lines)
└── (existing modules remain):
    ├── gait_engine.py
    ├── SimpleEyes.py
    ├── MarsMenu.py
    ├── st7789.py
    └── ...
```

---

## Module Responsibilities

### 1. `telemetry.py` (~400 lines)
**Extract from controller.py lines ~200-600, 750-900**

Contents:
- Dataclasses: `SystemTelemetry`, `ServoTelemetry`, `LegTelemetry`, `SafetyTelemetry`
- IDX_* constants (telemetry field indices)
- `processTelemS1()`, `processTelemS2()`, `processTelemS3()`, `processTelemS4()`, `processTelemS5()`
- Binary telemetry parsers: `parseBinaryS1()`, etc.
- Telemetry validation helpers

Dependencies: None (pure data + parsing)

### 2. `display_thread.py` (~600 lines)
**Extract from controller.py lines ~1600-2200**

Contents:
- `DisplayThread` class
- `UpdateDisplay()` function
- Eye/LCD rendering helpers
- Overlay rendering (NO TEENSY, DISABLED, SAFETY)
- Frame hashing and change detection

Dependencies: `SimpleEyes`, `st7789`, `MarsMenu`, `telemetry` (for state reading)

### 3. `input_handler.py` (~500 lines)
**Extract from controller.py lines ~900-1400**

Contents:
- `poll_gamepad()` method logic (refactored as standalone)
- `poll_keyboard()` function
- Button/axis event mapping
- Gait control input processing
- Touch event handlers

Dependencies: `evdev`, `XBoxController`, `gait_engine`, `posture`

### 4. `posture.py` (~200 lines)
**Extract from controller.py scattered posture helpers**

Contents:
- `apply_posture()` function
- `ensure_enabled()` helper
- `start_pounce_move()` and kinematic move helpers
- Auto-disable scheduling logic
- Posture-related command wrappers

Dependencies: `send_cmd()`, `gait_engine`

### 5. `config_manager.py` (~300 lines)
**Extract from controller.py config handling**

Contents:
- `load_config()` function
- `save_gait_settings()`, `save_eye_settings()`, etc.
- Config validation and defaults
- INI section handlers

Dependencies: `configparser`

### 6. `mars_state.py` (~300 lines)
**Extract Controller class and global state**

Contents:
- `Controller` class definition
- Global state initialization (`_state`, `_servo`, `_legs`, `_safety_state`)
- State synchronization helpers
- `send_cmd()` function

Dependencies: `serial`, `telemetry`

### 7. `controller.py` (slim main, ~500 lines)
**Remaining orchestration**

Contents:
- Imports from all modules
- `main()` function
- Top-level loop structure
- Hardware initialization (LCD, touch, serial)
- Shutdown/cleanup

Dependencies: All modules above

---

## Implementation Phases

### Phase 1: Extract `telemetry.py` (Low Risk)
- Pure data structures and parsing functions
- No side effects
- Easy to test in isolation
- **Estimate**: 1 session

### Phase 2: Extract `config_manager.py` (Low Risk)
- Self-contained INI handling
- Clear interface boundaries
- **Estimate**: 0.5 session

### Phase 3: Extract `posture.py` (Medium Risk)
- Some coupling to gait_engine and send_cmd
- Requires careful import ordering
- **Estimate**: 0.5 session

### Phase 4: Extract `mars_state.py` (Medium Risk)
- Controller class is central
- Need to handle circular import potential
- **Estimate**: 1 session

### Phase 5: Extract `display_thread.py` (Medium Risk)
- Threading concerns
- Eye/menu integration
- **Estimate**: 1 session

### Phase 6: Extract `input_handler.py` (Higher Risk)
- Heavy coupling to state and commands
- Event loop integration
- **Estimate**: 1-2 sessions

### Phase 7: Final cleanup (Low Risk)
- Slim down controller.py main
- Update imports
- Integration testing
- **Estimate**: 0.5 session

---

## Migration Strategy

1. **Create module with extracted code**
2. **Add imports in controller.py** pointing to new module
3. **Keep original code commented** (one session) for rollback
4. **Test on hardware** before removing commented code
5. **Update CHANGELOG** after each phase

---

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Circular imports | Use late imports or dependency injection |
| Global state access | Pass state explicitly or use module-level singletons |
| Threading issues | Keep DisplayThread isolated; use Queue for cross-thread comms |
| Regression | Test each phase on hardware before proceeding |

---

## Success Criteria

- [x] `controller.py` reduced significantly (4844→3825, ~21% reduction)
- [x] Each module has clear single responsibility
- [x] No circular import errors
- [x] All existing functionality preserved
- [x] All modules have `__all__` exports for explicit API
- [ ] Hardware testing passes after each phase (requires physical robot)

---

## Notes

- The modularization does **not** require version bumps until behavioral changes occur
- Focus on one phase at a time to minimize risk
- Consider adding `__all__` exports to each module for explicit API
