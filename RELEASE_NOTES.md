# Release Notes

## v0.1.0

Initial public release candidate for ITOBOTAutoRuntime.

### Added

- Arduino-compatible `AutoCommand` runtime contract for ITOBOT Auto Studio generated headers.
- `AutoRunner` command sequencer for timed drive, strafe, turn, wait, intake, drop, stop, and vision pickup commands.
- Hardware-agnostic callback interface through `AutoRuntimeIO`.
- Configurable gyro-based `TurnToYaw` and heading hold behavior.
- Optional per-loop `updateVisionPickup` callback for existing robot vision autonomous state machines.
- Runtime metadata types: `AutoEventInfo`, `AutoPathInfo`, and `AutoRoutineRef`.
- Arduino examples:
  - `BasicAutoRunner`
  - `VisionPickupIntegration`
- Host test harness and Windows/Linux test scripts.
- Arduino ESP32 verification script.

### Safety Notes

- This runtime does not implement encoder odometry or onboard trajectory following.
- Studio generated routines execute as timed+gyro approximations.
- Robot teams must calibrate generated durations and validate routines at low power before match use.

### Arduino Library Manager Readiness

- Standard Arduino library layout is present.
- `library.properties` is included.
- `keywords.txt` is included.
- Examples are located under `examples/`.
