# Generated Header Contract

ITOBOT Auto Studio generated firmware headers are plain Arduino-compatible C++.

## Command Contract

The runtime owns the shared `AutoCommand.h` contract:

```cpp
enum class Cmd : uint8_t {
  DriveHoldYaw,
  StrafeHoldYaw,
  TurnToYaw,
  VisionPickup,
  IntakeOn,
  IntakeOff,
  LiftOn,
  LiftOff,
  Drop,
  Wait,
  Stop
};

struct AutoCommand {
  Cmd type;
  float a;
  float b;
  uint32_t durationMs;
};
```

New generated headers should include:

```cpp
#include <ITOBOTAutoRuntime.h>
```

`ITOBOTAutoRuntime.h` provides `AutoCommand`, `Cmd`, and the optional metadata structs used by generated headers. Old generated headers that only include `AutoCommand.h` and only contain `AutoCommand[]` arrays continue to work unchanged when the runtime library is installed.

Runtime callbacks such as `updateVisionPickup` are not part of generated headers. They are supplied by robot firmware through `AutoRuntimeIO`, so generated `AutoCommand` files do not need to change when the callback set grows.

## Metadata Contract

Generated metadata uses runtime-owned structs:

```cpp
AutoEventInfo
AutoPathInfo
AutoRoutineRef
```

Generated headers should not redefine these structs.

## Existing Metadata Headers

Some older generated headers may define metadata structs directly. Prefer regenerating those headers with current ITOBOT Auto Studio before including them together with `ITOBOTAutoRuntime.h`, so C++ struct redefinition conflicts are avoided.

## Routine Selection

Single routine:

```cpp
autoRunner.begin(RED_RIGHT_AUTO, RED_RIGHT_AUTO_COUNT);
```

Project registry:

```cpp
const AutoRoutineRef& routine = PROJECT_AUTOS[selectedIndex];
autoRunner.begin(routine.commands, routine.commandCount);
```
