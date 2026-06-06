# ITOBOTAutoRuntime Integration Guide

## Include Pattern

Generated headers only need the shared command contract:

```cpp
#include <ITOBOTAutoRuntime.h>
#include "generated/my_auto.h"
```

If a generated header uses shared metadata structs from the runtime, include `AutoMetadata.h` from the generated header or before using metadata.

## Required Loop

```cpp
AutoRunner autoRunner;
AutoRuntimeIO autoIO;

void autonomousInit() {
  configureRuntimeIo();
  autoRunner.begin(MY_AUTO_AUTO, MY_AUTO_AUTO_COUNT);
}

void autonomousLoop() {
  updateGyro();
  processVisionSerial();

  AutoRuntimeInput input;
  input.nowMs = millis();
  input.yawDeg = currentYaw;
  input.visionPickupFinished = isVisionPickupFinished();

  autoRunner.update(input, autoIO);
}
```

## Mecanum Adapter

```cpp
void runtimeDriveMecanum(float vx, float vy, float omega) {
  driveMecanum({ vx, vy, omega });
}

void runtimeStopDrive() {
  stopMotors();
}
```

Positive `omega` should increase yaw. If your drivetrain function uses the opposite convention, pass `-omega`.

## Tank Adapter

Tank firmware can ignore `vy`. The runtime will still call the generic drive callback so the generated command contract stays shared.

```cpp
void runtimeDriveMecanum(float vx, float vy, float omega) {
  (void)vy;
  const float left = constrain(vx - omega, -1.0f, 1.0f);
  const float right = constrain(vx + omega, -1.0f, 1.0f);
  setTankPower(left, right);
}
```

## Mechanism Adapters

`IntakeOn` / `IntakeOff` use `setIntake`. `LiftOn` / `LiftOff` use `setLift`.

```cpp
void runtimeSetIntake(float power) {
  setSingleMotorI2C(INTAKE, power);
}

void runtimeSetLift(float power) {
  writeSpark(LIFT, power);
}

void runtimeDrop() {
  stopMotors();
  setSingleMotorI2C(INTAKE, 0.0f);
  // Trigger your real drop actuator here if one exists.
}
```

`LiftOn` calls `setLift(1.0f)` and finishes immediately. `LiftOff` calls `setLift(0.0f)` and finishes immediately. On finish, error, cancel, and timeout, the runtime also calls `setLift(0.0f)` when the callback is configured.

## Vision Pickup Adapter

```cpp
void runtimeStartVisionPickup() {
  resetVisionTarget();
  setVisionState(VISION_SEARCH);
}

void runtimeUpdateVisionPickup() {
  runVisionAutonomy();
}

void runtimeCancelVisionPickup() {
  stopMotors();
  setSingleMotorI2C(INTAKE, 0.0f);
  setVisionState(VISION_IDLE);
}

bool isVisionPickupFinished() {
  return visionState == VISION_DROP;
}
```

`runtimeStartVisionPickup()` is called once when the command starts. `runtimeUpdateVisionPickup()` is optional and is called by the runtime on every `autoRunner.update()` tick while `Cmd::VisionPickup` is active. The command finishes only when the next input snapshot reports `visionPickupFinished = true`.

Set `config.visionOwnsDrive = true` when the vision state machine drives the robot itself. Leave it false when the robot should remain stopped while waiting for vision completion.

## Callback Setup

```cpp
AutoRuntimeIO autoIO;

void configureRuntimeIo() {
  clearAutoRuntimeIO(autoIO);
  autoIO.driveMecanum = runtimeDriveMecanum;
  autoIO.stopDrive = runtimeStopDrive;
  autoIO.setIntake = runtimeSetIntake;
  autoIO.setLift = runtimeSetLift;
  autoIO.drop = runtimeDrop;
  autoIO.startVisionPickup = runtimeStartVisionPickup;
  autoIO.updateVisionPickup = runtimeUpdateVisionPickup;
  autoIO.cancelVisionPickup = runtimeCancelVisionPickup;
  autoIO.onError = runtimeError;
}
```

`onCommandStart`, `onCommandFinish`, and `onError` are optional, but useful for Driver Station/Serial debugging.
