# Safety Notes

ITOBOTAutoRuntime is intentionally conservative, but it cannot make a no-odometry robot perfectly accurate.

## No Odometry

Studio export is timed and gyro-assisted. The runtime does not know actual field position. Wheel slip, carpet, battery voltage, collisions, and calibration drift can move the robot away from the simulated path.

## Gyro Required For Yaw Features

`DriveHoldYaw`, `StrafeHoldYaw`, and `TurnToYaw` depend on a valid yaw value in `AutoRuntimeInput.yawDeg`. If yaw is stale or inverted, heading hold and turns will be wrong.

## Timeouts

`TurnToYaw` and `VisionPickup` are protected by `commandTimeoutMs`.

- Default: `5000ms`
- `0`: disable timeout
- `stopOnTimeout=true`: enter Error and stop drive
- `stopOnTimeout=false`: stop the timed-out command, report the issue, and skip to the next command

Use timeout disable only when your firmware has an independent safety path.

## Callback Safety

`stopDrive` is required during runtime updates. The runtime calls it on finish, error, and cancel. Missing callbacks are reported as errors instead of causing null-pointer calls.

`updateVisionPickup` is optional, but if used it should be fast and non-blocking. The runtime calls it once per `autoRunner.update()` tick while `Cmd::VisionPickup` is active, so long delays inside the callback will delay timeout checks and command sequencing.

`setLift` is required only when a routine contains `LiftOn` or `LiftOff`. Those commands finish immediately after calling the callback. If `setLift` is configured, the runtime also calls `setLift(0.0f)` on final stop, cancel, error, and timeout so the lift is not left running.

## Power Limits

Movement power and turn output are clamped to `[-1, 1]`. Heading corrections are also clamped by `maxHeadingCorrection`.

## Field Testing

Always test at low power first. Verify:

- Positive omega increases yaw, or invert it in the adapter.
- `TurnToYaw` turns the shortest direction.
- `stopDrive` immediately stops all drivetrain motors.
- Vision timeout behavior is acceptable for match play.
