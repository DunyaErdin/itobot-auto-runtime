#include <stdio.h>
#include <math.h>
#include "ITOBOTAutoRuntime.h"

struct FakeIoState {
    int driveCalls;
    int stopCalls;
    int intakeCalls;
    int liftCalls;
    int dropCalls;
    int visionStartCalls;
    int visionUpdateCalls;
    int visionCancelCalls;
    int commandStartCalls;
    int commandFinishCalls;
    int errorCalls;
    int safetyEventCalls;
    bool abortRequested;
    float lastVx;
    float lastVy;
    float lastOmega;
    float lastIntake;
    float lastLift;
    const char* lastError;
};

static FakeIoState gIo;
static int gFailures = 0;

static void resetIo() {
    gIo.driveCalls = 0;
    gIo.stopCalls = 0;
    gIo.intakeCalls = 0;
    gIo.liftCalls = 0;
    gIo.dropCalls = 0;
    gIo.visionStartCalls = 0;
    gIo.visionUpdateCalls = 0;
    gIo.visionCancelCalls = 0;
    gIo.commandStartCalls = 0;
    gIo.commandFinishCalls = 0;
    gIo.errorCalls = 0;
    gIo.safetyEventCalls = 0;
    gIo.abortRequested = false;
    gIo.lastVx = 0.0f;
    gIo.lastVy = 0.0f;
    gIo.lastOmega = 0.0f;
    gIo.lastIntake = 0.0f;
    gIo.lastLift = 0.0f;
    gIo.lastError = 0;
}

static void expectTrue(bool condition, const char* message) {
    if (!condition) {
        ++gFailures;
        printf("FAIL: %s\n", message);
    }
}

static void expectNear(float actual, float expected, float tolerance, const char* message) {
    if (fabsf(actual - expected) > tolerance) {
        ++gFailures;
        printf("FAIL: %s actual=%f expected=%f\n", message, actual, expected);
    }
}

static void fakeDrive(float vx, float vy, float omega) {
    ++gIo.driveCalls;
    gIo.lastVx = vx;
    gIo.lastVy = vy;
    gIo.lastOmega = omega;
}

static void fakeStop() {
    ++gIo.stopCalls;
}

static void fakeIntake(float power) {
    ++gIo.intakeCalls;
    gIo.lastIntake = power;
}

static void fakeLift(float power) {
    ++gIo.liftCalls;
    gIo.lastLift = power;
}

static void fakeDrop() {
    ++gIo.dropCalls;
}

static void fakeVisionStart() {
    ++gIo.visionStartCalls;
}

static void fakeVisionUpdate() {
    ++gIo.visionUpdateCalls;
}

static void fakeVisionCancel() {
    ++gIo.visionCancelCalls;
}

static void fakeCommandStart(size_t, Cmd) {
    ++gIo.commandStartCalls;
}

static void fakeCommandFinish(size_t, Cmd) {
    ++gIo.commandFinishCalls;
}

static void fakeError(const char* message) {
    ++gIo.errorCalls;
    gIo.lastError = message;
}

static bool fakeShouldAbort() {
    return gIo.abortRequested;
}

static void fakeSafetyEvent(const char*) {
    ++gIo.safetyEventCalls;
}

static AutoRuntimeIO makeIo(bool includeDrive = true, bool includeVisionUpdate = true, bool includeLift = true) {
    AutoRuntimeIO io;
    io.driveMecanum = includeDrive ? fakeDrive : 0;
    io.stopDrive = fakeStop;
    io.setIntake = fakeIntake;
    io.setLift = includeLift ? fakeLift : 0;
    io.drop = fakeDrop;
    io.startVisionPickup = fakeVisionStart;
    io.updateVisionPickup = includeVisionUpdate ? fakeVisionUpdate : 0;
    io.cancelVisionPickup = fakeVisionCancel;
    io.onCommandStart = fakeCommandStart;
    io.onCommandFinish = fakeCommandFinish;
    io.onError = fakeError;
    io.shouldAbort = fakeShouldAbort;
    io.onSafetyEvent = fakeSafetyEvent;
    return io;
}

static AutoRuntimeInput inputAt(uint32_t nowMs, float yawDeg, bool visionDone = false) {
    AutoRuntimeInput input;
    input.nowMs = nowMs;
    input.yawDeg = yawDeg;
    input.visionPickupFinished = visionDone;
    return input;
}

static void testMath() {
    expectNear(normalizeAngleDeg(181.0f), -179.0f, 0.001f, "normalize 181");
    expectNear(normalizeAngleDeg(-181.0f), 179.0f, 0.001f, "normalize -181");
    expectNear(shortestAngleErrorDeg(10.0f, 350.0f), 20.0f, 0.001f, "shortest angle wraps positive");
    expectNear(clampFloat(2.0f, -1.0f, 1.0f), 1.0f, 0.001f, "clamp upper");
}

static void testTurnDoesNotFinishInstantly() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::TurnToYaw, 90.0f, 0.0f, 0 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 2);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(runner.isRunning(), "TurnToYaw should keep running while yaw is far");
    expectTrue(runner.activeIndex() == 0, "TurnToYaw should remain active");
    expectTrue(gIo.driveCalls == 1, "TurnToYaw should drive turn output");
    expectTrue(gIo.lastOmega > 0.0f, "TurnToYaw omega should be positive when target yaw is ahead");
}

static void testTurnFinishesOnTolerance() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::TurnToYaw, 90.0f, 0.0f, 0 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 2);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    runner.update(inputAt(20, 89.0f), io);
    expectTrue(runner.isFinished(), "TurnToYaw should finish when inside tolerance");
    expectTrue(gIo.stopCalls > 0, "TurnToYaw finish should stop drive");
}

static void testDriveFinishesAfterDurationAndClampsPower() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::DriveHoldYaw, 2.0f, 0.0f, 100 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 2);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    expectNear(gIo.lastVx, 1.0f, 0.001f, "Drive power should clamp to 1");
    runner.update(inputAt(99, 0.0f), io);
    expectTrue(runner.isRunning(), "Drive should run before duration");
    runner.update(inputAt(100, 0.0f), io);
    expectTrue(runner.isFinished(), "Drive should finish after duration and process Stop");
}

static void testWaitFinishesAfterDuration() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::Wait, 0.0f, 0.0f, 50 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 2);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(runner.isRunning(), "Wait should run at start");
    runner.update(inputAt(50, 0.0f), io);
    expectTrue(runner.isFinished(), "Wait should finish after duration");
}

static void testStopFinishesRoutine() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 1);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(runner.isFinished(), "Stop should finish routine");
    expectTrue(gIo.stopCalls > 0, "Stop should stop drive");
}

static void testLiftCommands() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::LiftOn, 0.0f, 0.0f, 0 },
        { Cmd::Wait, 0.0f, 0.0f, 250 },
        { Cmd::LiftOff, 0.0f, 0.0f, 0 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 4);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(gIo.liftCalls >= 1, "LiftOn should call setLift");
    expectNear(gIo.lastLift, 1.0f, 0.001f, "LiftOn should set lift to 1");
    runner.update(inputAt(250, 0.0f), io);
    expectTrue(runner.isFinished(), "Lift routine should finish");
    expectNear(gIo.lastLift, 0.0f, 0.001f, "LiftOff/final Stop should set lift to 0");
}

static void testMissingLiftCallbackErrorsSafely() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::LiftOn, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 1);
    const AutoRuntimeIO io = makeIo(true, true, false);
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(runner.hasError(), "missing setLift callback should error safely");
    expectTrue(gIo.errorCalls == 1, "missing setLift callback should report error");
    expectTrue(gIo.stopCalls > 0, "missing setLift callback should stop drive");
}

static void testNullAndZeroSafety() {
    AutoRunner runner;
    runner.begin(0, 1);
    expectTrue(runner.hasError(), "null commands with count should error");
    runner.begin(0, 0);
    expectTrue(runner.isFinished(), "zero count should finish safely");
}

static void testMissingRequiredCallback() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::DriveHoldYaw, 0.2f, 0.0f, 100 }
    };
    AutoRunner runner;
    runner.begin(commands, 1);
    const AutoRuntimeIO io = makeIo(false);
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(runner.hasError(), "missing drive callback should error");
    expectTrue(gIo.errorCalls == 1, "missing drive callback should report error");
}

static void testTimeoutSafety() {
    resetIo();
    AutoRuntimeConfig config;
    config.commandTimeoutMs = 50;
    const AutoCommand commands[] = {
        { Cmd::TurnToYaw, 90.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 1, config);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    runner.update(inputAt(49, 0.0f), io);
    expectTrue(runner.isRunning(), "turn should run before timeout");
    runner.update(inputAt(50, 0.0f), io);
    expectTrue(runner.hasError(), "turn should timeout safely");
    expectTrue(gIo.stopCalls > 0, "timeout should stop drive");
    expectNear(gIo.lastLift, 0.0f, 0.001f, "timeout should stop lift when callback exists");
}

static void testInvalidYawErrorsSafely() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::DriveHoldYaw, 0.2f, 0.0f, 100 }
    };
    AutoRunner runner;
    runner.begin(commands, 1);
    const AutoRuntimeIO io = makeIo();
    AutoRuntimeInput input = inputAt(0, 0.0f);
    input.yawValid = false;
    runner.update(input, io);
    expectTrue(runner.hasError(), "invalid yaw should error safely");
    expectTrue(gIo.stopCalls > 0, "invalid yaw should stop drive");
    expectTrue(gIo.driveCalls == 0, "invalid yaw should not drive");
}

static void testStaleYawErrorsSafely() {
    resetIo();
    AutoRuntimeConfig config;
    config.yawStaleTimeoutMs = 25;
    const AutoCommand commands[] = {
        { Cmd::TurnToYaw, 30.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 1, config);
    const AutoRuntimeIO io = makeIo();
    AutoRuntimeInput input = inputAt(100, 0.0f);
    input.yawTimestampMs = 50;
    runner.update(input, io);
    expectTrue(runner.hasError(), "stale yaw should error safely");
    expectTrue(gIo.stopCalls > 0, "stale yaw should stop drive");
}

static void testNanCommandErrorsSafely() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::DriveHoldYaw, NAN, 0.0f, 100 }
    };
    AutoRunner runner;
    runner.begin(commands, 1);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(runner.hasError(), "NaN command should error safely");
    expectTrue(gIo.stopCalls > 0, "NaN command should stop drive");
    expectTrue(gIo.driveCalls == 0, "NaN command should not drive");
}

static void testAbortCallbackStopsDrive() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::DriveHoldYaw, 0.2f, 0.0f, 100 }
    };
    AutoRunner runner;
    runner.begin(commands, 1);
    const AutoRuntimeIO io = makeIo();
    gIo.abortRequested = true;
    runner.update(inputAt(0, 0.0f), io);
    expectTrue(runner.hasError(), "abort callback should enter error state");
    expectTrue(gIo.safetyEventCalls == 1, "abort callback should report safety event");
    expectTrue(gIo.stopCalls > 0, "abort callback should stop drive");
}

static void testVisionWaitsForCompletion() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::VisionPickup, 0.0f, 0.0f, 0 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 2);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f, false), io);
    runner.update(inputAt(100, 0.0f, false), io);
    expectTrue(runner.isRunning(), "VisionPickup should wait while not finished");
    expectTrue(gIo.visionStartCalls == 1, "VisionPickup should start once");
    expectTrue(gIo.visionUpdateCalls == 2, "VisionPickup should update every active tick");
    runner.update(inputAt(200, 0.0f, true), io);
    expectTrue(runner.isFinished(), "VisionPickup should finish on input flag");
    expectTrue(gIo.visionStartCalls == 1, "VisionPickup should not restart on finish tick");
    expectTrue(gIo.visionUpdateCalls == 3, "VisionPickup should update on the finish tick before checking completion");
    runner.update(inputAt(300, 0.0f, true), io);
    expectTrue(gIo.visionUpdateCalls == 3, "VisionPickup update should stop after command finishes");
}

static void testVisionNullUpdateDoesNotCrash() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::VisionPickup, 0.0f, 0.0f, 0 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 2);
    const AutoRuntimeIO io = makeIo(true, false);
    runner.update(inputAt(0, 0.0f, false), io);
    expectTrue(runner.isRunning(), "VisionPickup with null update should keep waiting");
    runner.update(inputAt(20, 0.0f, true), io);
    expectTrue(runner.isFinished(), "VisionPickup with null update should finish on input flag");
    expectTrue(gIo.errorCalls == 0, "VisionPickup with null update should not error");
}

static void testVisionStartIsOptional() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::VisionPickup, 0.0f, 0.0f, 0 },
        { Cmd::Stop, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 2);
    AutoRuntimeIO io = makeIo();
    io.startVisionPickup = 0;
    runner.update(inputAt(0, 0.0f, false), io);
    expectTrue(runner.isRunning(), "VisionPickup with null start should keep waiting");
    runner.update(inputAt(20, 0.0f, true), io);
    expectTrue(runner.isFinished(), "VisionPickup with null start should finish on input flag");
    expectTrue(gIo.errorCalls == 0, "VisionPickup with null start should not error");
    expectTrue(gIo.visionUpdateCalls == 2, "VisionPickup with null start should still call update");
}

static void testVisionTimeoutCancelsPickup() {
    resetIo();
    AutoRuntimeConfig config;
    config.commandTimeoutMs = 50;
    const AutoCommand commands[] = {
        { Cmd::VisionPickup, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 1, config);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f, false), io);
    runner.update(inputAt(50, 0.0f, false), io);
    expectTrue(runner.hasError(), "VisionPickup should timeout safely");
    expectTrue(gIo.visionCancelCalls == 1, "VisionPickup timeout should cancel vision");
    expectTrue(gIo.stopCalls > 0, "VisionPickup timeout should stop drive");
}

static void testCancelStopsDriveAndVision() {
    resetIo();
    const AutoCommand commands[] = {
        { Cmd::VisionPickup, 0.0f, 0.0f, 0 }
    };
    AutoRunner runner;
    runner.begin(commands, 1);
    const AutoRuntimeIO io = makeIo();
    runner.update(inputAt(0, 0.0f, false), io);
    runner.cancel(io);
    expectTrue(runner.isCancelled(), "cancel should enter Cancelled state");
    expectTrue(gIo.stopCalls > 0, "cancel should stop drive");
    expectTrue(gIo.visionCancelCalls == 1, "cancel should cancel active vision");
    expectNear(gIo.lastLift, 0.0f, 0.001f, "cancel should stop lift when callback exists");
}

int main() {
    testMath();
    testTurnDoesNotFinishInstantly();
    testTurnFinishesOnTolerance();
    testDriveFinishesAfterDurationAndClampsPower();
    testWaitFinishesAfterDuration();
    testStopFinishesRoutine();
    testLiftCommands();
    testMissingLiftCallbackErrorsSafely();
    testNullAndZeroSafety();
    testMissingRequiredCallback();
    testTimeoutSafety();
    testInvalidYawErrorsSafely();
    testStaleYawErrorsSafely();
    testNanCommandErrorsSafely();
    testAbortCallbackStopsDrive();
    testVisionWaitsForCompletion();
    testVisionNullUpdateDoesNotCrash();
    testVisionStartIsOptional();
    testVisionTimeoutCancelsPickup();
    testCancelStopsDriveAndVision();

    if (gFailures == 0) {
        printf("AutoRunnerTests passed\n");
        return 0;
    }

    printf("AutoRunnerTests failed: %d\n", gFailures);
    return 1;
}
