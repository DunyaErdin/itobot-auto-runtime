#include <stdio.h>
#include <math.h>
#include "ITOBOTAutoRuntime.h"

struct FakeIoState {
    int driveCalls;
    int stopCalls;
    int intakeCalls;
    int dropCalls;
    int visionStartCalls;
    int visionUpdateCalls;
    int visionCancelCalls;
    int commandStartCalls;
    int commandFinishCalls;
    int errorCalls;
    float lastVx;
    float lastVy;
    float lastOmega;
    float lastIntake;
    const char* lastError;
};

static FakeIoState gIo;
static int gFailures = 0;

static void resetIo() {
    gIo.driveCalls = 0;
    gIo.stopCalls = 0;
    gIo.intakeCalls = 0;
    gIo.dropCalls = 0;
    gIo.visionStartCalls = 0;
    gIo.visionUpdateCalls = 0;
    gIo.visionCancelCalls = 0;
    gIo.commandStartCalls = 0;
    gIo.commandFinishCalls = 0;
    gIo.errorCalls = 0;
    gIo.lastVx = 0.0f;
    gIo.lastVy = 0.0f;
    gIo.lastOmega = 0.0f;
    gIo.lastIntake = 0.0f;
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

static AutoRuntimeIO makeIo(bool includeDrive = true, bool includeVisionUpdate = true) {
    AutoRuntimeIO io;
    io.driveMecanum = includeDrive ? fakeDrive : 0;
    io.stopDrive = fakeStop;
    io.setIntake = fakeIntake;
    io.drop = fakeDrop;
    io.startVisionPickup = fakeVisionStart;
    io.updateVisionPickup = includeVisionUpdate ? fakeVisionUpdate : 0;
    io.cancelVisionPickup = fakeVisionCancel;
    io.onCommandStart = fakeCommandStart;
    io.onCommandFinish = fakeCommandFinish;
    io.onError = fakeError;
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
}

int main() {
    testMath();
    testTurnDoesNotFinishInstantly();
    testTurnFinishesOnTolerance();
    testDriveFinishesAfterDurationAndClampsPower();
    testWaitFinishesAfterDuration();
    testStopFinishesRoutine();
    testNullAndZeroSafety();
    testMissingRequiredCallback();
    testTimeoutSafety();
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
