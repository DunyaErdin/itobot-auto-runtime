#include <Arduino.h>
#include <ITOBOTAutoRuntime.h>
// #include "generated/my_auto.h"

enum VisionState {
    VISION_IDLE,
    VISION_SEARCH,
    VISION_PICKUP,
    VISION_DROP
};

static const AutoCommand VISION_AUTO[] = {
    { Cmd::DriveHoldYaw, 0.25f, 0.0f, 800 },
    { Cmd::VisionPickup, 0.0f, 0.0f, 0 },
    { Cmd::LiftOn, 0.0f, 0.0f, 0 },
    { Cmd::Wait, 0.0f, 0.0f, 300 },
    { Cmd::LiftOff, 0.0f, 0.0f, 0 },
    { Cmd::Drop, 0.0f, 0.0f, 0 },
    { Cmd::Stop, 0.0f, 0.0f, 0 }
};
static const size_t VISION_AUTO_COUNT = sizeof(VISION_AUTO) / sizeof(VISION_AUTO[0]);

AutoRunner autoRunner;
AutoRuntimeIO autoIO;
VisionState visionState = VISION_IDLE;
float currentYawDeg = 0.0f;

void driveMecanumRobot(float vx, float vy, float omega) {
    (void)vx;
    (void)vy;
    (void)omega;
    // Existing robot firmware should map vx/vy/omega to motor outputs here.
}

void stopMotors() {
    // Existing robot firmware stop function.
}

void setIntakePower(float power) {
    (void)power;
    // Existing intake control.
}

void setLiftPower(float power) {
    (void)power;
    // Existing lift motor control.
}

void dropGamePiece() {
    // Existing drop/servo command.
}

void setVisionState(VisionState state) {
    visionState = state;
}

void resetVisionTarget() {
    // Existing firmware can clear target filters/counters here.
}

void runVisionAutonomy() {
    if (visionState == VISION_SEARCH) {
        visionState = VISION_PICKUP;
    } else if (visionState == VISION_PICKUP) {
        visionState = VISION_DROP;
    }
}

bool isVisionPickupFinished() {
    return visionState == VISION_DROP;
}

void runtimeDriveMecanum(float vx, float vy, float omega) {
    driveMecanumRobot(vx, vy, omega);
}

void runtimeStopDrive() {
    stopMotors();
}

void runtimeStartVisionPickup() {
    resetVisionTarget();
    setVisionState(VISION_SEARCH);
}

void runtimeUpdateVisionPickup() {
    runVisionAutonomy();
}

void runtimeCancelVisionPickup() {
    stopMotors();
    setIntakePower(0.0f);
    setVisionState(VISION_IDLE);
}

void runtimeSetIntake(float power) {
    setIntakePower(power);
}

void runtimeSetLift(float power) {
    setLiftPower(power);
}

void runtimeDrop() {
    dropGamePiece();
}

void runtimeError(const char* message) {
    Serial.print("Auto runtime error: ");
    Serial.println(message);
}

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

void autonomousInit() {
    configureRuntimeIo();
    AutoRuntimeConfig config;
    config.visionOwnsDrive = true;
    config.commandTimeoutMs = 7000;
    autoRunner.begin(VISION_AUTO, VISION_AUTO_COUNT, config);
}

void autonomousLoop() {
    AutoRuntimeInput input;
    input.nowMs = millis();
    input.yawDeg = currentYawDeg;
    input.visionPickupFinished = isVisionPickupFinished();

    autoRunner.update(input, autoIO);
}

void setup() {
    Serial.begin(115200);
    autonomousInit();
}

void loop() {
    autonomousLoop();
}
