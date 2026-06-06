#include <Arduino.h>
#include <ITOBOTAutoRuntime.h>

static const AutoCommand BASIC_AUTO[] = {
    { Cmd::DriveHoldYaw, 0.30f, 0.0f, 1200 },
    { Cmd::TurnToYaw, 90.0f, 0.0f, 0 },
    { Cmd::IntakeOn, 0.0f, 0.0f, 0 },
    { Cmd::Wait, 0.0f, 0.0f, 500 },
    { Cmd::IntakeOff, 0.0f, 0.0f, 0 },
    { Cmd::LiftOn, 0.0f, 0.0f, 0 },
    { Cmd::Wait, 0.0f, 0.0f, 300 },
    { Cmd::LiftOff, 0.0f, 0.0f, 0 },
    { Cmd::Stop, 0.0f, 0.0f, 0 }
};
static const size_t BASIC_AUTO_COUNT = sizeof(BASIC_AUTO) / sizeof(BASIC_AUTO[0]);

AutoRunner autoRunner;
AutoRuntimeIO autoIO;

float fakeYawDeg = 0.0f;

void runtimeDriveMecanum(float vx, float vy, float omega) {
    fakeYawDeg += omega * 4.0f;
    Serial.print("drive vx=");
    Serial.print(vx, 3);
    Serial.print(" vy=");
    Serial.print(vy, 3);
    Serial.print(" omega=");
    Serial.println(omega, 3);
}

void runtimeStopDrive() {
    Serial.println("stop drive");
}

void runtimeSetIntake(float power) {
    Serial.print("intake=");
    Serial.println(power, 2);
}

void runtimeSetLift(float power) {
    Serial.print("lift=");
    Serial.println(power, 2);
}

void runtimeDrop() {
    Serial.println("drop");
}

void runtimeStartVisionPickup() {
    Serial.println("vision start");
}

void runtimeUpdateVisionPickup() {
    Serial.println("vision update");
}

void runtimeCancelVisionPickup() {
    Serial.println("vision cancel");
}

void runtimeCommandStart(size_t index, Cmd command) {
    Serial.print("command start ");
    Serial.print(index);
    Serial.print(" type=");
    Serial.println(static_cast<int>(command));
}

void runtimeCommandFinish(size_t index, Cmd command) {
    Serial.print("command finish ");
    Serial.print(index);
    Serial.print(" type=");
    Serial.println(static_cast<int>(command));
}

void runtimeError(const char* message) {
    Serial.print("auto error: ");
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
    autoIO.onCommandStart = runtimeCommandStart;
    autoIO.onCommandFinish = runtimeCommandFinish;
    autoIO.onError = runtimeError;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    configureRuntimeIo();
    autoRunner.begin(BASIC_AUTO, BASIC_AUTO_COUNT);
}

void loop() {
    AutoRuntimeInput input;
    input.nowMs = millis();
    input.yawDeg = fakeYawDeg;
    input.visionPickupFinished = false;

    autoRunner.update(input, autoIO);

    const AutoTelemetry telemetry = autoRunner.telemetry();
    Serial.print("active=");
    Serial.print(telemetry.activeIndex);
    Serial.print(" elapsed=");
    Serial.print(telemetry.commandElapsedMs);
    Serial.print(" yawError=");
    Serial.println(telemetry.yawErrorDeg, 2);

    delay(100);
}
