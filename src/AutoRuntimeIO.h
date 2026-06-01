#pragma once

#include <stddef.h>
#include <stdint.h>
#include "AutoCommand.h"

struct AutoRuntimeInput {
    uint32_t nowMs;
    float yawDeg;
    bool visionPickupFinished;

    AutoRuntimeInput()
        : nowMs(0),
          yawDeg(0.0f),
          visionPickupFinished(false) {}
};

struct AutoRuntimeIO {
    void (*driveMecanum)(float vx, float vy, float omega);
    void (*stopDrive)();
    void (*setIntake)(float power);
    void (*drop)();
    void (*startVisionPickup)();
    void (*updateVisionPickup)();
    void (*cancelVisionPickup)();
    void (*onCommandStart)(size_t index, Cmd command);
    void (*onCommandFinish)(size_t index, Cmd command);
    void (*onError)(const char* message);

    AutoRuntimeIO()
        : driveMecanum(0),
          stopDrive(0),
          setIntake(0),
          drop(0),
          startVisionPickup(0),
          updateVisionPickup(0),
          cancelVisionPickup(0),
          onCommandStart(0),
          onCommandFinish(0),
          onError(0) {}
};

inline void clearAutoRuntimeIO(AutoRuntimeIO& io) {
    io = AutoRuntimeIO();
}
