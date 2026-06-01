#pragma once

#include <stddef.h>
#include <stdint.h>

enum class Cmd : uint8_t {
    DriveHoldYaw,
    StrafeHoldYaw,
    TurnToYaw,
    VisionPickup,
    IntakeOn,
    IntakeOff,
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
