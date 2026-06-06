#pragma once

#include <stdint.h>

struct AutoRuntimeConfig {
    float turnKp;
    float turnToleranceDeg;
    float maxTurnPower;
    float minTurnPower;
    float headingHoldKp;
    float maxHeadingCorrection;
    uint32_t commandTimeoutMs;
    uint32_t yawStaleTimeoutMs;
    bool stopOnTimeout;
    bool visionOwnsDrive;

    AutoRuntimeConfig()
        : turnKp(0.025f),
          turnToleranceDeg(2.0f),
          maxTurnPower(0.45f),
          minTurnPower(0.12f),
          headingHoldKp(0.018f),
          maxHeadingCorrection(0.25f),
          commandTimeoutMs(5000),
          yawStaleTimeoutMs(0),
          stopOnTimeout(true),
          visionOwnsDrive(false) {}
};
