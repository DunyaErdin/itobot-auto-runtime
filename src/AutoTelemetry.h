#pragma once

#include <stddef.h>
#include <stdint.h>
#include "AutoCommand.h"

struct AutoTelemetry {
    bool running;
    bool finished;
    bool error;
    size_t activeIndex;
    Cmd activeCommand;
    uint32_t commandElapsedMs;
    float yawDeg;
    float yawErrorDeg;

    AutoTelemetry()
        : running(false),
          finished(false),
          error(false),
          activeIndex(0),
          activeCommand(Cmd::Stop),
          commandElapsedMs(0),
          yawDeg(0.0f),
          yawErrorDeg(0.0f) {}
};
