#pragma once

#include <stddef.h>
#include <stdint.h>
#include "AutoCommand.h"

struct AutoTelemetry {
    bool running;
    bool finished;
    bool error;
    bool cancelled;
    size_t activeIndex;
    size_t commandCount;
    Cmd activeCommand;
    uint32_t commandElapsedMs;
    float yawDeg;
    float yawErrorDeg;
    float routineProgress;
    bool visionActive;
    const char* lastError;

    AutoTelemetry()
        : running(false),
          finished(false),
          error(false),
          cancelled(false),
          activeIndex(0),
          commandCount(0),
          activeCommand(Cmd::Stop),
          commandElapsedMs(0),
          yawDeg(0.0f),
          yawErrorDeg(0.0f),
          routineProgress(0.0f),
          visionActive(false),
          lastError(0) {}
};
