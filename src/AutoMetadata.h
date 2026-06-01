#pragma once

#include "AutoCommand.h"

#ifndef ITOBOT_AUTO_METADATA_TYPES_DEFINED
#define ITOBOT_AUTO_METADATA_TYPES_DEFINED

struct AutoEventInfo {
    const char* id;
    const char* label;
    const char* type;
    float pathDistanceMm;
    float timeSec;
    float xMm;
    float yMm;
    float headingDeg;
};

struct AutoPathInfo {
    const char* name;
    const char* driveBehavior;
    float totalDistanceMm;
    float estimatedDurationSec;
    float robotWidthMm;
    float robotLengthMm;
    float robotMassKg;
};

struct AutoRoutineRef {
    const char* name;
    const AutoCommand* commands;
    size_t commandCount;
};

#endif
