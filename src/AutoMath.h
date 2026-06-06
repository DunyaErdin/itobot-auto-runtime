#pragma once

#include <math.h>

inline float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

inline bool isFiniteFloat(float value) {
    return isfinite(value) != 0;
}

inline float normalizeAngleDeg(float angle) {
    float out = fmodf(angle, 360.0f);
    if (out >= 180.0f) out -= 360.0f;
    if (out < -180.0f) out += 360.0f;
    if (out > -0.000001f && out < 0.000001f) out = 0.0f;
    return out;
}

inline float shortestAngleErrorDeg(float target, float current) {
    return normalizeAngleDeg(target - current);
}
