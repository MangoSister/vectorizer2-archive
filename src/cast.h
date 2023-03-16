#pragma once

#include "maths.h"

namespace vtrz
{

constexpr double kScaleFactor = 1LL << 20;
constexpr double kInvScaleFactor = 1.0 / kScaleFactor;

inline int64_t cast(double x) {
    return std::llround(x * kScaleFactor);
}

inline double cast(int64_t x) {
    return (double)x * kInvScaleFactor;
}

inline Vector2I64 cast(const Vector2 &v) {
    return Vector2I64(
        std::llround((double)v.x * kScaleFactor),
        std::llround((double)v.y * kScaleFactor));
}

inline Vector2 cast(const Vector2I64 &v) {
    return Vector2(
        (float)((double)v.x * kInvScaleFactor),
        (float)((double)v.y * kInvScaleFactor));
}

}