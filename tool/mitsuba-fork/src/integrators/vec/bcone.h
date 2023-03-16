#pragma once

#include <mitsuba/mitsuba.h>
#include <vector>

MTS_NAMESPACE_BEGIN

struct SphericalCircle
{
    Vector3 n;
    float cosHalfAngle;

    SphericalCircle(const Vector3 &n, float cosha) : n(n) {
        SAssert(cosha > 0.0f);
        cosHalfAngle = math::clamp(cosha, 0.0f, 1.0f);
    }

    // Assume normalized.
    SphericalCircle(const Vector3 &d0, const Vector3 &d1, const Vector3 &d2) {
        n = cross(d1 - d0, d2 - d1);
        Float len2 = n.lengthSquared();
        if (len2 > 0.0f) {
            n /= std::sqrt(len2);
            cosHalfAngle = dot(d0, n);
            if (cosHalfAngle < 0.0f) {
                cosHalfAngle = -cosHalfAngle;
                n = -n;
            }
        } else {
            n = d0 + d1 + d2;
            len2 = n.lengthSquared();
            SAssert(len2 > 0);
            n /= std::sqrt(len2);
            cosHalfAngle = std::min(std::min(dot(d0, n), dot(d1, n)), dot(d2, n));
        }
        SAssert(cosHalfAngle > 0.0f);
        cosHalfAngle = math::clamp(cosHalfAngle, 0.0f, 1.0f);
    }

    // Assume normalized.
    bool contain(const Vector3 &d) const {
        return dot(n, d) >= cosHalfAngle;
    }
};

// Barequet, Gill, and Gershon Elber. "Optimal bounding cones of vectors in three dimensions."
// Information Processing Letters 93.2 (2005): 83-89.
// !!Assume directions spans within a hemisphere (non-reflex cone).
SphericalCircle minBoundingCone(Vector *directions, uint32_t count);

MTS_NAMESPACE_END