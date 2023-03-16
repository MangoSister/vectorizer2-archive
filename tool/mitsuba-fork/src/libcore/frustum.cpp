/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// New file for Vectorizer integration.

#include <mitsuba/core/frustum.h>
//#include <mitsuba/core/sse.h>
//#include <mitsuba/core/ssemath.h>
//#include <mitsuba/core/ssevector.h>

MTS_NAMESPACE_BEGIN

Frustum::Frustum(const Point &apex, const Vector &centerDir, const Vector dirs[4], Float nearClip, FrustumAABBOption aabbOption) :
    apex(apex), centerDir(normalize(centerDir)) {
    for (uint32_t j = 0; j < 3; ++j) {
        allDirsPositive[j] = true;
        allDirsNegative[j] = true;
    }
    for (uint32_t i = 0; i < 4; ++i) {
        planes[i].normal = normalize(cross(dirs[i], dirs[(i + 1) % 4]));
        planes[i].dist = 0.0f;
        // planes[i].d = -dot(planes[i].normal, Vector(apex));
        for (uint32_t j = 0; j < 3; ++j) {
            planes[i].absNormal[j] = std::abs(planes[i].normal[j]);
            allDirsPositive[j] &= (dirs[i][j] >= 0.0);
            allDirsNegative[j] &= (dirs[i][j] <= 0.0);
        }
    }

    planes[4].normal = -centerDir;
    planes[4].dist = -nearClip;
    for (uint32_t j = 0; j < 3; ++j) {
        planes[4].absNormal[j] = std::abs(planes[4].normal[j]);
    }

    for (uint32_t i = 0; i < 4; ++i) {
        nearCornerBound.expandBy(apex + dirs[i] * (nearClip / dot(centerDir, dirs[i])));
    }

    if (aabbOption != FrustumAABBOption::eExact) {
        return;
    }

    static const Vector ortho[3] = {
        Vector(1.0f, 0.0f, 0.0f),
        Vector(0.0f, 1.0f, 0.0f),
        Vector(0.0f, 0.0f, 1.0f),
    };

    for (uint32_t i = 0; i < 4; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            Vector3 axis = cross(dirs[i], ortho[j]);
            Float len = axis.length();
            if (len == 0) continue;
            axis /= len;
            Float s = dot(axis, dirs[0]);
            bool valid = true;
            for (uint32_t k = 0; k < 4; ++k) {
                Float d = dot(axis, dirs[k]);
                if (s == 0.0f) s = d;
                else if (s * d < 0.0f) { valid = false; break; }
            }
            if (!valid || s == 0.0f) continue;

            axis = s > 0.0f ? axis : -axis;
            crossAxisRange[crossAxisCount] = dot(axis, (Vector)apex);
            crossAxis[crossAxisCount] = axis;
            ++crossAxisCount;
        }
    }
}

FrustumIsectResult frustumIntersectCoarse(const Frustum &frustum, const AABB &aabb)
{
    // 3 face normals from the aabb.
    for (uint32_t i = 0; i < 3; ++i) {
        // frustum corner dir.
        if (frustum.allDirsPositive[i] && frustum.nearCornerBound.min[i] > aabb.max[i]) {
            return FrustumIsectResult::EDisjoint;
        }
        if (frustum.allDirsNegative[i] && frustum.nearCornerBound.max[i] < aabb.min[i]) {
            return FrustumIsectResult::EDisjoint;
        }
    }

    Vector halfExtents = Float(0.5) * aabb.getExtents();
    Vector center = (Vector)aabb.getCenter();
    Vector centerOffset = (Point)center - frustum.apex;
    bool included = true;
    // 4 face normals from the frustum.
    for (uint32_t i = 0; i < 5; ++i) {
        Float e = dot(frustum.planes[i].absNormal, halfExtents);
        Float s = dot(frustum.planes[i].normal, centerOffset);
        if (s - e >= frustum.planes[i].dist) {
            return FrustumIsectResult::EDisjoint;
        } else if (s + e > frustum.planes[i].dist) {
            included = false; // definitely not included, but maybe intersected.
        } // else: included for this plane.
    }
    if (included) {
        return FrustumIsectResult::EIncluded;
    }

    return FrustumIsectResult::EIntersect;
}

FrustumIsectResult frustumIntersectExact(const Frustum &frustum, const AABB &aabb)
{
    // 3 face normals from the aabb.
    for (uint32_t i = 0; i < 3; ++i) {
        // frustum corner dir.
        if (frustum.allDirsPositive[i] && frustum.nearCornerBound.min[i] > aabb.max[i]) {
            return FrustumIsectResult::EDisjoint;
        }
        if (frustum.allDirsNegative[i] && frustum.nearCornerBound.max[i] < aabb.min[i]) {
            return FrustumIsectResult::EDisjoint;
        }
    }

    Vector halfExtents = Float(0.5) * aabb.getExtents();
    Vector center = (Vector)aabb.getCenter();
    Vector centerOffset = (Point)center - frustum.apex;
    bool included = true;
    // 4 face normals from the frustum.
    for (uint32_t i = 0; i < 5; ++i) {
        Float e = dot(frustum.planes[i].absNormal, halfExtents);
        Float s = dot(frustum.planes[i].normal, centerOffset);
        if (s - e >= frustum.planes[i].dist) {
            return FrustumIsectResult::EDisjoint;
        } else if (s + e > frustum.planes[i].dist) {
            included = false; // definitely not included, but maybe intersected.
        } // else: included for this plane.
    }
    if (included) {
        return FrustumIsectResult::EIncluded;
    }

    // Edge cross products?...
    for (uint32_t i = 0; i < frustum.crossAxisCount; ++i) {
        Float c = dot(frustum.crossAxis[i], center);
        Float t =
            std::abs(halfExtents.x * frustum.crossAxis[i].x) +
            std::abs(halfExtents.y * frustum.crossAxis[i].y) +
            std::abs(halfExtents.z * frustum.crossAxis[i].z);
        if (c + t <= frustum.crossAxisRange[i]) {
            return FrustumIsectResult::EDisjoint;
        }
    }

    return FrustumIsectResult::EIntersect;
}

// Cone-frustum-AABB exact test, based on http://jcgt.org/published/0009/01/01/
// Somewhat optimized implementation with SSE but doesn't seem to be faster though...
//static FrustumIsectResult frustumIntersectExact(const Frustum &frustum, const AABB &aabb)
//{
//    using math::SSEVector4f;
//    using math::hmax_ps;
//    using math::hmin_ps;
//
//    Vector halfExtents = Float(0.5) * aabb.getExtents();
//    Vector centerOffset = aabb.getCenter() - frustum.apex;
//    bool included = true;
//    // 4 face normals from the frustum.
//    for (uint32_t i = 0; i < 4; ++i) {
//        Float e = dot(frustum.planes[i].absNormal, halfExtents);
//        Float s = dot(frustum.planes[i].normal, centerOffset);
//        if (s - e >= 0.0) {
//            return FrustumIsectResult::EDisjoint;
//        } else if (s + e > 0.0) {
//            included = false; // definitely not included, but maybe intersected.
//        } // else: included for this plane.
//    }
//    if (included) {
//        return FrustumIsectResult::EIncluded;
//    }
//
//    SSEVector4f inf(SSEConstants::p_inf.ps);
//
//    // Duplicate last channel.
//    SSEVector4f aabb_min(aabb.min.x, aabb.min.y, aabb.min.z, aabb.min.z);
//    SSEVector4f aabb_max(aabb.max.x, aabb.max.y, aabb.max.z, aabb.max.z);
//    SSEVector4f apex(frustum.apex.x, frustum.apex.y, frustum.apex.z, frustum.apex.z);
//    SSEVector4f precompute[2];
//    precompute[0] = SSEVector4f(frustum.precompute[0].x, frustum.precompute[0].y, frustum.precompute[0].z, frustum.precompute[0].z);
//    precompute[1] = SSEVector4f(frustum.precompute[1].x, frustum.precompute[1].y, frustum.precompute[1].z, frustum.precompute[1].z);
//
//    SSEVector4f minOffset = aabb_min - apex;
//    SSEVector4f maxOffset = aabb_max - apex;
//
//    SSEVector4f t0s = minOffset * precompute[0];
//    SSEVector4f t1s = minOffset * precompute[1];
//    SSEVector4f t2s = maxOffset * precompute[0];
//    SSEVector4f t3s = maxOffset * precompute[1];
//
//    SSEVector4f t0sNeg = t0s < SSEVector4f::zero();
//    SSEVector4f t1sNeg = t1s < SSEVector4f::zero();
//    SSEVector4f t2sNeg = t2s < SSEVector4f::zero();
//    SSEVector4f t3sNeg = t3s < SSEVector4f::zero();
//
//    SSEVector4f tMinAxisIfNotNeg =
//        min(select(t0sNeg, inf, t0s),
//            min(select(t1sNeg, inf, t1s),
//                min(select(t2sNeg, inf, t2s),
//                    select(t3sNeg, inf, t3s))));
//
//    SSEVector4f between = (aabb_min < apex) & (apex < aabb_max);
//    SSEVector4f tMinAxis = select(between, SSEVector4f::zero(), tMinAxisIfNotNeg);
//
//    SSEVector4f tMaxAxisIfNotNeg =
//        max(select(t0sNeg, SSEVector4f::zero(), t0s),
//            max(select(t1sNeg, SSEVector4f::zero(), t1s),
//                max(select(t2sNeg, SSEVector4f::zero(), t2s),
//                    select(t3sNeg, SSEVector4f::zero(), t3s))));
//
//    // 	bool3 betweenCond = (!t0sNeg && !t1sNeg) || (!t2sNeg && !t3sNeg);
//    SSEVector4f betweenCond = (t0sNeg | t1sNeg) & (t2sNeg | t3sNeg) ^ SSEVector4f(SSEConstants::ffffffff.ps);
//    SSEVector4f tMaxAxis = select(betweenCond, tMaxAxisIfNotNeg, inf);
//
//    float tMin = std::max(0.0f, math::hmax_ps(tMinAxis));
//    float tMax = std::min(std::numeric_limits<float>::infinity(), math::hmin_ps(tMaxAxis));
//
//    return tMin <= tMax ? FrustumIsectResult::EIntersect : FrustumIsectResult::EDisjoint;
//}

MTS_NAMESPACE_END
