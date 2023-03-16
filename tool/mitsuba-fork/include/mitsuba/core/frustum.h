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

#pragma once
#if !defined(__MITSUBA_CORE_FRUSTUM_H_)
#define __MITSUBA_CORE_FRUSTUM_H_

#include <mitsuba/mitsuba.h>
#include <mitsuba/core/aabb.h>

MTS_NAMESPACE_BEGIN

enum class FrustumAABBOption
{
    eCoarse,
    eExact,
};

// More like a pyramid.
struct MTS_EXPORT_CORE Frustum
{
    struct Plane {
        Vector normal;
		Vector absNormal;
        Float dist;
    };

	Plane planes[5];
	Point apex;
    Vector centerDir;
    AABB nearCornerBound;
	bool allDirsPositive[3];
	bool allDirsNegative[3];

    Vector crossAxis[12];
    Float crossAxisRange[12];
    uint32_t crossAxisCount = 0;

    inline Frustum() = default;
    // Assume dirs are ordered CCW.
    Frustum(const Point &apex, const Vector &centerDir, const Vector dirs[4], Float nearClip, FrustumAABBOption aabbOption);
};

enum class FrustumIsectResult : uint8_t
{
	EDisjoint,
	EIntersect,
	EIncluded,
};

MTS_EXPORT_CORE FrustumIsectResult frustumIntersectCoarse(const Frustum &frustum, const AABB &aabb);
MTS_EXPORT_CORE FrustumIsectResult frustumIntersectExact(const Frustum &frustum, const AABB &aabb);

MTS_NAMESPACE_END

#endif /* __MITSUBA_CORE_FRUSTUM_H_ */
