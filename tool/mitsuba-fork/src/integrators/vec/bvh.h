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

// Vectorizer: Added a stripped-down pbrt-style BVH, but also with modification and potential improvement.
// (TODO: SBVH?)

#pragma once

#include <mitsuba/core/frustum.h>
#include <vectorizer/vectorizer.h>

MTS_NAMESPACE_BEGIN

struct BVHBuildNode;
struct LinearBVHNode;
struct BVHPrimitiveInfo;

struct Primitive
{
    uint32_t meshIndex;
    uint32_t primIndex;
};

struct FrustumBVHIsectEntry
{
    uint32_t primOffset = 0;
    uint32_t primCount = 0;
    bool included = false;
};

struct FrustumBVHIsectResult
{
    std::vector<FrustumBVHIsectEntry> entries;
    uint32_t totalPrimCount = 0;

    void clear() {
        entries.clear();
        totalPrimCount = 0;
    }
};

struct SecondaryPipelineInc;
struct IncrementalContext
{
    Frustum frustum;
    const Scene *scene = nullptr;
    const TriMesh *queryMesh = nullptr;
    SecondaryPipelineInc *pipeline = nullptr;
    vtrz::Vectorizer *vect = nullptr;
};

struct BVH final : public Object
{
public:
    BVH() = default;
    virtual ~BVH();

    BVH(const BVH &other);
    BVH(BVH &&other);
    BVH &operator=(const BVH &other);
    BVH &operator=(BVH &&other);

    void build(const Scene &scene);
    void frustumIntersect(const Frustum &frustum, FrustumBVHIsectResult &result, FrustumAABBOption AABBOption) const;
    void vectorizeOcclusion(IncrementalContext &ctx) const;

    inline AABB getAABB() const;
    inline const std::vector<Primitive> &primitives() const { return m_primitives; }

private:
	uint32_t recursiveBuild(std::vector<BVHBuildNode> &buildNodePool,
		std::vector<BVHPrimitiveInfo> &primitiveInfo,
		uint32_t primStart, uint32_t primEnd, uint32_t &totalNodeCount,
		std::vector<Primitive> &orderedPrims) const;
	uint32_t flatten(uint32_t nodeIndex, const std::vector<BVHBuildNode> &buildNodePool, uint32_t &offset);


	std::vector<const TriMesh *> m_meshes;
    std::vector<Primitive> m_primitives; // This will be ordered after BVH building.
    LinearBVHNode *m_nodes = nullptr;
    uint32_t m_totalNodeCount = 0;
};

MTS_NAMESPACE_END
