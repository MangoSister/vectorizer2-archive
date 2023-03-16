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

#include "bvh.h"
#include "util.h"
#include "pipeline.h"
#include <mitsuba/render/scene.h>

MTS_NAMESPACE_BEGIN

struct BVHBuildNode
{
    void initLeaf(uint32_t first, uint32_t n, const AABB &b) {
        primOffset = first;
        primCount = n;
        bound = b;
        children[0] = children[1] = 0;
    }
    void initInterior(uint32_t axis, uint32_t first, uint32_t n,
        uint32_t c0, uint32_t c1,
        const std::vector<BVHBuildNode> &buildNodePool) {
        primOffset = first;
        primCount = n;
        children[0] = c0;
        children[1] = c1;
        bound = buildNodePool[c0].bound;
        bound.expandBy(buildNodePool[c1].bound);
        splitAxis = axis;
    }

    bool isLeaf() const {
        return children[0] == 0 && children[1] == 0;
    }

    AABB bound;
    uint32_t children[2];
    uint32_t splitAxis;
    uint32_t primOffset;
    uint32_t primCount;
};

struct BVHPrimitiveInfo
{
    BVHPrimitiveInfo() {}
    BVHPrimitiveInfo(uint32_t primitiveIndex, const AABB &bounds)
        : primitiveIndex(primitiveIndex), bounds(bounds), centroid(bounds.getCenter()) {}

    uint32_t primitiveIndex;
    AABB bounds;
    Point centroid;
};

struct alignas(32) LinearBVHNode
{
    AABB bound;
    uint32_t primOffset = 0;
    uint32_t splitAxis : 2; // Interior only
    uint32_t rightChildOffset : 30; // Interior only

    inline bool isLeaf() const { return rightChildOffset == 0; }
};
static_assert(sizeof(LinearBVHNode) == 32, "Unexpected LinearBVHNode size.");

struct TraversalStackEntry
{
    uint32_t nodeIndex = 0;
    uint32_t primCount = 0;
};

struct TraversalStack
{
    static constexpr uint8_t maxStackDepth = 64;
    TraversalStackEntry entries[maxStackDepth];
    uint8_t stackSize = 0;

    void push(uint32_t nodeIndex, uint32_t primCount) {
        SAssert(stackSize < maxStackDepth);
        uint32_t index = stackSize++;
        entries[index].nodeIndex = nodeIndex;
        entries[index].primCount = primCount;
    }

    TraversalStackEntry pop() {
        SAssert(stackSize > 0);
        uint32_t index = --stackSize;
        return entries[index];
    }

    bool empty() const { return stackSize == 0; }
};

BVH::~BVH()
{
	if (m_nodes) {
		freeAligned(m_nodes);
		m_nodes = nullptr;
	}
}

BVH::BVH(const BVH &other) :
    m_meshes(other.m_meshes),
    m_primitives(other.m_primitives),
    m_totalNodeCount(other.m_totalNodeCount)
{
    m_nodes = (LinearBVHNode *)allocAligned(sizeof(LinearBVHNode) * m_totalNodeCount);
    memcpy(m_nodes, other.m_nodes, sizeof(LinearBVHNode) * m_totalNodeCount);
}

BVH::BVH(BVH &&other)
{
    std::swap(m_meshes, other.m_meshes);
    std::swap(m_primitives, other.m_primitives);
    std::swap(m_totalNodeCount, other.m_totalNodeCount);
    other.m_totalNodeCount = 0;
    std::swap(m_nodes, other.m_nodes);
    other.m_totalNodeCount = 0;
}

BVH &BVH::operator=(const BVH &other)
{
    if (m_nodes) {
        freeAligned(m_nodes);
        m_nodes = nullptr;
    }

    m_meshes = other.m_meshes;
    m_primitives = other.m_primitives;
    m_totalNodeCount = other.m_totalNodeCount;
    m_nodes = (LinearBVHNode *)allocAligned(sizeof(LinearBVHNode) * m_totalNodeCount);
    memcpy(m_nodes, other.m_nodes, sizeof(LinearBVHNode) * m_totalNodeCount);
    return *this;
}

BVH &BVH::operator=(BVH &&other)
{
    std::swap(m_meshes, other.m_meshes);
    std::swap(m_primitives, other.m_primitives);
    std::swap(m_totalNodeCount, other.m_totalNodeCount);
    other.m_totalNodeCount = 0;
    std::swap(m_nodes, other.m_nodes);
    other.m_totalNodeCount = 0;
    return *this;
}

void BVH::build(const Scene &scene)
{
    const auto &meshes = scene.getMeshes();
    for (const TriMesh *mesh : meshes) {
        m_meshes.push_back(mesh);
    }

	uint32_t totalPrimCount = 0;
	for (const TriMesh *mesh : m_meshes) {
		totalPrimCount += (uint32_t)mesh->getTriangleCount();
	}
	m_primitives.reserve(totalPrimCount);
	for (uint32_t meshIndex = 0; meshIndex < (uint32_t)m_meshes.size(); ++meshIndex) {
		const TriMesh *mesh = m_meshes[meshIndex];
		for (uint32_t triIndex = 0; triIndex < (uint32_t)mesh->getTriangleCount(); ++triIndex) {
			m_primitives.push_back({ meshIndex, triIndex });
		}
	}

	std::vector<BVHPrimitiveInfo> primitiveInfo(m_primitives.size());
	for (uint32_t i = 0; i < (uint32_t)m_primitives.size(); ++i) {
		const auto &prim = m_primitives[i];
		const TriMesh &mesh = *m_meshes[prim.meshIndex];
		primitiveInfo[i] = { i, mesh.getTriangles()[prim.primIndex].getAABB(mesh.getVertexPositions()) };
	}

	std::vector<Primitive> orderedPrims;
	orderedPrims.reserve(m_primitives.size());
    m_totalNodeCount = 0;

	std::vector<BVHBuildNode> buildNodePool;
    buildNodePool.reserve(std::min((uint32_t)1024, 2 * totalPrimCount - 1));

	recursiveBuild(buildNodePool, primitiveInfo, 0, (uint32_t)m_primitives.size(), m_totalNodeCount, orderedPrims);
	std::swap(m_primitives, orderedPrims);

	m_nodes = (LinearBVHNode *)allocAligned(sizeof(LinearBVHNode) * m_totalNodeCount);
	uint32_t offset = 0;
    flatten(0, buildNodePool, offset);
	Assert(m_totalNodeCount == offset);
}

struct SAHBucketInfo
{
	uint32_t count = 0;
	AABB bounds;
};

inline Float offset(const Point &point, const AABB &bounds, uint32_t dim)
{
	Float ext = bounds.getExtents()[dim];
	if (ext == 0.0) {
		return 0.0;
	}
	return (point[dim] - bounds.min[dim]) / ext;
}


uint32_t BVH::recursiveBuild(
	std::vector<BVHBuildNode> &buildNodePool,
	std::vector<BVHPrimitiveInfo> & primitiveInfo,
	uint32_t primStart, uint32_t primEnd, uint32_t &totalNodeCount,
	std::vector<Primitive> &orderedPrims) const
{
	Assert(primStart < primEnd);

    uint32_t nodeIndex = totalNodeCount++;
	buildNodePool.resize(totalNodeCount);

	// Compute bounds of all primitives in BVH node.
	AABB bounds;
	for (uint32_t i = primStart; i < primEnd; ++i) {
		bounds.expandBy(primitiveInfo[i].bounds);
	}
	uint32_t primCount = primEnd - primStart;

	auto asLeaf = [&]() {
		uint32_t firstPrim = (uint32_t)orderedPrims.size();
		for (uint32_t i = primStart; i < primEnd; ++i) {
			uint32_t index = primitiveInfo[i].primitiveIndex;
			orderedPrims.push_back(m_primitives[index]);
		}
        buildNodePool[nodeIndex].initLeaf(firstPrim, primCount, bounds);
		return nodeIndex;
	};

	if (primCount == 1) {
		return asLeaf();
	} else {
		// Compute bound of primitive centroids, choose split dimension.
		AABB centroidBounds;
		for (uint32_t i = primStart; i < primEnd; ++i)
			centroidBounds.expandBy(primitiveInfo[i].centroid);
		int dim = centroidBounds.getLargestAxis();
		// Partition primitives into two sets and build children.
		uint32_t primMid = (primStart + primEnd) / 2;
		// Super degeneracy...no bound at all.
		if (centroidBounds.max[dim] == centroidBounds.min[dim]) {
			return asLeaf();
		} else {
			// SAH.
			if (primCount <= 2) {
				std::nth_element(&primitiveInfo[primStart], &primitiveInfo[primMid], &primitiveInfo[primEnd - 1] + 1,
					[dim](const BVHPrimitiveInfo &a, const BVHPrimitiveInfo &b) {
					return a.centroid[dim] < b.centroid[dim];
				});
			} else {
				constexpr uint32_t bucketCount = 12;
				SAHBucketInfo buckets[bucketCount];
				for (uint32_t i = primStart; i < primEnd; ++i) {
					uint32_t b = (uint32_t)std::floor(bucketCount * offset(primitiveInfo[i].centroid, centroidBounds, dim));
					b = std::min(b, bucketCount - 1);
					++buckets[b].count;
					buckets[b].bounds.expandBy(primitiveInfo[i].bounds);
				}
				// Compute costs for splitting after each bucket.
				// Can optimize the double loop.
				Float cost[bucketCount - 1];
				for (uint32_t i = 0; i < bucketCount - 1; ++i) {
					AABB b0, b1;
					uint32_t count0 = 0, count1 = 0;
					for (uint32_t j = 0; j <= i; ++j) {
						b0.expandBy(buckets[j].bounds);
						count0 += buckets[j].count;
					}
					for (uint32_t j = i + 1; j < bucketCount; ++j) {
						b1.expandBy(buckets[j].bounds);
						count1 += buckets[j].count;
					}
					cost[i] = 1 + (count0 * b0.getSurfaceArea() + count1 * b1.getSurfaceArea()) / bounds.getSurfaceArea();
				}
				// Find bucket to split at that minimizes SAH metric.
				Float minCost = cost[0];
				uint32_t minCostSplitBucket = 0;
				for (uint32_t i = 1; i < bucketCount - 1; ++i) {
					if (cost[i] < minCost) {
						minCost = cost[i];
						minCostSplitBucket = i;
					}
				}
				// Either create leaf or split primitives at selected SAH bucket.
				Float leafCost = (Float)primCount;
				constexpr uint32_t maxPrimsInNode = 4;
				if (primCount <= maxPrimsInNode && minCost >= leafCost) {
					return asLeaf();
				} else {
					BVHPrimitiveInfo *pmid = std::partition(&primitiveInfo[primStart], &primitiveInfo[primEnd - 1] + 1,
						[=](const BVHPrimitiveInfo &pi) {
						uint32_t b = (uint32_t)std::floor(bucketCount * offset(pi.centroid, centroidBounds, dim));
						b = std::min(b, bucketCount - 1);
						Assert(b >= 0 && b < bucketCount);
						return b <= minCostSplitBucket;
					});
					primMid = (uint32_t)(pmid - &primitiveInfo[0]);
				}
			}
            uint32_t orderedPrimStart = (uint32_t)orderedPrims.size();
            uint32_t child0Index = recursiveBuild(buildNodePool, primitiveInfo, primStart, primMid, totalNodeCount, orderedPrims);
            uint32_t child1Index = recursiveBuild(buildNodePool, primitiveInfo, primMid, primEnd, totalNodeCount, orderedPrims);
            uint32_t orderedPrimEnd = (uint32_t)orderedPrims.size();
            buildNodePool[nodeIndex].initInterior(dim, orderedPrimStart, orderedPrimEnd - orderedPrimStart,
                child0Index, child1Index, buildNodePool);
            Assert(buildNodePool[nodeIndex].primCount ==
                buildNodePool[buildNodePool[nodeIndex].children[0]].primCount +
                buildNodePool[buildNodePool[nodeIndex].children[1]].primCount);
            Assert(buildNodePool[buildNodePool[nodeIndex].children[0]].primOffset == orderedPrimStart &&
                buildNodePool[buildNodePool[nodeIndex].children[0]].primOffset < orderedPrimEnd);
            Assert(buildNodePool[buildNodePool[nodeIndex].children[1]].primOffset > orderedPrimStart &&
                buildNodePool[buildNodePool[nodeIndex].children[1]].primOffset < orderedPrimEnd);
			return nodeIndex;
		}
	}
}

// TODO: Can I avoid this pass?
uint32_t BVH::flatten(uint32_t nodeIndex, const std::vector<BVHBuildNode> &buildNodePool, uint32_t &offset)
{
	LinearBVHNode &linearNode = m_nodes[offset];
    const BVHBuildNode &buildNode = buildNodePool[nodeIndex];
	linearNode.bound = buildNode.bound;
    linearNode.primOffset = buildNode.primOffset;
	uint32_t oldOffset = offset++;
    if (!buildNode.isLeaf()) { // If interior node.
        // Create interior flattened BVH node
        linearNode.splitAxis = buildNode.splitAxis;
        // linearNode->nPrimitives = 0;
        flatten(buildNode.children[0], buildNodePool, offset);
        linearNode.rightChildOffset = flatten(buildNode.children[1], buildNodePool, offset);
        Assert(linearNode.rightChildOffset < (uint32_t)buildNodePool.size());
    } else {
        linearNode.rightChildOffset = 0;
    }
	return oldOffset;
}

AABB BVH::getAABB() const
{
    return m_nodes[0].bound;
}

void BVH::frustumIntersect(const Frustum &frustum, FrustumBVHIsectResult &result, FrustumAABBOption AABBOption) const
{
    TraversalStack stack;
    TraversalStackEntry curr = { 0, (uint32_t)m_primitives.size() };
    while (true) {
        const LinearBVHNode *node = &m_nodes[curr.nodeIndex];
        FrustumIsectResult res;
        if (AABBOption == FrustumAABBOption::eCoarse) {
            res = mitsuba::frustumIntersectCoarse(frustum, node->bound);
        } else {
            res = mitsuba::frustumIntersectExact(frustum, node->bound);
        }
        if (res == FrustumIsectResult::EDisjoint) {
            if (stack.empty()) break;
            curr = stack.pop();
        } else if (res == FrustumIsectResult::EIncluded || node->isLeaf()) {
            result.entries.push_back({ node->primOffset, curr.primCount, res == FrustumIsectResult::EIncluded });
            result.totalPrimCount += curr.primCount;
            if (stack.empty()) break;
            curr = stack.pop();
        } else {
            Assert(res == FrustumIsectResult::EIntersect && !node->isLeaf());
            // const LinearBVHNode *left = &m_nodes[curr.nodeIndex + 1];
            const LinearBVHNode *right = &m_nodes[node->rightChildOffset];
            Assert(right->primOffset > node->primOffset);
            uint32_t leftPrimCount = right->primOffset - node->primOffset;
            Assert(leftPrimCount < curr.primCount);
            uint32_t rightPrimCount = curr.primCount - leftPrimCount;
            stack.push(node->rightChildOffset, rightPrimCount);
            curr = { curr.nodeIndex + 1, leftPrimCount };
        }
    }
}

void BVH::vectorizeOcclusion(IncrementalContext &ctx) const
{
    TraversalStack stack;
    TraversalStackEntry curr = { 0, (uint32_t)m_primitives.size() };
    while (true) {
        const LinearBVHNode *node = &m_nodes[curr.nodeIndex];
        FrustumIsectResult res = mitsuba::frustumIntersectCoarse(ctx.frustum, node->bound);
        if (res == FrustumIsectResult::EDisjoint) {
            if (stack.empty()) break;
            curr = stack.pop();
            continue;
        }

        constexpr uint32_t kOcclusionQueryThreshold = 32;
        if (curr.primCount >= kOcclusionQueryThreshold) {
            BoxOccluder box = makeBoxOccluder(node->bound, ctx.frustum.apex, ctx.pipeline->vpTrans);
            if (box.valid && ctx.vect->queryOccluder(box.occluder, box.depth)) {
                if (stack.empty()) break;
                curr = stack.pop();
                continue;
            }
            // transform and clip
            //AABBClipper aabbClipper;
            //aabbClipper.run(ctx.frustum.apex, node->bound, ctx.pipeline->vpTrans, res == FrustumIsectResult::EIncluded);
            //if (aabbClipper.vertCount && ctx.vect->queryOccluders(
            //    reinterpret_cast<vtrz::Vector3 *>(aabbClipper.positions.data()),
            //    aabbClipper.vertCount)) {
            //    if (stack.empty()) break;
            //    curr = stack.pop();
            //    continue;
            //}
        }

        if (res == FrustumIsectResult::EIncluded || node->isLeaf()) {
            FrustumBVHIsectEntry entry = { node->primOffset, curr.primCount, res == FrustumIsectResult::EIncluded };
            ctx.pipeline->clear();
            ctx.pipeline->run(*ctx.scene, *this, entry, ctx.queryMesh);
            if (!ctx.pipeline->positions.empty()) {
                ctx.vect->addOccluders(
                    reinterpret_cast<vtrz::Vector3 *>(ctx.pipeline->positions.data()),
                    (uint32_t)ctx.pipeline->positions.size());
            }

            if (stack.empty()) break;
            curr = stack.pop();
        } else {
            Assert(res == FrustumIsectResult::EIntersect && !node->isLeaf());
            const LinearBVHNode *left = &m_nodes[curr.nodeIndex + 1];
            const LinearBVHNode *right = &m_nodes[node->rightChildOffset];
            Assert(right->primOffset > node->primOffset);
            uint32_t leftPrimCount = right->primOffset - node->primOffset;
            Assert(leftPrimCount < curr.primCount);
            uint32_t rightPrimCount = curr.primCount - leftPrimCount;

            if (ctx.frustum.centerDir[node->splitAxis] < 0.0f) {
                stack.push(curr.nodeIndex + 1, leftPrimCount);
                curr = { node->rightChildOffset, rightPrimCount };
            } else {
                stack.push(node->rightChildOffset, rightPrimCount);
                curr = { curr.nodeIndex + 1, leftPrimCount };
            }
        }
    }
}

MTS_NAMESPACE_END
