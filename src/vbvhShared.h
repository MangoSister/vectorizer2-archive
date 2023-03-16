#pragma once

#include "util.h"
#include "maths.h"
#include "diff.h"
#include "poolArray.h"
#include "polygonShared.h"
#include "vectorizer.h"
#include <vector>
#include <fstream>

namespace vtrz
{

using NodeIndex = uint32_t;
constexpr NodeIndex kEmptyParentNodeIndex = (NodeIndex)~0;
constexpr uint32_t kBackgroundTriIndex = (uint32_t)~0;

template <typename T>
struct Node
{
    Node() = default;
    Node(const Node &other) {
        bound = other.bound;
        if (other.isLeaf()) {
            leaf.geom = other.leaf.geom;
            leaf.vertCount = other.leaf.vertCount;
            leaf.parent = other.leaf.parent;
        } else {
            inner.children[0] = other.inner.children[0];
            inner.children[1] = other.inner.children[1];
            inner.flag = other.inner.flag;
            inner.parent = other.inner.parent;
        }
    }

    void initInner(const AABB2 &b, NodeIndex parent) {
        bound = b;
        inner.parent = parent;
        inner.flag = 0;
    }

    void initLeaf(const AABB2 &b, NodeIndex parent, const T *geom, uint32_t vertCount, uint32_t triIndex) {
        ASSERT(vertCount >= 3);
        bound = b;
        leaf.parent = parent;
        leaf.vertCount = vertCount;
        leaf.geom = (T *)allocAligned(sizeof(T) * vertCount + sizeof(uint32_t), 16);
        memcpy(leaf.geom, geom, sizeof(T) * vertCount);
        *reinterpret_cast<uint32_t *>(&leaf.geom[vertCount]) = triIndex;
    }

    void free() {
        if (isLeaf()) {
            freeAligned(leaf.geom);
            leaf.geom = nullptr;
        }
    }

    bool isLeaf() const { return inner.flag != 0; }

    uint32_t triIndex() const {
        ASSERT(isLeaf());
        return *reinterpret_cast<uint32_t *>(&leaf.geom[leaf.vertCount]);
    }

    uint32_t &triIndex() {
        ASSERT(isLeaf());
        return *reinterpret_cast<uint32_t *>(&leaf.geom[leaf.vertCount]);
    }

    Vector2I64 vertPos(uint32_t index) const {
        ASSERT(isLeaf());
        ASSERT(index < leaf.vertCount);
        return getVal(leaf.geom[index]);
    }

    AABB2 bound;
    union {
        struct {
            NodeIndex children[2];
            uint32_t flag;
            NodeIndex parent;
        } inner;
        struct {
            T *geom;
            uint32_t vertCount;
            NodeIndex parent;
        } leaf;
    };
};
static_assert(sizeof(Node<Vector2I64>) == 32, "Unexpected Node size.");

struct RangeQuery
{
    const uint32_t *triIndices = nullptr;
    uint32_t triCount = 0;
    std::vector<std::pair<uint32_t, uint32_t>> leafRefRanges;

    void reset(const uint32_t *newTriIndices, uint32_t newTriCount) {
        triIndices = newTriIndices;
        triCount = newTriCount;
        leafRefRanges.clear();
    }
};

struct TraverseStack
{
    static constexpr uint8_t maxStackDepth = 64;
    uint32_t nodes[maxStackDepth];
    uint8_t heights[maxStackDepth];
    uint8_t size = 0;

    void reset() { size = 0; }

    void push(uint32_t nodeIndex, const uint8_t *height = nullptr) {
        ASSERT_FATAL(size < maxStackDepth);
        uint32_t index = size++;
        nodes[index] = nodeIndex;
        if (height) heights[index] = *height;
    }

    bool pop(uint32_t &nodeIndex, uint8_t *height = nullptr) {
        if (size == 0) return false;
        uint32_t index = --size;
        nodeIndex = nodes[index];
        if (height) *height = heights[index];
        return true;
    }

    bool empty() const { return size == 0; }
};

struct TraverseContext
{
    struct MergeNode
    {
        NodeIndex index;
        uint32_t height;
    };
    std::vector<MergeNode> mergeList;
    uint32_t triIndex;
    bool foreground = true;
    TraverseStack stack;

    struct {
        std::vector<Vector2I64> occluderShapes;
        std::vector<uint32_t> vertCounts;
    } hiz;

    void reset(uint32_t tri) {
        mergeList.clear();
        triIndex = tri;
        foreground = true;
        stack.reset();

        hiz.occluderShapes.clear();
        hiz.vertCounts.clear();
    }
};

inline uint32_t permute(uint32_t i, uint32_t len) {
    constexpr uint32_t seed = 2020;
    uint32_t w = len - 1;
    w |= w >> 1;
    w |= w >> 2;
    w |= w >> 4;
    w |= w >> 8;
    w |= w >> 16;
    do {
        i ^= seed;
        i *= 0xe170893d;
        i ^= seed >> 16;
        i ^= (i & w) >> 4;
        i ^= seed >> 8;
        i *= 0x0929eb3f;
        i ^= seed >> 23;
        i ^= (i & w) >> 1;
        i *= 1 | seed >> 27;
        i *= 0x6935fa69;
        i ^= (i & w) >> 11;
        i *= 0x74dcb303;
        i ^= (i & w) >> 2;
        i *= 0x9e501cc3;
        i ^= (i & w) >> 2;
        i *= 0xc860a3df;
        i &= w;
        i ^= i >> 5;
    } while (i >= len);
    return (i + seed) % len;
}

struct TriBoundHelper
{
    explicit TriBoundHelper(const Tri2 &tri) {
        for (uint32_t i = 0; i < 3; ++i) {
            bound.expand(tri[i]);
            dir[i] = tri[(i + 1) % 3] - tri[i];
            axis[i] = Vector2(dir[i].y, -dir[i].x);
            axisNegDir[i][0] = axis[i].x < 0.0f;
            axisNegDir[i][1] = axis[i].y < 0.0f;
            axisPosDir[i][0] = !axisNegDir[i][0];
            axisPosDir[i][1] = !axisNegDir[i][1];
            tmin[i] = Constants::inf;
            tmax[i] = -Constants::inf;
            for (uint32_t j = 0; j < 3; ++j) {
                float p = dot(axis[i], tri[j]);
                tmin[i] = std::min(tmin[i], p);
                tmax[i] = std::max(tmax[i], p);
            }
        }
    }

    AABB2 bound;
    Vector2 dir[3];
    Vector2 axis[3];
    float tmin[3];
    float tmax[3];
    bool axisNegDir[3][2];
    bool axisPosDir[3][2];
};

// '0': disjoint | '1': intersect | 'c': bound is contained in tri.
inline char triBoundIsect(const Tri2 &tri, const TriBoundHelper &help, const AABB2 &bound) {
    if (!intersectBool(help.bound, bound)) {
        return '0';
    }

    for (uint32_t i = 0; i < 3; ++i) {
        float tmin = help.axis[i].x * bound[help.axisNegDir[i][0]].x + help.axis[i].y * bound[help.axisNegDir[i][1]].y;
        float tmax = help.axis[i].x * bound[help.axisPosDir[i][0]].x + help.axis[i].y * bound[help.axisPosDir[i][1]].y;
        if (help.tmin[i] >= tmax || help.tmax[i] <= tmin) {
            return '0';
        }
    }

    for (uint32_t i = 0; i < 4; ++i) {
        Vector2 corner(bound[i >> 1].x, bound[i & 1].y);
        for (uint32_t j = 0; j < 3; ++j) {
            if (cross(corner - tri[j], help.dir[j]) < 0.0f) {
                return '1';
            }
        }
    }
    return 'c';
}

inline DVector3 barycentricCoord(const Vector2 &point, const Tri2Grad &tri) {
    DVector3 coord;

    // {0, 1, 2} -> {a, b, c}
    DVector2 eAb = tri[1] - tri[0];
    DVector2 eAc = tri[2] - tri[0];
    dfloat invAreaAbc = 1.0f / abs(cross(eAb, eAc));

    DVector2 d = DVector2(point) - tri[0];
    dfloat areaAbp = abs(cross(d, eAb));
    coord.z = areaAbp * invAreaAbc;

    dfloat areaApc = abs(cross(d, eAc));
    coord.y = areaApc * invAreaAbc;

    coord.x = 1.0f - coord.y - coord.z;
    coord.x.value = saturate(coord[0].value);

    return coord;
}

// https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_standard_multisample_quality_levels
constexpr Vector2 msaaPattern1[1]{
    Vector2(0.0f, 0.0f),
};

constexpr Vector2 msaaPattern2[2]{
    Vector2(4.0f / 8.0f,    4.0f / 8.0f),
    Vector2(-4.0f / 8.0f,   -4.0f / 8.0f),
};

constexpr Vector2 msaaPattern4[4]{
    Vector2(-2.0f / 8.0f,   -6.0f / 8.0f),
    Vector2(6.0f / 8.0f,    -2.0f / 8.0f),
    Vector2(-6.0f / 8.0f,   2.0f / 8.0f),
    Vector2(2.0f / 8.0f,    6.0f / 8.0f),
};

constexpr Vector2 msaaPattern8[8]{
    Vector2(1.0f / 8.0f,    -3.0f / 8.0f),
    Vector2(-1.0f / 8.0f,   3.0f / 8.0f),
    Vector2(5.0f / 8.0f,    1.0f / 8.0f),
    Vector2(-3.0f / 8.0f,   -5.0f / 8.0f),

    Vector2(-5.0f / 8.0f,   5.0f / 8.0f),
    Vector2(-7.0f / 8.0f,   -1.0f / 8.0f),
    Vector2(3.0f / 8.0f,    7.0f / 8.0f),
    Vector2(7.0f / 8.0f,    -7.0f / 8.0f),
};

constexpr Vector2 msaaPattern16[16]{
    Vector2(1.0f / 8.0f,    1.0f / 8.0f),
    Vector2(-1.0f / 8.0f,   -3.0f / 8.0f),
    Vector2(-3.0f / 8.0f,   2.0f / 8.0f),
    Vector2(4.0f / 8.0f,    -1.0f / 8.0f),
    Vector2(-5.0f / 8.0f,   -2.0f / 8.0f),
    Vector2(2.0f / 8.0f,    5.0f / 8.0f),
    Vector2(5.0f / 8.0f,    3.0f / 8.0f),
    Vector2(3.0f / 8.0f,    -5.0f / 8.0f),

    Vector2(-2.0f / 8.0f,   6.0f / 8.0f),
    Vector2(0.0f / 8.0f,    -7.0f / 8.0f),
    Vector2(-4.0f / 8.0f,   -6.0f / 8.0f),
    Vector2(-6.0f / 8.0f,   4.0f / 8.0f),
    Vector2(-7.9f / 8.0f,   0.0f / 8.0f), // 8.0 -> 7.9: avoid boundary
    Vector2(7.0f / 8.0f,    -4.0f / 8.0f),
    Vector2(6.0f / 8.0f,    7.0f / 8.0f),
    Vector2(-7.0f / 8.0f,   -7.9f / 8.0f), // 8.0 -> 7.9: avoid boundary
};

template <typename TPoint2I>
inline AABB2 computeNewBound(const TPoint2I *poly, uint32_t count)
{
    int64_t xmin = (int64_t)(kScaleFactor);
    int64_t ymin = (int64_t)(kScaleFactor);
    int64_t xmax = -(int64_t)(kScaleFactor);
    int64_t ymax = -(int64_t)(kScaleFactor);
    for (uint32_t i = 0; i < count; ++i) {
        Vector2I64 pos = getVal(poly[i]);
        xmin = std::min(xmin, pos.x);
        ymin = std::min(ymin, pos.y);
        xmax = std::max(xmax, pos.x);
        ymax = std::max(ymax, pos.y);
    }
    AABB2 ret;
    ret.min.x = (float)((double)std::max(xmin - 1, -(int64_t)kScaleFactor) * kInvScaleFactor);
    ret.min.y = (float)((double)std::max(ymin - 1, -(int64_t)kScaleFactor) * kInvScaleFactor);
    ret.max.x = (float)((double)std::min(xmax + 1, (int64_t)kScaleFactor) * kInvScaleFactor);
    ret.max.y = (float)((double)std::min(ymax + 1, (int64_t)kScaleFactor) * kInvScaleFactor);

    return ret;
}

enum class TraversalType : uint8_t
{
    eGeneral,
    eOcclusionOnly,
    eOcclusionOnlyHiZ,
};

template <typename Types>
struct VBVHShared
{
    using TFloat = typename Types::TFloat;
    using TPoint2I = typename Types::TPoint2I;
    using TPoint2F = typename Types::TPoint2F;
    using TPoint3F = typename Types::TPoint3F;
    using TTri2 = typename Types::TTri2;
    using TTri3 = typename Types::TTri3;
    using TLeafRegion = typename Types::TLeafRegion;
    using TPointSample = typename Types::TPointSample;
    using TPointSampleArray = typename Types::TPointSampleArray;

    virtual ~VBVHShared();

    void grow(NodeIndex parent, NodeIndex *leafs, uint32_t leafCount,
        uint32_t height, TraverseContext &ctx);
    void traverseNoTest(NodeIndex subtree, uint8_t height, uint32_t triIndex, TraverseContext &ctx);
    bool depthOrder(uint32_t triIndex1, uint32_t triIndex2, const Vector2 &ref) const;
    bool depthOrder(const Tri3 &tri1, uint32_t triIndex2, const Vector2 &ref) const;
    void removeLeaf(NodeIndex stale);
    void buildLeafRefs();

    void removeBackground();
    NodeIndex initOccludeeBVH(uint32_t refStart, uint32_t refEnd);
    void deleteOccluded(TraverseContext &ctx);

    uint32_t leafCount() const;
    TLeafRegion getLeaf(uint32_t leafRefIndex) const;
    TFloat coverageQuery() const;
    void rangeQuery(RangeQuery &query) const;
    bool pointQuery(const Vector2 &point, NodeIndex &leaf) const;

    TPointSample pointSample() const;
    TPointSampleArray pointSampleMulti(const uint16_t *matIdBuffer, const Vector3 *vertexNormalBuffer) const;

    uint32_t getTriCount() const;
    TTri2 getTri2d(uint32_t triId) const;
    TTri3 getTri3d(uint32_t triId) const;

    // Debug/Visualization
    void exportTo(const char *outputPath) const;

    using NodeType = Node<TPoint2I>;
    PoolArray<NodeType> nodePool;
    NodeIndex rootIndex;
    std::vector<NodeIndex> leafRefs;
    std::vector<TPoint3F> vertexBuffer;
    std::vector<uint32_t> indexBuffer;
    DepthFunc depthFunc;

    static constexpr int64_t kSnapRadius = 10;
    static constexpr int64_t kPointQueryRadius = 1024;

    static constexpr double kOcclusionQueryThreshold = 0.999;
};

template <typename Types>
VBVHShared<Types>::~VBVHShared()
{
    // Can I avoid this...
    std::vector<uint32_t> allocated = nodePool.allocatedList();
    for (uint32_t i = 0; i < (uint32_t)allocated.size(); ++i) {
        nodePool[allocated[i]].free();
    }
}

template <typename Types>
uint32_t VBVHShared<Types>::getTriCount() const
{
    return indexBuffer.size() ? (uint32_t)indexBuffer.size() / 3 : (uint32_t)vertexBuffer.size() / 3;
}

template <typename Types>
typename VBVHShared<Types>::TTri3 VBVHShared<Types>::getTri3d(uint32_t triId) const
{
    uint32_t offset = 3 * triId;
    if (indexBuffer.size()) {
        return {
            vertexBuffer[indexBuffer[offset + 0]],
            vertexBuffer[indexBuffer[offset + 1]],
            vertexBuffer[indexBuffer[offset + 2]],
        };
    } else {
        return {
            vertexBuffer[offset + 0],
            vertexBuffer[offset + 1],
            vertexBuffer[offset + 2],
        };
    }
}

template <typename Types>
typename VBVHShared<Types>::TTri2 VBVHShared<Types>::getTri2d(uint32_t triId) const
{
    auto tri3d = getTri3d(triId);
    return { (TPoint2F)tri3d[0], (TPoint2F)tri3d[1], (TPoint2F)tri3d[2] };
}

template <typename Types>
void VBVHShared<Types>::grow(NodeIndex parent, NodeIndex *leafs, uint32_t leafCount,
    uint32_t height, TraverseContext &ctx)
{
    ASSERT(leafCount >= 2 && leafCount <= 4);
    if (leafCount == 2) {
        for (uint32_t i = 0; i < 2; ++i) {
            nodePool[parent].inner.children[i] = leafs[i];
            nodePool[leafs[i]].leaf.parent = parent;
            if (nodePool[leafs[i]].triIndex() == ctx.triIndex) {
                ctx.mergeList.push_back({ leafs[i], height + 1 });
            }
        }
        return;
    }
    float invParentArea = 1.0f / nodePool[parent].bound.area();
    AABB2 centerBound;
    for (uint32_t i = 0; i < leafCount; ++i) {
        centerBound.expand(nodePool[leafs[i]].bound.center());
    }
    uint32_t dim = centerBound.largestAxis();
    struct Strategy
    {
        AABB2 leftBound, rightBound;
        float cost;
    };
    VLA(strats, Strategy, leafCount * 2);
    uint32_t bestStrat = 0;
    for (uint32_t i = 0; i < 2 * leafCount; ++i) {
        float split = nodePool[leafs[i >> 1]].bound[i & 1][dim];
        uint32_t nl = 0, nr = 0;
        strats[i].leftBound = AABB2();
        strats[i].rightBound = AABB2();
        for (uint32_t j = 0; j < leafCount; ++j) {
            if (nodePool[leafs[j]].bound.center()[dim] <= split) {
                strats[i].leftBound.expand(nodePool[leafs[j]].bound);
                ++nl;
            } else {
                strats[i].rightBound.expand(nodePool[leafs[j]].bound);
                ++nr;
            }
        }
        constexpr float travWeight = 1.0f;
        strats[i].cost = travWeight +
            nl * strats[i].leftBound.area() * invParentArea +
            nr * strats[i].rightBound.area() * invParentArea;
        bestStrat = i == 0 ? i : (strats[i].cost < strats[bestStrat].cost ? i : bestStrat);
    }
    float split = nodePool[leafs[bestStrat >> 1]].bound[bestStrat & 1][dim];
    NodeIndex *mid = std::partition(leafs, leafs + leafCount,
        [&](NodeIndex i) { return nodePool[i].bound.center()[dim] <= split; });
    uint32_t nl = (uint32_t)(mid - leafs);
    uint32_t nr = leafCount - nl;

    AABB2 leftBound = strats[bestStrat].leftBound;
    AABB2 rightBound = strats[bestStrat].rightBound;

    if (nl == 0 || nr == 0) {
        // Fall back to middle split.
        float midSplit = centerBound.center()[dim];
        mid = std::partition(leafs, leafs + leafCount,
            [&](NodeIndex i) { return nodePool[i].bound.center()[dim] <= midSplit; });
        nl = (uint32_t)(mid - leafs);
        nr = leafCount - nl;
        if (nl == 0 || nr == 0) {
            // Fall back to equal count.
            std::nth_element(leafs, leafs + (leafCount / 2), leafs + leafCount,
                [&](NodeIndex a, NodeIndex b) {
                return nodePool[a].bound.center()[dim] < nodePool[b].bound.center()[dim];
            });
            nl = leafCount / 2;
            nr = leafCount - nl;
        }
        leftBound = AABB2();
        for (uint32_t i = 0; i < nl; ++i) {
            leftBound.expand(nodePool[leafs[i]].bound);
        }
        rightBound = AABB2();
        for (uint32_t i = nl; i < leafCount; ++i) {
            rightBound.expand(nodePool[leafs[i]].bound);
        }
    }
    ASSERT_FATAL(nl > 0 && nr > 0);

    if (nl == 1) {
        nodePool[parent].inner.children[0] = leafs[0];
        nodePool[leafs[0]].leaf.parent = parent;
        if (nodePool[leafs[0]].triIndex() == ctx.triIndex) {
            ctx.mergeList.push_back({ leafs[0], height + 1 });
        }
    } else {
        NodeIndex left = nodePool.append();
        nodePool[parent].inner.children[0] = left;
        nodePool[left].initInner(leftBound, parent);
        grow(left, leafs, nl, height + 1, ctx);
    }
    if (nr == 1) {
        nodePool[parent].inner.children[1] = leafs[nl];
        nodePool[leafs[nl]].leaf.parent = parent;
        if (nodePool[leafs[nl]].triIndex() == ctx.triIndex) {
            ctx.mergeList.push_back({ leafs[nl], height + 1 });
        }
    } else {
        NodeIndex right = nodePool.append();
        nodePool[parent].inner.children[1] = right;
        nodePool[right].initInner(rightBound, parent);
        grow(right, leafs + nl, nr, height + 1, ctx);
    }
}

template <typename Types>
void VBVHShared<Types>::traverseNoTest(NodeIndex subtree, uint8_t height, uint32_t triIndex, TraverseContext &ctx)
{
    NodeIndex curr = subtree;
    while (true) {
        NodeType &node = nodePool[curr];
        if (node.isLeaf()) {
            Vector2 ref;
            {
                Vector2I64 center(0);
                for (uint32_t i = 0; i < node.leaf.vertCount; ++i) {
                    center += node.vertPos(i);
                }
                ref = cast(center) / (float)node.leaf.vertCount;
            }
            if (depthOrder(triIndex, node.triIndex(), ref)) {
                // Node completely occluded by tri. Simply change "color" and add to merge list.
                node.triIndex() = triIndex;
                ctx.mergeList.push_back({ curr , height });
            } else {
                // Tri is always bigger than node in this case.
                ctx.foreground = false;
            }
            if (!ctx.stack.pop(curr, &height)) break;
        } else {
            ++height;
            ctx.stack.push(node.inner.children[1], &height);
            curr = node.inner.children[0];
        }
    }
}

template <typename Types>
bool VBVHShared<Types>::depthOrder(uint32_t triIndex1, uint32_t triIndex2, const Vector2 &ref) const
{
    ASSERT(!(triIndex1 == kBackgroundTriIndex && triIndex2 == kBackgroundTriIndex));
    if (triIndex1 == kBackgroundTriIndex) return false;
    if (triIndex2 == kBackgroundTriIndex) return true;
    Tri3 tri1 = getVal(getTri3d(triIndex1));
    Tri2 tri12d = { (Vector2)tri1[0], (Vector2)tri1[1], (Vector2)tri1[2] };
    Tri3 tri2 = getVal(getTri3d(triIndex2));
    Tri2 tri22d = { (Vector2)tri2[0], (Vector2)tri2[1], (Vector2)tri2[2] };
    Vector3 coord1 = barycentricCoord(ref, tri12d);
    Vector3 coord2 = barycentricCoord(ref, tri22d);
    float d1 = tri1[0].z * coord1.x + tri1[1].z * coord1.y + tri1[2].z * coord1.z;
    float d2 = tri2[0].z * coord2.x + tri2[1].z * coord2.y + tri2[2].z * coord2.z;

    switch (depthFunc) {
    case DepthFunc::eGreater: return d1 > d2;
    case DepthFunc::eLess:default: return d1 < d2;
    }
}

template <typename Types>
bool VBVHShared<Types>::depthOrder(const Tri3 &tri1, uint32_t triIndex2, const Vector2 &ref) const
{
    if (triIndex2 == kBackgroundTriIndex) return true;
    Tri2 tri12d = { (Vector2)tri1[0], (Vector2)tri1[1], (Vector2)tri1[2] };
    Tri3 tri2 = getVal(getTri3d(triIndex2));
    Tri2 tri22d = { (Vector2)tri2[0], (Vector2)tri2[1], (Vector2)tri2[2] };
    Vector3 coord1 = barycentricCoord(ref, tri12d);
    Vector3 coord2 = barycentricCoord(ref, tri22d);
    float d1 = tri1[0].z * coord1.x + tri1[1].z * coord1.y + tri1[2].z * coord1.z;
    float d2 = tri2[0].z * coord2.x + tri2[1].z * coord2.y + tri2[2].z * coord2.z;

    switch (depthFunc) {
    case DepthFunc::eGreater: return d1 > d2;
    case DepthFunc::eLess:default: return d1 < d2;
    }
}

template <typename Types>
void VBVHShared<Types>::removeLeaf(NodeIndex stale)
{
    NodeType &staleNode = nodePool[stale];
    NodeIndex parent = staleNode.leaf.parent;
    if (parent == kEmptyParentNodeIndex) {
        // This is the root (happens when buildOcclusion()).
        nodePool.remove(stale);
        rootIndex = kEmptyParentNodeIndex;
        return;
    }
    NodeIndex sibling;
    {
        NodeType &parentNode = nodePool[parent];
        sibling = stale == parentNode.inner.children[0] ?
            parentNode.inner.children[1] :
            parentNode.inner.children[0];
        NodeType &siblingNode = nodePool[sibling];
        staleNode.free();
        nodePool.remove(stale);

        siblingNode.inner.parent = parentNode.inner.parent;
        if (parentNode.inner.parent != kEmptyParentNodeIndex) {
            if (nodePool[parentNode.inner.parent].inner.children[0] == parent) {
                nodePool[parentNode.inner.parent].inner.children[0] = sibling;
            } else {
                ASSERT(nodePool[parentNode.inner.parent].inner.children[1] == parent);
                nodePool[parentNode.inner.parent].inner.children[1] = sibling;
            }
        }
        nodePool.remove(parent);
        // Update rootIndex if necessary.
        if (parent == rootIndex) {
            rootIndex = sibling;
        }
    }
    NodeIndex curr = sibling;
    parent = nodePool[curr].inner.parent;
    while (parent != kEmptyParentNodeIndex) {
        NodeType &parentNode = nodePool[parent];
        ASSERT(!parentNode.isLeaf());
        sibling = curr == parentNode.inner.children[0] ?
            parentNode.inner.children[1] :
            parentNode.inner.children[0];
        parentNode.bound = join(nodePool[curr].bound, nodePool[sibling].bound);
        curr = parent;
        parent = nodePool[parent].inner.parent;
    }
}

template <typename Types>
void VBVHShared<Types>::buildLeafRefs()
{
    leafRefs = nodePool.allocatedList();
    uint32_t n = (uint32_t)leafRefs.size();
    for (uint32_t i = 0; i < n;) {
        if (!nodePool[leafRefs[i]].isLeaf() ||
            nodePool[leafRefs[i]].triIndex() == kBackgroundTriIndex) {
            std::swap(leafRefs[i], leafRefs[n - 1]);
            --n;
        } else {
            ++i;
        }
    }
    leafRefs.resize(n);

    // This is needed for range query.
    //std::sort(leafRefs.begin(), leafRefs.end(), [this](uint32_t x, uint32_t y) {
    //    return nodePool[x].triIndex() < nodePool[y].triIndex();
    //});
}

template <typename Types>
void VBVHShared<Types>::removeBackground()
{
    leafRefs = nodePool.allocatedList();
    uint32_t n = (uint32_t)leafRefs.size();
    for (uint32_t i = 0; i < n;) {
        NodeType &node = nodePool[leafRefs[i]];
        if (!node.isLeaf() || node.triIndex() == kBackgroundTriIndex) {
            node.free();
            nodePool.remove(leafRefs[i]);
            std::swap(leafRefs[i], leafRefs[n - 1]);
            --n;
        } else {
            node.leaf.parent = (NodeIndex)~0; // For debug.
            ++i;
        }
    }
    leafRefs.resize(n);

    if (leafRefs.empty()) {
        rootIndex = kEmptyParentNodeIndex;
    } else {
        rootIndex = initOccludeeBVH(0, (uint32_t)leafRefs.size());
        nodePool[rootIndex].inner.parent = kEmptyParentNodeIndex;
        leafRefs.clear();
    }
}

template <typename Types>
NodeIndex VBVHShared<Types>::initOccludeeBVH(uint32_t refStart, uint32_t refEnd)
{
    uint32_t leafCount = refEnd - refStart;
    ASSERT_FATAL(leafCount >= 1);
    if (leafCount == 1) {
        return leafRefs[refStart];
    }

    AABB2 totalBound;
    AABB2 centroidBound;
    for (uint32_t i = refStart; i < refEnd; ++i) {
        totalBound.expand(nodePool[leafRefs[i]].bound);
        centroidBound.expand(nodePool[leafRefs[i]].bound.center());
    }
    float invTotalBoundArea = 1.0f / totalBound.area();

    uint32_t dim = centroidBound.largestAxis();
    struct Bucket
    {
        AABB2 bound;
        uint32_t count = 0;
    };
    constexpr uint32_t bucketCount = 8;
    Bucket buckets[bucketCount];
    for (uint32_t i = refStart; i < refEnd; ++i) {
        uint32_t b = (uint32_t)std::floor(bucketCount * centroidBound.offset(nodePool[leafRefs[i]].bound.center(), dim));
        b = std::min(b, bucketCount - 1);
        ++buckets[b].count;
        buckets[b].bound.expand(nodePool[leafRefs[i]].bound);
    }
    float costs[bucketCount - 1];
    AABB2 leftBounds[bucketCount - 1];
    AABB2 rightBounds[bucketCount - 1];
    for (uint32_t i = 0; i < bucketCount - 1; ++i) {
        uint32_t count0 = 0, count1 = 0;
        for (uint32_t j = 0; j <= i; ++j) {
            leftBounds[i].expand(buckets[j].bound);
            count0 += buckets[j].count;
        }
        for (uint32_t j = i + 1; j < bucketCount; ++j) {
            rightBounds[i].expand(buckets[j].bound);
            count1 += buckets[j].count;
        }
        constexpr float travWeight = 1.0f;
        costs[i] = travWeight +
            (count0 * leftBounds[i].area() + count1 * rightBounds[i].area()) * invTotalBoundArea;
    }

    uint32_t bestBucket = 0;
    for (uint32_t i = 1; i < bucketCount - 1; ++i) {
        if (costs[i] < costs[bestBucket]) {
            bestBucket = i;
        }
    }

    auto mid = std::partition(leafRefs.begin() + refStart, leafRefs.begin() + refEnd, [&](uint32_t p) {
        uint32_t b = (uint32_t)std::floor(bucketCount * centroidBound.offset(nodePool[p].bound.center(), dim));
        b = std::min(b, bucketCount - 1);
        ASSERT(b >= 0 && b < bucketCount);
        return b <= bestBucket;
    });
    uint32_t nl = (uint32_t)std::distance(leafRefs.begin() + refStart, mid);
    uint32_t nr = leafCount - nl;

    AABB2 leftBound = leftBounds[bestBucket];
    AABB2 rightBound = rightBounds[bestBucket];

    if (nl == 0 || nr == 0) {
        // Fall back to middle split.
        float midSplit = centroidBound.center()[dim];
        mid = std::partition(leafRefs.begin() + refStart, leafRefs.begin() + refEnd,
            [&](NodeIndex i) { return nodePool[i].bound.center()[dim] <= midSplit; });
        nl = (uint32_t)std::distance(leafRefs.begin() + refStart, mid);
        nr = leafCount - nl;
        if (nl == 0 || nr == 0) {
            // Fall back to equal count.
            std::nth_element(leafRefs.begin() + refStart, leafRefs.begin() + refStart + (leafCount / 2), leafRefs.begin() + refEnd,
                [&](NodeIndex a, NodeIndex b) {
                return nodePool[a].bound.center()[dim] < nodePool[b].bound.center()[dim];
            });
            nl = leafCount / 2;
            nr = leafCount - nl;
        }
        leftBound = AABB2();
        for (uint32_t i = 0; i < nl; ++i) {
            leftBound.expand(nodePool[leafRefs[i]].bound);
        }
        rightBound = AABB2();
        for (uint32_t i = nl; i < leafCount; ++i) {
            rightBound.expand(nodePool[leafRefs[i]].bound);
        }
    }
    ASSERT_FATAL(nl > 0 && nr > 0);

    uint32_t parent = nodePool.append();
    nodePool[parent].initInner(totalBound, kEmptyParentNodeIndex);
    uint32_t leftChild = initOccludeeBVH(refStart, refStart + nl);
    uint32_t rightChild = initOccludeeBVH(refStart + nl, refEnd);
    nodePool[leftChild].inner.parent = parent;
    nodePool[rightChild].inner.parent = parent;
    nodePool[parent].inner.children[0] = leftChild;
    nodePool[parent].inner.children[1] = rightChild;
    return parent;
}

template <typename Types>
void VBVHShared<Types>::deleteOccluded(TraverseContext &ctx)
{
    for (uint32_t i = 0; i < (uint32_t)ctx.mergeList.size(); ++i) {
        removeLeaf(ctx.mergeList[i].index);
    }
}

template <typename Types>
uint32_t vtrz::VBVHShared<Types>::leafCount() const
{
    return (uint32_t)leafRefs.size();
}

template <typename Types>
typename VBVHShared<Types>::TLeafRegion VBVHShared<Types>::getLeaf(uint32_t leafRefIndex) const
{
    TLeafRegion region;
    region.poly = nodePool[leafRefs[leafRefIndex]].leaf.geom;
    region.vertCount = nodePool[leafRefs[leafRefIndex]].leaf.vertCount;
    region.triId = nodePool[leafRefs[leafRefIndex]].triIndex();
    return region;
}

static inline float saturateCoverage(float c) {
    return saturate(c);
}

static inline dfloat saturateCoverage(dfloat c) {
    c.value = saturate(c.value);
    if (c.value == 1.0) c.zeroGrad();
    return c;
}

template <typename Types>
typename VBVHShared<Types>::TFloat VBVHShared<Types>::coverageQuery() const
{
    TFloat coverage = 0.0f;
    if (rootIndex == kEmptyParentNodeIndex) {
        return coverage;
    }
    float invBoundArea = 1.0f / nodePool[rootIndex].bound.area();
    for (uint32_t i = 0; i < (uint32_t)leafRefs.size(); ++i) {
        const NodeType &leaf = nodePool[leafRefs[i]];
        uint32_t n = leaf.leaf.vertCount;
        for (uint32_t j = 0; j < n; ++j) {
            TPoint2F v0 = cast(leaf.leaf.geom[j]);
            TPoint2F v1 = cast(leaf.leaf.geom[(j + 1) % n]);
            coverage += cross(v0, v1);
        }
    }
    return saturateCoverage(coverage * 0.5f * invBoundArea);
}

template <typename Types>
void VBVHShared<Types>::rangeQuery(RangeQuery &query) const
{
    auto compLo = [this](uint32_t x, uint32_t triIndex) {
        return nodePool[x].triIndex() < triIndex;
    };
    auto compHi = [this](uint32_t triIndex, uint32_t x) {
        return triIndex < nodePool[x].triIndex();
    };
    for (uint32_t i = 0; i < query.triCount; ++i) {
        uint32_t tri = query.triIndices[i];
        auto low = std::lower_bound(leafRefs.begin(), leafRefs.end(), tri, compLo);
        auto high = std::upper_bound(leafRefs.begin(), leafRefs.end(), tri, compHi);
        query.leafRefRanges.push_back({
            (uint32_t)std::distance(leafRefs.begin(), low),
            (uint32_t)std::distance(leafRefs.begin(), high) });
    }
}

template <typename Types>
bool VBVHShared<Types>::pointQuery(const Vector2 &point, NodeIndex &leaf) const
{
    Vector2I64 queries[5];
    queries[0] = cast(point);
    queries[1] = cast(point) + Vector2I64(-kPointQueryRadius, -kPointQueryRadius);
    queries[2] = cast(point) + Vector2I64(kPointQueryRadius, -kPointQueryRadius);
    queries[3] = cast(point) + Vector2I64(kPointQueryRadius, kPointQueryRadius);
    queries[4] = cast(point) + Vector2I64(-kPointQueryRadius, kPointQueryRadius);
    AABB2 queryBound(cast(queries[1]), cast(queries[3]));

    int64_t area2 = 0;
    TraverseStack stack;
    NodeIndex curr = rootIndex;
    while (true) {
        const NodeType &node = nodePool[curr];
        if (!intersectBool(node.bound, queryBound)) {
            if (!stack.pop(curr)) break;
        } else {
            if (node.isLeaf()) {
                if (node.triIndex() != kBackgroundTriIndex) {
                    int64_t newArea2 = convexArea2(node.leaf.geom, node.leaf.vertCount);
                    if (newArea2 > area2) {
                        for (uint32_t i = 0; i < countOf(queries); ++i) {
                            if (pointInConvex(queries[i], node.leaf.geom, node.leaf.vertCount)) {
                                area2 = newArea2;
                                leaf = curr;
                                break;
                            }
                        }
                    }
                }
                if (!stack.pop(curr)) break;
            } else {
                stack.push(node.inner.children[1]);
                curr = node.inner.children[0];
            }
        }
    }

    return area2 > 0;
}

template <typename Types>
typename VBVHShared<Types>::TPointSample VBVHShared<Types>::pointSample() const
{
    TPointSample sample;
    if (leafRefs.empty()) {
        sample.weight = 0.0f;
        return sample;
    }

    float boundArea = nodePool[rootIndex].bound.area();
    float invBoundArea = 1.0f / boundArea;
    TFloat coverage = 0.0f;
    uint32_t largestLeaf = (uint32_t)~0;
    float largestLeafArea = 0.0f;
    for (uint32_t i = 0; i < (uint32_t)leafRefs.size(); ++i) {
        const NodeType &leaf = nodePool[leafRefs[i]];
        uint32_t n = leaf.leaf.vertCount;
        TFloat area = 0.0f;
        for (uint32_t j = 0; j < n; ++j) {
            TPoint2F v0 = cast(leaf.leaf.geom[j]);
            TPoint2F v1 = cast(leaf.leaf.geom[(j + 1) % n]);
            area += cross(v0, v1);
        }
        coverage += area;

        if (getVal(area) > largestLeafArea) {
            largestLeaf = leafRefs[i];
            largestLeafArea = getVal(area);
        }
    }
    coverage *= (0.5f * invBoundArea);
    sample.weight = coverage;
    setVal(sample.weight, saturate(getVal(sample.weight)));
    if (getVal(sample.weight) == 0.0f) {
        return sample;
    }

    Vector2 samplePos;
    NodeIndex sampleLeaf;
    if (getVal(sample.weight) < 0.95f) {
        const NodeType &leaf = nodePool[largestLeaf];
        Vector2I64 centerI64(0);
        for (uint32_t v = 0; v < leaf.leaf.vertCount; ++v) {
            centerI64 += getVal(leaf.leaf.geom[v]);
        }
        samplePos = cast(centerI64) / (float)leaf.leaf.vertCount;
        sampleLeaf = largestLeaf;
    } else {
        samplePos = lerp(nodePool[rootIndex].bound.min, nodePool[rootIndex].bound.max, Vector2(0.5f));
        if (!pointQuery(samplePos, sampleLeaf)) {
            sample.weight = 0.0f;
            return sample;
        }
    }

    sample.triId = nodePool[sampleLeaf].triIndex();
    sample.coord = barycentricCoord(samplePos, getTri2d(sample.triId));
    return sample;
}


template <typename Types>
typename  VBVHShared<Types>::TPointSampleArray VBVHShared<Types>::pointSampleMulti(
    const uint16_t *matIdBuffer, const Vector3 *vertexNormalBuffer) const
{
    struct Surface
    {
        Vector3 accNormal;
        Vector2 depthRange;
        TFloat coverage;
        uint16_t matId;

        Vector2 samplePos;
        uint32_t sampleTriId;
        bool centerSampled;

        bool merge(const Surface &other) {
            if (matId != other.matId) {
                return false;
            }
            float unionRange = std::max(depthRange.y, other.depthRange.y) - std::min(depthRange.x, other.depthRange.x);
            constexpr float depthRangeThreshold = 0.02f;
            if (unionRange > depthRangeThreshold) {
                constexpr float depthOverlapThreshold = 0.5f;
                float depthThreshold = depthOverlapThreshold * std::max(depthRange.y - depthRange.x, other.depthRange.y - other.depthRange.x);
                if (depthRange.x > other.depthRange.y + depthThreshold || depthRange.y + depthThreshold < other.depthRange.x) {
                    return false;
                }
            }
            constexpr float normalThreshold = 0.9f;
            if (dot(accNormal, other.accNormal) < normalThreshold) {
                return false;
            }

            if (other.centerSampled) {
                centerSampled = true;
                samplePos = other.samplePos;
                sampleTriId = other.sampleTriId;
            }

            accNormal = normalize(getVal(coverage) * accNormal + getVal(other.coverage) * accNormal);
            depthRange.x = std::min(depthRange.x, other.depthRange.x);
            depthRange.y = std::max(depthRange.y, other.depthRange.y);
            coverage += other.coverage;

            return true;
        }
    };

    float boundArea = nodePool[rootIndex].bound.area();
    float invBoundArea = 1.0f / boundArea;
    Vector2 center = lerp(nodePool[rootIndex].bound.min, nodePool[rootIndex].bound.max, Vector2(0.5f));

    uint8_t arrSize = 0;
    std::array<Surface, TPointSampleArray::maxSize> surfaces;

    for (uint32_t ref : leafRefs) {
        const NodeType &leaf = nodePool[ref];

        Surface newSurface;
        newSurface.sampleTriId = leaf.triIndex();
        Tri3 tri3 = getVal(getTri3d(newSurface.sampleTriId));
        Tri2 tri2 = { (Vector2)tri3[0], (Vector2)tri3[1], (Vector2)tri3[2] };

        newSurface.coverage = 0.0f;
        bool coverCenter = true;
        Vector2 centroid(0.0f);

        newSurface.depthRange.x = Constants::inf;
        newSurface.depthRange.y = -Constants::inf;

        uint32_t n = leaf.leaf.vertCount;
        for (uint32_t i = 0; i < n; ++i) {
            TPoint2F v0 = cast(leaf.leaf.geom[i]);
            TPoint2F v1 = cast(leaf.leaf.geom[(i + 1) % n]);
            TFloat cr = cross(v0, v1);
            newSurface.coverage += cr;

            coverCenter &= (getVal(cr) >= 0.0f);
            centroid += getVal(v0);

            Vector3 coord = barycentricCoord(getVal(v0), tri2);
            float depth = tri3[0].z * coord.x + tri3[1].z * coord.y + tri3[2].z * coord.z;
            newSurface.depthRange.x = std::min(newSurface.depthRange.x, depth);
            newSurface.depthRange.y = std::max(newSurface.depthRange.y, depth);
        }
        newSurface.coverage *= (0.5f * invBoundArea);
        setVal(newSurface.coverage, saturate(getVal(newSurface.coverage)));
        constexpr float smallCoverageThreshold = 1e-3f;
        if (getVal(newSurface.coverage) <= smallCoverageThreshold) {
            continue;
        }

        if (coverCenter) {
            newSurface.samplePos = center;
            newSurface.centerSampled = true;
        } else {
            newSurface.samplePos = centroid * (1.0f / (float)n);
            newSurface.centerSampled = false;
        }

        newSurface.matId = matIdBuffer[newSurface.sampleTriId];
        if (indexBuffer.size()) {
            uint32_t offset = 3 * newSurface.sampleTriId;
            newSurface.accNormal = normalize(
                vertexNormalBuffer[indexBuffer[offset + 0]] +
                vertexNormalBuffer[indexBuffer[offset + 1]] +
                vertexNormalBuffer[indexBuffer[offset + 2]]);
        } else {
            uint32_t offset = 3 * newSurface.sampleTriId;
            newSurface.accNormal = normalize(
                vertexNormalBuffer[offset + 0] +
                vertexNormalBuffer[offset + 1] +
                vertexNormalBuffer[offset + 2]);
        }

        bool merged = false;
        for (uint8_t i = 0; i < arrSize; ++i) {
            if (surfaces[i].merge(newSurface)) {
                merged = true;
                break;
            }
        }
        if (!merged) {
            if (arrSize < TPointSampleArray::maxSize) {
                surfaces[arrSize++] = newSurface;
            } else {
                uint8_t smallest = 0;
                for (uint8_t i = 1; i < arrSize; ++i) {
                    if (getVal(surfaces[i].coverage) < getVal(surfaces[smallest].coverage)) {
                        smallest = i;
                    }
                }
                if (getVal(newSurface.coverage) > getVal(surfaces[smallest].coverage)) {
                    surfaces[smallest] = newSurface;
                }
            }
        }
    }

    TPointSampleArray samples;
    samples.size = arrSize;
    for (uint8_t i = 0; i < samples.size; ++i) {
        TTri2 tri2 = getTri2d(surfaces[i].sampleTriId);
        samples.entries[i].coord = barycentricCoord(surfaces[i].samplePos, tri2);
        samples.entries[i].triId = surfaces[i].sampleTriId;
        samples.entries[i].weight = surfaces[i].coverage;
    }

    return samples;
}

template <typename Types>
void VBVHShared<Types>::exportTo(const char *outputPath) const
{
    std::ofstream file(outputPath);

    // Skip the first face, which is the boundary.
    for (NodeIndex leaf : leafRefs) {
        const NodeType &leafNode = nodePool[leaf];
        uint32_t triIndex = leafNode.triIndex();
        // if (triIndex == kBackgroundTriIndex) continue;
        file << triIndex << ",  ";
        ASSERT(leafNode.leaf.vertCount >= 3);
        for (uint32_t i = 0; i < leafNode.leaf.vertCount; ++i) {
            // Cast back to double.
            Vector2I64 pos = leafNode.vertPos(i);
            double x = (double)pos.x * kInvScaleFactor;
            double y = (double)pos.y * kInvScaleFactor;
            file << x << ", " << y;
            if (i < leafNode.leaf.vertCount - 1) {
                file << ", ";
            }
        }
        file << "\n";
    }
}

}