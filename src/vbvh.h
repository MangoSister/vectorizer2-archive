#pragma once

#include "vbvhShared.h"
#include "vhiz.h"

namespace vtrz
{

struct ConvexIntersector;

struct VBVHTypes
{
    using TFloat = float;
    using TPoint2I = Vector2I64;
    using TPoint2F = Vector2;
    using TPoint3F = Vector3;
    using TTri2 = Tri2;
    using TTri3 = Tri3;
    using TIsector = ConvexIntersector;
    using TLeafRegion = LeafRegion;
    using TPointSample = PointSample;
    using TPointSampleArray = PointSampleArray;
};

struct VBVH : public VBVHShared<VBVHTypes>
{
public:
    void reset(const AABB2 &bound, DepthFunc depthFunc = DepthFunc::eLess);
    void reset(const Vector2 *rootShape, uint32_t vertCount, DepthFunc depthFunc = DepthFunc::eLess);
    void build(
        const Vector3 *vertexBuffer, uint32_t vertexCount,
        const uint32_t *indexBuffer, uint32_t indexCount,
        bool enableMerge = true);
    void buildOcclusion(
        const Vector3 *vertexBuffer, uint32_t vertexCount,
        const uint32_t *indexBuffer, uint32_t indexCount,
        const uint32_t *occludeeList, uint32_t occludeeCount);

    void simplify(const uint16_t *meshIds, const uint32_t *primIds);
    //////////////////////////////////////////////////////////////////////////

    void traverse(TraverseContext &ctx, ConvexIntersector &conv, TraversalType type);
    void merge(TraverseContext &ctx);

    float triangleCoverageQuery(const Tri3 &tri,
        uint16_t refMeshId, uint32_t refPrimId,
        const uint16_t *meshIds, const uint32_t *primIds,
        const Vector3 *worldPositions, const float *wInvs,
        const Matrix4x4 &queryTransform) const;

    //////////////////////////////////////////////////////////////////////////
    void reset(const AABB2 &bound, uint32_t vhizRes, DepthFunc depthFunc = DepthFunc::eLess);
    void addOccludees(const Vector3 *vertexBuffer, uint32_t vertexCount);
    void finishOccludees();
    bool queryOccluders(const AABB2 &box, float depth) const;
    bool queryOccluders(const Vector3 *vertexBuffer, uint32_t vertexCount) const;
    void addOccluders(const Vector3 *vertexBuffer, uint32_t vertexCount);
    void finishOcclusion();

    bool queryBoxOcclusion(const AABB2 &box) const;

    void updateVHiZOccludee(NodeIndex nodeIdx);
    void updateVHiZOccluder(const Vector2I64 *shape, uint32_t vertCount, std::vector<Vector2I> &occluderCells);

    VHiZ vhiz;
};

}