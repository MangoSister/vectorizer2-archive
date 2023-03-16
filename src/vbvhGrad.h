#pragma once

#include "vbvhShared.h"

namespace vtrz
{

struct ConvexIntersectorGrad;

struct VBVHGradTypes
{
    using TFloat = dfloat;
    using TPoint2I = DVector2I64;
    using TPoint2F = DVector2;
    using TPoint3F = DVector3;
    using TTri2 = Tri2Grad;
    using TTri3 = Tri3Grad;
    using TIsector = ConvexIntersectorGrad;
    using TLeafRegion = LeafRegionGrad;
    using TPointSample = PointSampleGrad;
    using TPointSampleArray = PointSampleArrayGrad;
};

struct VBVHGrad : public VBVHShared<VBVHGradTypes>
{
public:
    void reset(const AABB2 &bound, DepthFunc depthFunc = DepthFunc::eLess);
    void reset(const DVector2 *rootShape, uint32_t vertCount, DepthFunc depthFunc = DepthFunc::eLess);
    void build(
        const DVector3 *vertexBuffer, uint32_t vertexCount,
        const uint32_t *indexBuffer, uint32_t indexCount,
        bool enableMerge = true);
    void buildOcclusion(
        const DVector3 *vertexBuffer, uint32_t vertexCount,
        const uint32_t *indexBuffer, uint32_t indexCount,
        const uint32_t *occludeeList, uint32_t occludeeCount);

    void simplify(const uint16_t *meshIds, const uint32_t *primIds);
    //////////////////////////////////////////////////////////////////////////

    void traverse(TraverseContext &ctx, ConvexIntersectorGrad &conv, TraversalType type);
    void merge(TraverseContext &ctx);

    dfloat triangleCoverageQuery(const Tri3Grad &tri,
        uint16_t refMeshId, uint32_t refPrimId,
        const uint16_t *meshIds, const uint32_t *primIds,
        const DVector3 *worldPositions, const dfloat *wInvs,
        const Matrix4x4 &queryTransform) const;
};

}