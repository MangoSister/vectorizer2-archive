#include "vectorizer.h"
#include "vbvh.h"
#include "vbvhGrad.h"

namespace vtrz
{

RangeQueryInterface::RangeQueryInterface() : query(new RangeQuery) { }

RangeQueryInterface::~RangeQueryInterface()
{
    delete query;
    query = nullptr;
}

void RangeQueryInterface::reset(const uint32_t *newTriIndices, uint32_t newTriCount)
{
    query->reset(newTriIndices, newTriCount);
}

const std::pair<uint32_t, uint32_t> *RangeQueryInterface::leafRefRanges() const
{
    return query->leafRefRanges.data();
}

uint32_t RangeQueryInterface::leafRefRangeCount() const
{
    return (uint32_t)query->leafRefRanges.size();
}

Vectorizer::Vectorizer() : vbvh(new VBVH) { }

Vectorizer::~Vectorizer()
{
    delete vbvh;
    vbvh = nullptr;
}

void Vectorizer::reset(const AABB2 &bound, DepthFunc depthFunc)
{
    vbvh->reset(bound, depthFunc);
}

void Vectorizer::reset(const Vector2 *rootShape, uint32_t vertCount, DepthFunc depthFunc)
{
    vbvh->reset(rootShape, vertCount, depthFunc);
}

void Vectorizer::build(
    const Vector3 *vertexBuffer, uint32_t vertexCount,
    const uint32_t *indexBuffer, uint32_t indexCount,
    bool enableMerge)
{
    vbvh->build(vertexBuffer, vertexCount, indexBuffer, indexCount, enableMerge);
}

void Vectorizer::buildOcclusion(
    const Vector3 *vertexBuffer, uint32_t vertexCount,
    const uint32_t *indexBuffer, uint32_t indexCount,
    const uint32_t *occludeeList, uint32_t occludeeCount)
{
    vbvh->buildOcclusion(vertexBuffer, vertexCount, indexBuffer, indexCount, occludeeList, occludeeCount);
}

void Vectorizer::simplify(const uint16_t *meshIds, const uint32_t *primIds)
{
    vbvh->simplify(meshIds, primIds);
}

//////////////////////////////////////////////////////////////////////////
void Vectorizer::reset(const AABB2 &bound, uint32_t vhizRes, DepthFunc depthFunc)
{
    vbvh->reset(bound, vhizRes, depthFunc);
}

void Vectorizer::addOccludees(const Vector3 *vertexBuffer, uint32_t vertexCount)
{
    vbvh->addOccludees(vertexBuffer, vertexCount);
}

void Vectorizer::finishOccludees()
{
    vbvh->finishOccludees();
}

bool Vectorizer::queryOccluder(const AABB2 &box, float depth) const
{
    return vbvh->queryOccluders(box, depth);
}

bool Vectorizer::queryOccluders(const Vector3 *vertexBuffer, uint32_t vertexCount) const
{
    return vbvh->queryOccluders(vertexBuffer, vertexCount);
}

void Vectorizer::addOccluders(const Vector3 *vertexBuffer, uint32_t vertexCount)
{
    vbvh->addOccluders(vertexBuffer, vertexCount);
}

void Vectorizer::finishOcclusion()
{
    vbvh->finishOcclusion();
}
//////////////////////////////////////////////////////////////////////////

void Vectorizer::rangeQuery(RangeQueryInterface &query) const
{
    vbvh->rangeQuery(*query.query);
}

uint32_t Vectorizer::leafCount() const
{
    return vbvh->leafCount();
}

LeafRegion Vectorizer::getLeaf(uint32_t leafRefIndex) const
{
    return vbvh->getLeaf(leafRefIndex);
}

PointSample Vectorizer::pointSample() const
{
    return vbvh->pointSample();
}

vtrz::PointSampleArray Vectorizer::pointSampleMulti(const uint16_t *matIdBuffer, const Vector3 *vertexNormalBuffer) const
{
    return vbvh->pointSampleMulti(matIdBuffer, vertexNormalBuffer);
}

float Vectorizer::coverageQuery() const
{
    return vbvh->coverageQuery();
}

float Vectorizer::triangleCoverageQuery(const Tri3 &tri,
    uint16_t refMeshId, uint32_t refPrimId,
    const uint16_t *meshIds, const uint32_t *primIds,
    const Vector3 *worldPositions, const float *wInvs,
    const Matrix4x4 &queryTransform) const
{
    return vbvh->triangleCoverageQuery(
        tri, refMeshId, refPrimId, meshIds, primIds,
        worldPositions, wInvs, queryTransform);
}

void Vectorizer::exportTo(const char *outputPath) const
{
    return vbvh->exportTo(outputPath);
}

void Vectorizer::exportVHiZTo(const char *outputPath) const
{
    return vbvh->vhiz.exportTo(outputPath);
}

VectorizerGrad::VectorizerGrad() : vbvhGrad(new VBVHGrad) { }

VectorizerGrad::~VectorizerGrad()
{
    delete vbvhGrad;
    vbvhGrad = nullptr;
}

void VectorizerGrad::reset(const AABB2 &bound, DepthFunc depthFunc)
{
    vbvhGrad->reset(bound, depthFunc);
}

void VectorizerGrad::reset(const DVector2 *rootShape, uint32_t vertCount, DepthFunc depthFunc)
{
    vbvhGrad->reset(rootShape, vertCount, depthFunc);
}

void VectorizerGrad::build(
    const DVector3 *vertexBuffer, uint32_t vertexCount,
    const uint32_t *indexBuffer, uint32_t indexCount,
    bool enableMerge)
{
    vbvhGrad->build(vertexBuffer, vertexCount, indexBuffer, indexCount, enableMerge);
}

void VectorizerGrad::buildOcclusion(
    const DVector3 *vtxBuffer, uint32_t vertCount,
    const uint32_t *idxBuffer, uint32_t idxCount,
    const uint32_t *occludeeList, uint32_t occludeeCount)
{
    vbvhGrad->buildOcclusion(vtxBuffer, vertCount, idxBuffer, idxCount, occludeeList, occludeeCount);
}

void VectorizerGrad::simplify(const uint16_t *meshIds, const uint32_t *primIds)
{
    vbvhGrad->simplify(meshIds, primIds);
}

void VectorizerGrad::rangeQuery(RangeQueryInterface &query) const
{
    vbvhGrad->rangeQuery(*query.query);
}

uint32_t VectorizerGrad::leafCount() const
{
    return vbvhGrad->leafCount();
}

LeafRegionGrad VectorizerGrad::getLeaf(uint32_t leafRefIndex) const
{
    return vbvhGrad->getLeaf(leafRefIndex);
}

PointSampleGrad VectorizerGrad::pointSample() const
{
    return vbvhGrad->pointSample();
}

PointSampleArrayGrad VectorizerGrad::pointSampleMulti(const uint16_t *matIdBuffer, const Vector3 *vertexNormalBuffer) const
{
    return vbvhGrad->pointSampleMulti(matIdBuffer, vertexNormalBuffer);
}

dfloat VectorizerGrad::coverageQuery() const
{
    return vbvhGrad->coverageQuery();
}

dfloat VectorizerGrad::triangleCoverageQuery(
    const Tri3Grad &tri, uint16_t refMeshId, uint32_t refPrimId,
    const uint16_t *meshIds, const uint32_t *primIds,
    const DVector3 *worldPositions, const dfloat *wInvs,
    const Matrix4x4 &queryTransform) const
{
    return vbvhGrad->triangleCoverageQuery(
        tri, refMeshId, refPrimId, meshIds, primIds,
        worldPositions, wInvs, queryTransform);
}

}