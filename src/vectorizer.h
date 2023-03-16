#pragma once

#include "api.h"
#include "maths.h"
#include "diff.h"
#include "cast.h"

namespace vtrz
{

struct RangeQuery;
struct VBVH;
struct VBVHGrad;

struct VECTORIZER_API PointSample
{
    Vector3 coord; // (coord.x, coord.y, 1 - coord.x - coord.y)
    uint32_t triId;
    float weight;
};

struct VECTORIZER_API PointSampleArray
{
    static constexpr uint8_t maxSize = 4;
    PointSample entries[maxSize];
    uint8_t size;
};

#pragma warning( push )
#pragma warning( disable: 4251 )
struct VECTORIZER_API PointSampleGrad
{
    DVector3 coord;
    uint32_t triId;
    dfloat weight;
};

struct VECTORIZER_API PointSampleArrayGrad
{
    static constexpr uint8_t maxSize = 4;
    PointSampleGrad entries[maxSize];
    uint8_t size;
};
#pragma warning( pop )

struct VECTORIZER_API LeafRegion
{
    const Vector2I64 *poly;
    uint32_t vertCount;
    uint32_t triId;
};

struct VECTORIZER_API LeafRegionGrad
{
    const DVector2I64 *poly;
    uint32_t vertCount;
    uint32_t triId;
};

struct VECTORIZER_API RangeQueryInterface
{
    RangeQueryInterface();
    ~RangeQueryInterface();

    void reset(const uint32_t *newTriIndices, uint32_t newTriCount);
    const std::pair<uint32_t, uint32_t> *leafRefRanges() const;
    uint32_t leafRefRangeCount() const;

private:
    RangeQuery *query = nullptr;

    friend class Vectorizer;
    friend class VectorizerGrad;
};

enum class DepthFunc
{
    eLess,
    eGreater,
};

class VECTORIZER_API Vectorizer
{
public:
    Vectorizer();
    ~Vectorizer();

    void reset(const AABB2 &bound, DepthFunc depthFunc);
    void reset(const Vector2 *rootShape, uint32_t vertCount, DepthFunc depthFunc);
    void build(
        const Vector3 *vertexBuffer, uint32_t vertexCount,
        const uint32_t *indexBuffer, uint32_t indexCount,
        bool enableMerge = true);
    void buildOcclusion(
        const Vector3 *vtxBuffer, uint32_t vertCount,
        const uint32_t *idxBuffer, uint32_t idxCount,
        const uint32_t *occludeeList, uint32_t occludeeCount);
    void simplify(const uint16_t *meshIds, const uint32_t *primIds);
    //////////////////////////////////////////////////////////////////////////
    void reset(const AABB2 &bound, uint32_t vhizRes, DepthFunc depthFunc);
    void addOccludees(const Vector3 *vertexBuffer, uint32_t vertexCount);
    void finishOccludees();
    bool queryOccluder(const AABB2 &box, float depth) const;
    bool queryOccluders(const Vector3 *vertexBuffer, uint32_t vertexCount) const;
    void addOccluders(const Vector3 *vertexBuffer, uint32_t vertexCount);
    void finishOcclusion();
    //////////////////////////////////////////////////////////////////////////
    void rangeQuery(RangeQueryInterface &query) const;
    uint32_t leafCount() const;
    LeafRegion getLeaf(uint32_t leafRefIndex) const;
    PointSample pointSample() const;
    PointSampleArray pointSampleMulti(const uint16_t *matIdBuffer, const Vector3 *vertexNormalBuffer) const;
    float coverageQuery() const;
    float triangleCoverageQuery(const Tri3 &tri,
        uint16_t refMeshId, uint32_t refPrimId,
        const uint16_t *meshIds, const uint32_t *primIds,
        const Vector3 *worldPositions, const float *wInvs,
        const Matrix4x4 &queryTransform) const;
    //////////////////////////////////////////////////////////////////////////
    void exportTo(const char *outputPath) const;
    void exportVHiZTo(const char *outputPath) const;
private:
    VBVH *vbvh = nullptr;
};

class VECTORIZER_API VectorizerGrad
{
public:
    VectorizerGrad();
    ~VectorizerGrad();

    void reset(const AABB2 &bound, DepthFunc depthFunc);
    void reset(const DVector2 *rootShape, uint32_t vertCount, DepthFunc depthFunc);
    void build(
        const DVector3 *vertexBuffer, uint32_t vertexCount,
        const uint32_t *indexBuffer, uint32_t indexCount,
        bool enableMerge = true);
    void buildOcclusion(
        const DVector3 *vtxBuffer, uint32_t vertCount,
        const uint32_t *idxBuffer, uint32_t idxCount,
        const uint32_t *occludeeList, uint32_t occludeeCount);
    void simplify(const uint16_t *meshIds, const uint32_t *primIds);

    void rangeQuery(RangeQueryInterface &query) const;
    uint32_t leafCount() const;
    LeafRegionGrad getLeaf(uint32_t leafRefIndex) const;
    PointSampleGrad pointSample() const;
    PointSampleArrayGrad pointSampleMulti(const uint16_t *matIdBuffer, const Vector3 *vertexNormalBuffer) const;
    dfloat coverageQuery() const;
    dfloat triangleCoverageQuery(const Tri3Grad &tri,
        uint16_t refMeshId, uint32_t refPrimId,
        const uint16_t *meshIds, const uint32_t *primIds,
        const DVector3 *worldPositions, const dfloat *wInvs,
        const Matrix4x4 &queryTransform) const;

private:
    VBVHGrad *vbvhGrad = nullptr;
};

}