#pragma once

#include "util.h"
#include <mitsuba/mitsuba.h>
#include <mitsuba/core/transform.h>
#include <mitsuba/core/aabb.h>
#include <mitsuba/render/scene.h>
#include <mitsuba/render/trimesh.h>
#include <vectorizer/vectorizer.h>
#include <vector>
#include <array>

MTS_NAMESPACE_BEGIN

struct FrustumBVHIsectEntry;
struct FrustumBVHIsectResult;
struct BVH;

struct PrimaryPipeline
{
    // Per-vertex.
    std::vector<Point> positions;
    std::vector<Point> worldPositions;
    std::vector<Normal> normals;
    std::vector<float> wInvs;

    // Per-face.
    std::vector<uint16_t> meshIds;
    std::vector<uint16_t> matIds;
    std::vector<uint32_t> primIds;

    Transform vpTrans;
    Point apex;

    void clear();
    void add(Vector4 *pos1, Point *world1, Normal *normal1,
        uint16_t meshId, uint16_t matId, uint32_t primId, bool noClip);
    void run(const Scene &scene, const BVH &bvh, const FrustumBVHIsectResult &isects);

    Intersection buildItsFromSample(const vtrz::PointSample &entry, const Point &sensorPos, const Scene &scene) const;
    bool buildItsPatch(const vtrz::LeafRegion &region, const Point &sensorPos, const Scene &scene, IntersectionPatch &patch) const;
};

struct SecondaryPipeline
{
    std::vector<Point> positions;

    std::vector<uint32_t> queryTriIndices;

    Transform vpTrans;
    Point apex;

    void clear();
    void add(Vector4 *pos1, bool query, bool noClip);
    void run(const Scene &scene, const BVH &bvh, const FrustumBVHIsectResult &isects, const TriMesh *queryMesh);
    void add(const IntersectionPatch &patch);
};

struct AABBClipper
{
    static constexpr uint32_t kMaxVertCount = 108;
    std::array<Point, kMaxVertCount> positions;
    uint32_t vertCount = 0;

    void run(const Point &apex, const AABB &aabb, const Transform &vpTrans, bool noClip);
    void add(const Transform &vpTrans, bool noClip, const std::array<Point, 3> &face);
};

struct SecondaryPipelineInc
{
    std::vector<Point> positions;

    Transform vpTrans;
    Point apex;

    void clear();
    void add(Vector4 *pos1, bool noClip);
    void run(const TriMesh *queryMesh);
    void run(const Scene &scene, const BVH &bvh, const FrustumBVHIsectEntry &entry, const TriMesh *queryMesh);
};

MTS_NAMESPACE_END