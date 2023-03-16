#pragma once

#include "util.h"
#include <mitsuba/mitsuba.h>
#include <mitsuba/core/transform.h>
#include <mitsuba/core/aabb.h>
#include <mitsuba/render/scene.h>
#include <mitsuba/render/trimesh.h>
#include <vectorizer/vectorizer.h>
#include <vector>

MTS_NAMESPACE_BEGIN

struct FrustumBVHIsectResult;
struct BVH;
struct SceneAD;

struct PrimaryPipelineAD
{
    // Per-vertex.
    std::vector<vtrz::DVector3> positions;
    std::vector<vtrz::DVector3> worldPositions;
    std::vector<Normal> normals;
    std::vector<vtrz::dfloat> wInvs;

    // Per-face.
    std::vector<uint16_t> meshIds;
    std::vector<uint16_t> matIds;
    std::vector<uint32_t> primIds;

    vtrz::DMatrix4x4 vpTrans;
    Point apex;

    void clear();
    void add(vtrz::DVector4 *pos1, vtrz::DVector3 *world1,  Normal *normal1,
        uint32_t meshId, uint16_t matId, uint32_t primId, bool noClip);
    void run(const Scene &scene, const BVH &bvh, const SceneAD &sceneAD, const FrustumBVHIsectResult &isects);

    IntersectionGrad buildItsFromSample(const vtrz::PointSampleGrad &entry, const Point &sensorPos,
        const Scene &scene, const SceneAD &sceneAD) const;
    bool buildItsPatch(const vtrz::LeafRegionGrad &region, const Point &sensorPos,
        const Scene &scene, const SceneAD &sceneAD, IntersectionPatchGrad &patch) const;
};

struct SecondaryPipelineAD
{
    std::vector<vtrz::DVector3> positions;

    std::vector<uint32_t> queryTriIndices;

    vtrz::DMatrix4x4 vpTrans;
    Point apex;

    void clear();
    void add(vtrz::DVector4 *pos1, bool query, bool noClip);
    void run(const Scene &scene, const BVH &bvh, const SceneAD &gradCache,
        const FrustumBVHIsectResult &isects, const TriMesh *queryMesh);
    void add(const IntersectionPatchGrad &patch);
};

MTS_NAMESPACE_END