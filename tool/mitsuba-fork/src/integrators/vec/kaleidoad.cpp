#include "util.h"
#include "pipeline.h"
#include "pipelinead.h"
#include "scenead.h"
#include "bvh.h"
#include <array>
#include <queue>
#include <vectorizer/vectorizer.h>
#include <boost/filesystem/path.hpp>

MTS_NAMESPACE_BEGIN

class KaleidoscopeAD final : public SamplingIntegrator
{
public:
    MTS_DECLARE_CLASS();

    KaleidoscopeAD(const Properties &props);

    KaleidoscopeAD(Stream *stream, InstanceManager *manager);

    bool preprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID) final;

    void postprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) final;

    void renderBlock(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points) const final;

    Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const final { return Spectrum(0.0); }

    struct BacktrackRecordAD
    {
        vtrz::DVector3 beamOrigin;
        vtrz::DVector4 plane;
    };

    struct TracePayloadAD
    {
        BeamGrad beam;
        std::array<vtrz::DVector3, 16> patch;
        std::array<BacktrackRecordAD, 16> backtrack;
        uint16_t vertCount = 0;
        uint16_t depth = 0;
    };

    TracePayloadAD createPayload(
        const vtrz::LeafRegionGrad &leaf, const PrimaryPipelineAD &pipeline, const TracePayloadAD &old, const ViewInfo &viewInfo) const;

    vtrz::dfloat propagateCoverage(const vtrz::LeafRegionGrad &leaf, const PrimaryPipelineAD &pipeline,
        const TracePayloadAD &payload, const vtrz::Matrix4x4 &initTrans) const;

    void writePixel(const Point2i &pixel, const vtrz::DVector3 &L, ImageBlock &block) const;

private:
    BVH bvh;
    uint32_t maxDepth;

    uint32_t totalGradDim = 0;
    vtrz::DMatrix4x4 prismToWorldAD;
    SceneAD sceneAD;
    mutable ref_vector<Bitmap> gradImages;
};

KaleidoscopeAD::KaleidoscopeAD(const Properties &props) :
    SamplingIntegrator(props)
{
    maxDepth = props.getInteger("maxDepth", 5);
}

KaleidoscopeAD::KaleidoscopeAD(Stream *stream, InstanceManager *manager) :
    SamplingIntegrator(stream, manager)
{
    maxDepth = stream->readInt();
}

bool KaleidoscopeAD::preprocess(
    const Scene *scene, RenderQueue *queue, const RenderJob *job,
    int sceneResID, int sensorResID, int samplerResID)
{
    bvh.build(*scene);

    totalGradDim = 1;
    prismToWorldAD = vtrz::makeRotate(vtrz::dfloat(0.0, 0), vtrz::DVector3(1.0f, 0.0f, 0.0f));

    auto &meshes = scene->getMeshes();
    for (uint32_t i = 0; i < meshes.size(); ++i) {
        if (meshes[i]->getID() == "prism") {
            sceneAD.indexMap.insert({ i, (uint32_t)sceneAD.meshADs.size() });
            sceneAD.meshADs.push_back(TriMeshAD(prismToWorldAD, TransformADType::eWorld, *meshes[i]));
            break;
        }
    }

    gradImages.resize(totalGradDim);
    for (uint32_t i = 0; i < totalGradDim; ++i) {
        gradImages[i] = new Bitmap(Bitmap::EPixelFormat::ERGB, Bitmap::EComponentFormat::EFloat32,
            scene->getFilm()->getCropSize());
        gradImages[i]->clear();
    }

    return true;
}

// Note that reflection matrix reverses handedness.
static vtrz::DMatrix4x4 makeReflect(const vtrz::DVector3 &n, const vtrz::DVector3 &p)
{
    vtrz::dfloat d = vtrz::dot(n, p);
    vtrz::DMatrix4x4 R(
        1.0f - 2.0f*n.x*n.x, -2.0f*n.y*n.x, -2.0f*n.z*n.x, 2.0f*d*n.x,
        -2.0f*n.x*n.y, 1.0f - 2.0f*n.y*n.y, -2.0f*n.z*n.y, 2.0f*d*n.y,
        -2.0f*n.x*n.z, -2.0f*n.y*n.z, 1.0f - 2.0f*n.z*n.z, 2.0f*d*n.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
    return R;
}

static Point rayPlaneIsect(const Point &ro, const Vector &rd, const Vector4 &plane)
{
    Vector n(plane.x, plane.y, plane.z);
    Float w = plane.w;
    Float t = -(w + dot(Vector(ro), n)) / dot(rd, n);
    SAssert(t > 0.0f);
    return ro + t * rd;
}

static vtrz::DVector3 rayPlaneIsectAD(const vtrz::DVector3 &ro, const vtrz::DVector3 &rd, const vtrz::DVector4 &plane)
{
    vtrz::DVector3 n(plane.x, plane.y, plane.z);
    vtrz::dfloat w = plane.w;
    vtrz::dfloat t = -(w + vtrz::dot(ro, n)) / vtrz::dot(rd, n);
    SAssert(t.value > 0.0f);
    return ro + t * rd;
}

static BeamGrad reflectBeamAD(
    const BeamGrad &old, Float nearClip, Float vFov, Float aspect,
    const vtrz::DVector3 &n, const vtrz::DVector3 &p, uint32_t depth)
{
    vtrz::DVector4 plane(n.x, n.y, n.z, -vtrz::dot(n, p));
    Vector4 planeValue(plane.x.value, plane.y.value, plane.z.value, plane.w.value);

    std::array<Point, 4> footPrint;
    vtrz::Matrix4x4 oldInvVp = (old.projTrans * old.viewTrans.value()).inverse();

    for (uint32_t i = 0; i < 4; ++i) {
        Vector rd = toVector((vtrz::Vector3)(oldInvVp * vtrz::Vector4(i / 2 ? 1.0f : -1.0f, i % 2 ? 1.0f : -1.0f, 0.0f, 1.0f)));
        footPrint[i] = rayPlaneIsect(old.frustum.apex, rd, planeValue);
    }

    BeamGrad beam;

    vtrz::DMatrix4x4 R = makeReflect(n, p);
    beam.viewTrans = old.viewTrans * R.inverse();
    vtrz::Matrix4x4 viewTransValue = beam.viewTrans.value();
    vtrz::Matrix4x4 viewToWorld = viewTransValue.inverse();
    Point origin(
        viewToWorld.arr[0][3],
        viewToWorld.arr[1][3],
        viewToWorld.arr[2][3]
    );

    beam.projTrans = castMatrix4x4(revInfPerspective(nearClip, vFov, aspect, AABB2(Point2(-1.0f), Point2(1.0f))));
    vtrz::Matrix4x4 vp = beam.projTrans * viewTransValue;
    AABB2 newNdcBound;
    for (uint32_t i = 0; i < 4; ++i) {
        vtrz::Vector4 h = vp * vtrz::Vector4(footPrint[i].x, footPrint[i].y, footPrint[i].z, 1.0f);
        h /= h.w;
        SAssert(h.x >= -1.0f - 1e4f && h.x <= 1.0f + 1e4f);
        SAssert(h.y >= -1.0f - 1e4f && h.y <= 1.0f + 1e4f);
        SAssert(h.z >= 0.0f - 1e4f && h.z <= 1.0f + 1e4f);
        newNdcBound.expandBy(Point2(h.x, h.y));
    }

    beam.projTrans = castMatrix4x4(revInfPerspective(nearClip, vFov, aspect, newNdcBound));
    vp = beam.projTrans * viewTransValue;
    vtrz::Matrix4x4 invVp = vp.inverse();
    std::array<Vector, 4> frustumCornerDirs = {
        Vector(-1.0f, -1.0f, 0.0f),
        Vector(1.0f, -1.0f, 0.0f),
        Vector(1.0f, 1.0f, 0.0f),
        Vector(-1.0f, 1.0f, 0.0f)
    };
    for (uint32_t i = 0; i < 4; ++i) {
        frustumCornerDirs[i] = toVector((vtrz::Vector3)(invVp * vtrz::Vector4(toVec3(frustumCornerDirs[i]), 1.0f)));
    }
    Vector centerDir = toVector((vtrz::Vector3)(invVp * vtrz::Vector4(0.0f, 0.0f, 0.0f, 1.0f)));

    if (depth % 2) {
        // Flip handedness.
        std::swap(frustumCornerDirs[1], frustumCornerDirs[3]);
    }

    beam.frustum = Frustum(origin, centerDir, frustumCornerDirs.data(), kSecondaryNearClip, FrustumAABBOption::eCoarse);

    return beam;
}

KaleidoscopeAD::TracePayloadAD KaleidoscopeAD::createPayload(
    const vtrz::LeafRegionGrad &leaf, const PrimaryPipelineAD &pipeline, const TracePayloadAD &old, const ViewInfo &viewInfo) const
{
    TracePayloadAD payload;

    payload.depth = old.depth + 1;
    SAssert(payload.depth <= (uint32_t)payload.backtrack.size());

    uint32_t offset = leaf.triId * 3;
    const vtrz::DVector3 &wp0 = pipeline.worldPositions[offset + 0];
    const vtrz::DVector3 &wp1 = pipeline.worldPositions[offset + 1];
    const vtrz::DVector3 &wp2 = pipeline.worldPositions[offset + 2];

    vtrz::DVector3 n = vtrz::normalize(vtrz::cross(wp1 - wp0, wp2 - wp1));
    Vector nval(n.x.value, n.y.value, n.z.value);

    std::copy(old.backtrack.begin(), old.backtrack.begin() + old.depth, payload.backtrack.begin());
    vtrz::DVector3 lastBeamOrigin;
    if (old.depth > 0) {
        lastBeamOrigin = old.backtrack[old.depth - 1].beamOrigin;
    } else {
        lastBeamOrigin = vtrz::DVector3(toVec3(pipeline.apex));
    }
    payload.backtrack[payload.depth - 1].beamOrigin = lastBeamOrigin - 2.0f * vtrz::dot(lastBeamOrigin - wp0, n) * n;
    payload.backtrack[payload.depth - 1].plane = vtrz::DVector4(n.x, n.y, n.z, -vtrz::dot(n, wp0));

    payload.vertCount = leaf.vertCount;
    SAssert(payload.vertCount <= (uint32_t)payload.patch.size());

    const vtrz::DVector3 &p0 = pipeline.positions[offset + 0];
    const vtrz::DVector3 &p1 = pipeline.positions[offset + 1];
    const vtrz::DVector3 &p2 = pipeline.positions[offset + 2];
    vtrz::Tri2Grad tri2d = {
        vtrz::DVector2(p0.x, p0.y),
        vtrz::DVector2(p1.x, p1.y),
        vtrz::DVector2(p2.x, p2.y),
    };
    for (uint32_t v = 0; v < leaf.vertCount; ++v) {
        vtrz::DVector2 vert = vtrz::cast(leaf.poly[v]);
        // NOTE: no clamp
        vtrz::DVector3 perspCoord = vtrz::barycentricCoordNoClamp(vert, tri2d);
        perspCoord.x *= pipeline.wInvs[offset + 0];
        perspCoord.y *= pipeline.wInvs[offset + 1];
        perspCoord.z *= pipeline.wInvs[offset + 2];
        perspCoord /= (perspCoord.x + perspCoord.y + perspCoord.z);
        payload.patch[v] = wp0 * perspCoord.x + wp1 * perspCoord.y + wp2 * perspCoord.z;
    }

    payload.beam = reflectBeamAD(
        old.beam, viewInfo.nearClip, viewInfo.vFov, viewInfo.aspect, n, payload.patch[0], payload.depth);

    return payload;
}

vtrz::dfloat KaleidoscopeAD::propagateCoverage(
    const vtrz::LeafRegionGrad &leaf, const PrimaryPipelineAD &pipeline, const TracePayloadAD &payload, const vtrz::Matrix4x4 &initTrans) const
{
    uint32_t offset = leaf.triId * 3;
    const vtrz::DVector3 &wp0 = pipeline.worldPositions[offset + 0];
    const vtrz::DVector3 &wp1 = pipeline.worldPositions[offset + 1];
    const vtrz::DVector3 &wp2 = pipeline.worldPositions[offset + 2];

    const vtrz::DVector3 &p0 = pipeline.positions[offset + 0];
    const vtrz::DVector3 &p1 = pipeline.positions[offset + 1];
    const vtrz::DVector3 &p2 = pipeline.positions[offset + 2];
    vtrz::Tri2Grad tri2d = {
        vtrz::DVector2(p0.x, p0.y),
        vtrz::DVector2(p1.x, p1.y),
        vtrz::DVector2(p2.x, p2.y),
    };

    vtrz::dfloat coverage(0.0f);
    vtrz::DVector2 v0, vlast;
    for (uint32_t v = 0; v < leaf.vertCount; ++v) {
        vtrz::DVector2 vert = vtrz::cast(leaf.poly[v]);
        // NOTE: no clamp
        vtrz::DVector3 perspCoord = vtrz::barycentricCoordNoClamp(vert, tri2d);
        perspCoord.x *= pipeline.wInvs[offset + 0];
        perspCoord.y *= pipeline.wInvs[offset + 1];
        perspCoord.z *= pipeline.wInvs[offset + 2];
        perspCoord /= (perspCoord.x + perspCoord.y + perspCoord.z);
        vtrz::DVector3 wp = wp0 * perspCoord.x + wp1 * perspCoord.y + wp2 * perspCoord.z;
        for (int d = payload.depth - 1; d >= 0; --d) {
            wp = rayPlaneIsectAD(
                payload.backtrack[d].beamOrigin,
                wp - payload.backtrack[d].beamOrigin,
                payload.backtrack[d].plane);
        }
        vtrz::DVector4 h = initTrans * vtrz::DVector4(wp, 1.0f);
        h.x /= h.w;
        h.y /= h.w;
        constexpr Float snapEps = 1e-3f;
        if (h.x.value <= -1.0f + snapEps || h.x.value >= 1.0f - snapEps) {
            h.x.zeroGrad();
        }
        if (h.y.value <= -1.0f + snapEps || h.y.value >= 1.0f - snapEps) {
            h.y.zeroGrad();
        }

        if (v == 0) {
            v0 = vlast = vtrz::DVector2(h.x, h.y);
        } else {
            coverage += vtrz::cross(vlast, vtrz::DVector2(h.x, h.y));
            vlast = vtrz::DVector2(h.x, h.y);
        }
    }
    coverage += vtrz::cross(vlast, v0);

    return vtrz::abs(coverage) * 0.125f;
}

void KaleidoscopeAD::renderBlock(
    const Scene *scene, const Sensor *sensor, Sampler *sampler,
    ImageBlock *block, const bool &stop, const std::vector<TPoint2<uint8_t>> &points) const
{
    block->clear();

    SAssert(sensor->getClass()->derivesFrom(MTS_CLASS(PerspectiveCamera)));
    ViewInfo view(*sensor);

    for (auto pt : points) {
        if (stop) break;

        Point2i pixel = Point2i(pt) + Vector2i(block->getOffset());
        if (pixel.x != 199 || pixel.y != 296) continue;

        vtrz::DVector3 L(0.0);
        std::queue<TracePayloadAD> queue;

        FrustumBVHIsectResult isects;
        PrimaryPipelineAD pipeline;
        vtrz::VectorizerGrad vect;

        TracePayloadAD init;
        Beam initBeam = view.createPrimaryBeam(pixel);
        vtrz::Matrix4x4 initView = castMatrix4x4(initBeam.viewTrans.getMatrix());
        vtrz::Matrix4x4 initProj = castMatrix4x4(initBeam.projTrans.getMatrix());
        init.beam.frustum = initBeam.frustum;
        init.beam.viewTrans = vtrz::DMatrix4x4(initView);
        init.beam.projTrans = initProj;
        vtrz::Matrix4x4 initTrans = initProj * initView;

        queue.push(init);
        while (!queue.empty()) {
            TracePayloadAD payload = queue.front();
            queue.pop();

            isects.clear();
            bvh.frustumIntersect(payload.beam.frustum, isects, FrustumAABBOption::eCoarse);
            pipeline.clear();

            pipeline.vpTrans = payload.beam.projTrans * payload.beam.viewTrans;
            if (payload.depth % 2) {
                // Flip handedness.
                vtrz::Matrix4x4 flipViewMtx;
                flipViewMtx.arr[1][1] = -1.0;
                pipeline.vpTrans = flipViewMtx * pipeline.vpTrans;
            }
            pipeline.apex = payload.beam.frustum.apex;

            pipeline.run(*scene, bvh, sceneAD, isects);
            if (payload.depth == 0) {
                vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
                vect.build(
                    reinterpret_cast<vtrz::DVector3 *>(pipeline.positions.data()), (uint32_t)pipeline.positions.size(),
                    nullptr, 0);
            } else {
                std::vector<vtrz::DVector2> rootShape(payload.vertCount);
                for (uint32_t i = 0; i < payload.vertCount; ++i) {
                    vtrz::DVector4 h = pipeline.vpTrans * vtrz::DVector4(payload.patch[i], 1.0f);
                    h.x /= h.w;
                    h.y /= h.w;

                    uint32_t idx = payload.vertCount - 1 - i;
                    rootShape[idx] = vtrz::DVector2(h.x, h.y);
                    // rootShape[idx].x = math::clamp(rootShape[idx].x, -1.0f, 1.0f);
                    // rootShape[idx].y = math::clamp(rootShape[idx].y, -1.0f, 1.0f);
                }
                // TODO.
                vect.reset(rootShape.data(), (uint32_t)rootShape.size(), vtrz::DepthFunc::eGreater);
                vect.build(
                    reinterpret_cast<vtrz::DVector3 *>(pipeline.positions.data()), (uint32_t)pipeline.positions.size(),
                    nullptr, 0, false);
            }
            // TODO.
            std::array<uint32_t, 7> hackId1 = {
                0, 1, 2, 3, 0, 1, 3
            };
            std::vector<uint32_t> hackId2(pipeline.primIds.size());
            for (uint32_t i = 0; i < (uint32_t)hackId2.size(); ++i) {
                if (pipeline.meshIds[i] == 0) {
                    hackId2[i] = hackId1[pipeline.primIds[i]];
                } else {
                    hackId2[i] = pipeline.primIds[i];
                }
            }
            vect.simplify(pipeline.meshIds.data(), hackId2.data());

            // vect.simplify(pipeline.meshIds.data(), pipeline.primIds.data());


            uint32_t leafCount = vect.leafCount();
            for (uint32_t leafIdx = 0; leafIdx < leafCount; ++leafIdx) {
                vtrz::LeafRegionGrad leaf = vect.getLeaf(leafIdx);
                const TriMesh *mesh = scene->getMeshes()[pipeline.meshIds[leaf.triId]];
                const BSDF *bsdf = mesh->getBSDF();
                if (bsdf->hasComponent(BSDF::EBSDFType::EDeltaReflection)) {
                    if (payload.depth < maxDepth) {
                        queue.push(createPayload(leaf, pipeline, payload, view));
                    }
                } else {
                    uint32_t primIndex = pipeline.primIds[leaf.triId];
                    size_t h = std::hash<uint32_t>()(primIndex);
                    vtrz::DVector3 color(Float(h % 256) / 255.0f, Float((h >> 8) % 256) / 255.0f, Float((h >> 16) % 256) / 255.0f);
                    L += color * propagateCoverage(leaf, pipeline, payload, initTrans);
                }
            }
        }

        writePixel(pixel, L, *block);
    }
}

void KaleidoscopeAD::writePixel(const Point2i &pixel, const vtrz::DVector3 &L, ImageBlock &block) const
{
    // Do I actually need pixel filtering?
    Point2 pixelCenter(Point2(pixel) + Vector2(0.5, 0.5));
    Spectrum value;
    value.fromLinearRGB(L.x.value, L.y.value, L.z.value);
    block.put(pixelCenter, value, 1.0f);
    for (uint32_t i = 0; i < totalGradDim; ++i) {
        float *data = gradImages[i]->getFloat32Data();
        uint32_t offset = (pixel.y * gradImages[i]->getWidth() + pixel.x) * gradImages[i]->getChannelCount();
        SAssert(!std::isnan(L.x.grad[i]) && !std::isinf(L.x.grad[i]));
        SAssert(!std::isnan(L.y.grad[i]) && !std::isinf(L.y.grad[i]));
        SAssert(!std::isnan(L.z.grad[i]) && !std::isinf(L.z.grad[i]));

        data[offset] = L.x.grad[i];
        data[offset + 1] = L.y.grad[i];
        data[offset + 2] = L.z.grad[i];
    }
}

void KaleidoscopeAD::postprocess(
    const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID)
{
    const auto &dest = scene->getDestinationFile();
    auto stem = dest.stem();
    for (uint32_t i = 0; i < totalGradDim; ++i) {
        std::string postfix = "-grad" + std::to_string(i) + ".exr";
        fs::path path = dest.parent_path() / dest.stem();
        path += fs::path(std::move(postfix));
        gradImages[i]->write(path);
    }
}

//////////////////////////////////////////////////////////////////////////
MTS_IMPLEMENT_CLASS_S(KaleidoscopeAD, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(KaleidoscopeAD, "Vectorizer kaleidoscope AD");
MTS_NAMESPACE_END