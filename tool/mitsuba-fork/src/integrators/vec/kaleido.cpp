#include "util.h"
#include "pipeline.h"
#include "bvh.h"
#include <array>
#include <queue>

MTS_NAMESPACE_BEGIN

class Kaleidoscope final : public SamplingIntegrator
{
public:
    MTS_DECLARE_CLASS();

    Kaleidoscope(const Properties &props);

    Kaleidoscope(Stream *stream, InstanceManager *manager);

    bool preprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID) final;

    void renderBlock(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points) const final;

    Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const final { return Spectrum(0.0); }

    struct BacktrackRecord
    {
        Point beamOrigin;
        Vector4 plane;
    };

    struct TracePayload
    {
        Beam beam;
        std::array<Point3, 16> patch;
        std::array<BacktrackRecord, 16> backtrack;
        uint16_t vertCount = 0;
        uint16_t depth = 0;
    };

    TracePayload createPayload(
        const vtrz::LeafRegion &leaf, const PrimaryPipeline &pipeline, const Scene &scene, const TracePayload &old, const ViewInfo &viewInfo) const;

    Float propagateCoverage(const vtrz::LeafRegion &leaf, const PrimaryPipeline &pipeline,
        const TracePayload &payload, const Transform &initTrans) const;

private:
    BVH bvh;
    uint32_t maxDepth;
};

Kaleidoscope::Kaleidoscope(const Properties &props) :
    SamplingIntegrator(props)
{
    maxDepth = props.getInteger("maxDepth", 5);
}

Kaleidoscope::Kaleidoscope(Stream *stream, InstanceManager *manager) :
    SamplingIntegrator(stream, manager)
{
    maxDepth = stream->readInt();
}

bool Kaleidoscope::preprocess(
    const Scene *scene, RenderQueue *queue, const RenderJob *job,
    int sceneResID, int sensorResID, int samplerResID)
{
    bvh.build(*scene);
    return true;
}

// Note that reflection matrix reverses handedness.
static Matrix4x4 makeReflect(const Normal &n, const Point &p)
{
    Float d = dot(n, Vector(p));
    Matrix4x4 R(
        1.0f-2.0f*n.x*n.x,  -2.0f*n.y*n.x,      -2.0f*n.z*n.x,      2.0f*d*n.x,
        -2.0f*n.x*n.y,      1.0f-2.0f*n.y*n.y,  -2.0f*n.z*n.y,      2.0f*d*n.y,
        -2.0f*n.x*n.z,      -2.0f*n.y*n.z,      1.0f-2.0f*n.z*n.z,  2.0f*d*n.z,
        0.0f,               0.0f,               0.0f,               1.0f
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

static Beam reflectBeam(
    const Beam &old, Float nearClip, Float vFov, Float aspect,
    const Normal &n, const Point &p, uint32_t depth)
{
    Vector4 plane(n.x, n.y, n.z, -dot(n, Vector(p)));

    std::array<Point, 4> footPrint;
    Transform oldInvVp = (old.projTrans * old.viewTrans).inverse();
    for (uint32_t i = 0; i < 4; ++i) {
        Vector rd = Vector(oldInvVp.transformAffine(Point(i / 2 ? 1.0f : -1.0f, i % 2 ? 1.0f : -1.0f, 0.0f)));
        footPrint[i] = rayPlaneIsect(old.frustum.apex, rd, plane);
    }

    Beam beam;

    Transform R = Transform(makeReflect(n, p));
    beam.viewTrans = old.viewTrans * R.inverse();

    Point origin(
        beam.viewTrans.getInverseMatrix().m[0][3],
        beam.viewTrans.getInverseMatrix().m[1][3],
        beam.viewTrans.getInverseMatrix().m[2][3]
    );

    beam.projTrans = Transform(revInfPerspective(nearClip, vFov, aspect, AABB2(Point2(-1.0f), Point2(1.0f))));

    Transform vp = beam.projTrans * beam.viewTrans;
    AABB2 newNdcBound;
    for (uint32_t i = 0; i < 4; ++i) {
        Point h = vp(footPrint[i]);
        SAssert(h.x >= -1.0f - 1e4f && h.x <= 1.0f + 1e4f);
        SAssert(h.y >= -1.0f - 1e4f && h.y <= 1.0f + 1e4f);
        SAssert(h.z >= 0.0f - 1e4f && h.z <= 1.0f + 1e4f);
        newNdcBound.expandBy(Point2(h.x, h.y));
    }

    beam.projTrans = Transform(revInfPerspective(nearClip, vFov, aspect, newNdcBound));
    vp = beam.projTrans * beam.viewTrans;
    Transform invVp = vp.inverse();
    std::array<Vector, 4> frustumCornerDirs = {
        Vector(-1.0f, -1.0f, 0.0f),
        Vector(1.0f, -1.0f, 0.0f),
        Vector(1.0f, 1.0f, 0.0f),
        Vector(-1.0f, 1.0f, 0.0f)
    };
    for (uint32_t i = 0; i < 4; ++i) {
        frustumCornerDirs[i] = Vector(invVp.transformAffine(Point(frustumCornerDirs[i])));
    }
    Vector centerDir = Vector(invVp.transformAffine(Point(0.0f)));

    if (depth % 2) {
        // Flip handedness.
        std::swap(frustumCornerDirs[1], frustumCornerDirs[3]);
    }

    beam.frustum = Frustum(origin, centerDir, frustumCornerDirs.data(), kSecondaryNearClip, FrustumAABBOption::eCoarse);

    return beam;
}

Kaleidoscope::TracePayload Kaleidoscope::createPayload(
    const vtrz::LeafRegion &leaf, const PrimaryPipeline &pipeline, const Scene &scene, const TracePayload &old, const ViewInfo &viewInfo) const
{
    TracePayload payload;

    payload.depth = old.depth + 1;
    SAssert(payload.depth <= (uint32_t)payload.backtrack.size());

    const TriMesh *mesh = scene.getMeshes()[pipeline.meshIds[leaf.triId]];
    Triangle tri = mesh->getTriangles()[pipeline.primIds[leaf.triId]];
    const Point &tri0 = mesh->getVertexPositions()[tri.idx[0]];
    const Point &tri1 = mesh->getVertexPositions()[tri.idx[1]];
    const Point &tri2 = mesh->getVertexPositions()[tri.idx[2]];
    Vector n = normalize(cross(tri1 - tri0, tri2 - tri1));

    // Vector n = normalize(cross(wp1 - wp0, wp2 - wp1));
    std::copy(old.backtrack.begin(), old.backtrack.begin() + old.depth, payload.backtrack.begin());
    payload.backtrack[payload.depth - 1].beamOrigin = Point(pipeline.apex - 2.0f * dot(pipeline.apex - tri0, n) * n);
    payload.backtrack[payload.depth - 1].plane = Vector4(n.x, n.y, n.z, -dot(n, Vector(tri0)));

    payload.vertCount = leaf.vertCount;
    SAssert(payload.vertCount <= (uint32_t)payload.patch.size());

    uint32_t offset = leaf.triId * 3;
    const Point &wp0 = pipeline.worldPositions[offset + 0];
    const Point &wp1 = pipeline.worldPositions[offset + 1];
    const Point &wp2 = pipeline.worldPositions[offset + 2];

    const Point &p0 = pipeline.positions[offset + 0];
    const Point &p1 = pipeline.positions[offset + 1];
    const Point &p2 = pipeline.positions[offset + 2];

    vtrz::Tri2 tri2d = {
        vtrz::Vector2(p0.x, p0.y),
        vtrz::Vector2(p1.x, p1.y),
        vtrz::Vector2(p2.x, p2.y),
    };
    for (uint32_t v = 0; v < leaf.vertCount; ++v) {
        vtrz::Vector2 vert = vtrz::cast(leaf.poly[v]);
        // NOTE: no clamp
        vtrz::Vector3 perspCoord = vtrz::barycentricCoordNoClamp(vert, tri2d);
        perspCoord.x *= pipeline.wInvs[offset + 0];
        perspCoord.y *= pipeline.wInvs[offset + 1];
        perspCoord.z *= pipeline.wInvs[offset + 2];
        perspCoord /= (perspCoord.x + perspCoord.y + perspCoord.z);
        payload.patch[v] = wp0 * perspCoord.x + wp1 * perspCoord.y + wp2 * perspCoord.z;
    }

    payload.beam = reflectBeam(
        old.beam, viewInfo.nearClip, viewInfo.vFov, viewInfo.aspect, n, payload.patch[0], payload.depth);

    return payload;
}

Float Kaleidoscope::propagateCoverage(const vtrz::LeafRegion &leaf, const PrimaryPipeline &pipeline,
    const TracePayload &payload, const Transform &initTrans) const
{
    uint32_t offset = leaf.triId * 3;
    const Point &wp0 = pipeline.worldPositions[offset + 0];
    const Point &wp1 = pipeline.worldPositions[offset + 1];
    const Point &wp2 = pipeline.worldPositions[offset + 2];

    Point p0(pipeline.positions[offset + 0].x, pipeline.positions[offset + 0].y, pipeline.positions[offset + 0].z);
    Point p1(pipeline.positions[offset + 1].x, pipeline.positions[offset + 1].y, pipeline.positions[offset + 1].z);
    Point p2(pipeline.positions[offset + 2].x, pipeline.positions[offset + 2].y, pipeline.positions[offset + 2].z);
    vtrz::Tri2 tri2d = {
        vtrz::Vector2(p0.x, p0.y),
        vtrz::Vector2(p1.x, p1.y),
        vtrz::Vector2(p2.x, p2.y),
    };

    Float coverage = 0.0f;
    Vector2 v0, vlast;
    for (uint32_t v = 0; v < leaf.vertCount; ++v) {
        vtrz::Vector2 vert = vtrz::cast(leaf.poly[v]);
        // NOTE: no clamp
        vtrz::Vector3 perspCoord = vtrz::barycentricCoordNoClamp(vert, tri2d);
        perspCoord.x *= pipeline.wInvs[offset + 0];
        perspCoord.y *= pipeline.wInvs[offset + 1];
        perspCoord.z *= pipeline.wInvs[offset + 2];
        perspCoord /= (perspCoord.x + perspCoord.y + perspCoord.z);
        Point wp = wp0 * perspCoord.x + wp1 * perspCoord.y + wp2 * perspCoord.z;

        for (int d = payload.depth - 1; d >= 0; --d) {
            wp = rayPlaneIsect(
                payload.backtrack[d].beamOrigin,
                wp - payload.backtrack[d].beamOrigin,
                payload.backtrack[d].plane);
        }
        Point h = initTrans(wp);
        if (v == 0) {
            v0 = vlast = Vector2(h.x, h.y);
        } else {
            coverage += det(vlast, Vector2(h.x, h.y));
            vlast = Vector2(h.x, h.y);
        }
    }
    coverage += det(vlast, v0);

    return std::abs(coverage) * 0.125f;
}

void Kaleidoscope::renderBlock(
    const Scene *scene, const Sensor *sensor, Sampler *sampler,
    ImageBlock *block, const bool &stop, const std::vector<TPoint2<uint8_t>> &points) const
{
    block->clear();

    SAssert(sensor->getClass()->derivesFrom(MTS_CLASS(PerspectiveCamera)));
    ViewInfo view(*sensor);

    for (auto pt : points) {
        if (stop) break;

        Point2i pixel = Point2i(pt) + Vector2i(block->getOffset());
        //if (pixel.x != 344 || pixel.y != 287) continue;

        Spectrum L(0.0);
        std::queue<TracePayload> queue;

        FrustumBVHIsectResult isects;
        PrimaryPipeline pipeline;
        vtrz::Vectorizer vect;

        TracePayload init;
        init.beam = view.createPrimaryBeam(pixel);
        Transform initTrans = init.beam.projTrans * init.beam.viewTrans;

        queue.push(init);
        while (!queue.empty()) {
            TracePayload payload = queue.front();
            queue.pop();

            isects.clear();
            bvh.frustumIntersect(payload.beam.frustum, isects, FrustumAABBOption::eCoarse);
            pipeline.clear();

            pipeline.vpTrans = payload.beam.projTrans * payload.beam.viewTrans;
            if (payload.depth % 2) {
                // Flip handedness.
                Matrix4x4 flipViewMtx;
                flipViewMtx.setIdentity();
                flipViewMtx.m[1][1] = -1.0;
                pipeline.vpTrans = Transform(flipViewMtx) * pipeline.vpTrans;
            }
            pipeline.apex = payload.beam.frustum.apex;

            pipeline.run(*scene, bvh, isects);
            if (payload.depth == 0) {
                vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
                vect.build(
                    reinterpret_cast<vtrz::Vector3 *>(pipeline.positions.data()), (uint32_t)pipeline.positions.size(),
                    nullptr, 0);
            } else {
                std::vector<Vector2> rootShape(payload.vertCount);
                for (uint32_t i = 0; i < payload.vertCount; ++i) {
                    Point p = pipeline.vpTrans(payload.patch[i]);

                    uint32_t idx = payload.vertCount - 1 - i;
                    rootShape[idx] = Vector2(p.x, p.y);
                    rootShape[idx].x = math::clamp(rootShape[idx].x, -1.0f, 1.0f);
                    rootShape[idx].y = math::clamp(rootShape[idx].y, -1.0f, 1.0f);
                }
                vect.reset((const vtrz::Vector2 *)rootShape.data(), (uint32_t)rootShape.size(), vtrz::DepthFunc::eGreater);
                vect.build(
                    reinterpret_cast<vtrz::Vector3 *>(pipeline.positions.data()), (uint32_t)pipeline.positions.size(),
                    nullptr, 0, false);
            }

            vect.simplify(pipeline.meshIds.data(), pipeline.primIds.data());

            uint32_t leafCount = vect.leafCount();
            for (uint32_t leafIdx = 0; leafIdx < leafCount; ++leafIdx) {
                vtrz::LeafRegion leaf = vect.getLeaf(leafIdx);
                const TriMesh *mesh = scene->getMeshes()[pipeline.meshIds[leaf.triId]];
                const BSDF *bsdf = mesh->getBSDF();
                if (bsdf->hasComponent(BSDF::EBSDFType::EDeltaReflection)) {
                    if (payload.depth < maxDepth) {
                        queue.push(createPayload(leaf, pipeline, *scene, payload, view));
                    }
                } else {
                    uint32_t primIndex = pipeline.primIds[leaf.triId];
                    size_t h = std::hash<uint32_t>()(primIndex);
                    Spectrum color;
                    color.fromLinearRGB(Float(h % 256) / 255.0f, Float((h >> 8) % 256) / 255.0f, Float((h >> 16) % 256) / 255.0f);
                    L += color * propagateCoverage(leaf, pipeline, payload, initTrans);
                }
            }
        }

        // Do I actually need pixel filtering?
        Point2 pixelCenter(Point2(pixel) + Vector2(0.5, 0.5));
        // No alpha.
        block->put(pixelCenter, L, 1.0);
    }
}

//////////////////////////////////////////////////////////////////////////
MTS_IMPLEMENT_CLASS_S(Kaleidoscope, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(Kaleidoscope, "Vectorizer kaleidoscope");
MTS_NAMESPACE_END