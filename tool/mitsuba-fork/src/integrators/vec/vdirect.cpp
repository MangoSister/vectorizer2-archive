#include "util.h"
#include "pipeline.h"
#include "bvh.h"
#include <unordered_set>
#include <fstream>
#include <memory>
#include <boost/filesystem/path.hpp>

MTS_NAMESPACE_BEGIN

class VDirectIntegrator final : public SamplingIntegrator
{
public:
    MTS_DECLARE_CLASS();

	VDirectIntegrator(const Properties &props);

	VDirectIntegrator(Stream *stream, InstanceManager *manager);

    bool preprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID) final;

    void postprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) final;

    void computePointShadows(const Scene &scene);

    void renderBlock(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points) const final;

    Spectrum evalDirectLighting(
        vtrz::Vectorizer &vect, const PrimaryPipeline &primary,
        SecondaryPipeline &secondary, FrustumBVHIsectResult &result,
        const Point &sensorPos, const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel) const;

	Spectrum evalDirectLightingArea(
        vtrz::Vectorizer &vect, const PrimaryPipeline &primary,
        SecondaryPipeline &secondary, FrustumBVHIsectResult &result,
        const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const;

    Spectrum evalDirectLightingAreaMulti(
        vtrz::Vectorizer &vect, const PrimaryPipeline &primary,
        SecondaryPipeline &secondary, FrustumBVHIsectResult &result,
        const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const;

    Spectrum evalDirectLightingAreaSample(
        vtrz::Vectorizer &vect, SecondaryPipeline &secondary, FrustumBVHIsectResult &result,
        const Intersection &its, const Point &sensorPos, const Scene &scene, const Emitter &emitter) const;

    Spectrum evalDirectLightingPoint(
        const vtrz::Vectorizer &vect, const PrimaryPipeline &primary,
        const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, uint32_t pointLightIndex) const;

    void renderBlockV2(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points) const;

    Spectrum evalDirectLightingV2(
        const PrimaryPipeline &primary, IncrementalContext &incCtx,
        const Point &sensorPos, const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel) const;

    Spectrum evalDirectLightingMultiV2(
        const PrimaryPipeline &primary, IncrementalContext &incCtx,
        const Point &sensorPos, const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel) const;

    Spectrum evalDirectLightingSampleV2(
        IncrementalContext &incCtx,
        const Intersection &its, const Point &sensorPos, const Scene &scene, const Emitter &emitter) const;

    Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const final;

    Spectrum E(const Scene *scene, const Intersection &its,
        const Medium *medium, Sampler *sampler, int nSamples,
        bool includeIndirect) const final;

private:
    bool onlyCoverage = false;
    bool multiSample = false;

    bool outputRaw = false;
    std::unique_ptr<float[]> rawPixels;

    BVH bvh;
    std::vector<const Emitter *> areaLights;
    std::vector<const Emitter *> pointLights;
    struct PointShadow
    {
        Transform transforms[6];
        std::vector<uint16_t> meshIds[6];
        std::vector<uint32_t> primIds[6];
        std::vector<Point> worldPositions[6];
        std::vector<float> wInvs[6];
        vtrz::Vectorizer vects[6];
    };
    std::vector<PointShadow> pointShadows;
};

//////////////////////////////////////////////////////////////////////////
VDirectIntegrator::VDirectIntegrator(const Properties &props) :
    SamplingIntegrator(props)
{
    onlyCoverage = props.getBoolean("onlycoverage", false);
    multiSample = props.getBoolean("multisample", false);
    outputRaw = props.getBoolean("outputraw", false);
}

VDirectIntegrator::VDirectIntegrator(Stream *stream, InstanceManager *manager)
    : SamplingIntegrator(stream, manager)
{ }

void VDirectIntegrator::renderBlock(const Scene *scene, const Sensor *sensor,
	Sampler *sampler, ImageBlock *block, const bool &stop,
	const std::vector< TPoint2<uint8_t> > &points) const
{
    block->clear();

    SAssert(sensor->getClass()->derivesFrom(MTS_CLASS(PerspectiveCamera)));
    ViewInfo view(*sensor);

	FrustumBVHIsectResult isects;
    PrimaryPipeline primary;
    SecondaryPipeline secondary;
    vtrz::Vectorizer vect;

    for (auto pt : points) {
        if (stop) {
            break;
        }

        Point2i pixel = Point2i(pt) + Vector2i(block->getOffset());
        //////////////////////////////////////////////////////////////////////////
        // Debug
        // if (pixel.x != 670 || pixel.y != 442) continue;
        //////////////////////////////////////////////////////////////////////////
        Beam beam = view.createPrimaryBeam(pixel);
        isects.clear();
        bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eExact);
        primary.clear();
        primary.vpTrans = beam.projTrans * beam.viewTrans;
        primary.apex = beam.frustum.apex;
        primary.run(*scene, bvh, isects);

        //////////////////////////////////////////////////////////////////////////
        // Debug
        //dumpVectorizerInput(scene, pixel, AABB2(Point2(-1.0f), Point2(1.0f)), primary.positions, nullptr);
        //////////////////////////////////////////////////////////////////////////
        vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
        vect.build(
            reinterpret_cast<vtrz::Vector3 *>(primary.positions.data()), (uint32_t)primary.positions.size(),
            nullptr, 0);

        Spectrum L;
        if (onlyCoverage) {
            L = Spectrum(Float(vect.coverageQuery()));
        } else {
            L = evalDirectLighting(vect, primary, secondary, isects, view.sensorPos, *scene, view.persp, pixel);
        }

        // Do I actually need pixel filtering?
        Point2 pixelCenter(Point2(pixel) + Vector2(0.5, 0.5));
        // No alpha.
        block->put(pixelCenter, L, 1.0);
        if (outputRaw) {
            Vector2i res = scene->getFilm()->getCropSize();
            uint32_t offset = (pixel.y * res.x + pixel.x) * 3;
            L.toLinearRGB(rawPixels[offset], rawPixels[offset + 1], rawPixels[offset + 2]);
        }
    }
}

Spectrum VDirectIntegrator::evalDirectLighting(
    vtrz::Vectorizer &vect, const PrimaryPipeline &primary, SecondaryPipeline &secondary, FrustumBVHIsectResult &isects,
    const Point &sensorPos, const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel) const
{
    RayDifferential rayDiff;
    persp.sampleRayDifferential(rayDiff, Point2(pixel) + Point2(0.5, 0.5), Point2(0.0), Float(0.0));
    // Do not consider rayDiff scaling for now.

    Spectrum L(0.0f);
    for (uint32_t pointLightIndex = 0; pointLightIndex < (uint32_t)pointLights.size(); ++pointLightIndex) {
        L += evalDirectLightingPoint(vect, primary, sensorPos, scene, rayDiff, pointLightIndex);
    }

    //if (!multiSample) {
    //    L += evalDirectLightingArea(vect, primary, secondary, isects, sensorPos, scene, rayDiff, pixel);
    //} else {
    //    L += evalDirectLightingAreaMulti(vect, primary, secondary, isects, sensorPos, scene, rayDiff, pixel);
    //}
    return L;
}

Spectrum VDirectIntegrator::evalDirectLightingArea(
    vtrz::Vectorizer &vect, const PrimaryPipeline &primary, SecondaryPipeline &secondary, FrustumBVHIsectResult &isects,
    const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const
{
    Spectrum L(0.0f);
    vtrz::PointSample sample = vect.pointSample();

    if (sample.weight > 0.0f) {
        Intersection its = primary.buildItsFromSample(sample, sensorPos, scene);
        its.getBSDF(rayDiff); // Calculate its partials here.
        uint32_t emitterCount = (uint32_t)scene.getEmitters().size();
        for (const Emitter *areaLight : areaLights) {
            L += evalDirectLightingAreaSample(vect, secondary, isects, its, sensorPos, scene, *areaLight);
        }
        L *= sample.weight;
    }

    L += (1.0f - sample.weight) * scene.evalEnvironment(rayDiff);

    return L;
}

Spectrum VDirectIntegrator::evalDirectLightingAreaMulti(
    vtrz::Vectorizer &vect, const PrimaryPipeline &primary, SecondaryPipeline &secondary, FrustumBVHIsectResult &isects,
    const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const
{
    Spectrum L(0.0f);
    float accWeight = 0.0f;
    vtrz::PointSampleArray samples = vect.pointSampleMulti(primary.matIds.data(),
        reinterpret_cast<const vtrz::Vector3 *>(primary.normals.data()));

    Intersection *its = (Intersection *)alloca(samples.size * sizeof(Intersection));
    for (uint8_t i = 0; i < samples.size; ++i) {
        its[i] = primary.buildItsFromSample(samples.entries[i], sensorPos, scene);
        its[i].getBSDF(rayDiff); // Calculate its partials here.
        accWeight += samples.entries[i].weight;
    }

    // Slightly better cache performance if loop this way?
    for (const Emitter *areaLight : areaLights) {
        for (uint8_t i = 0; i < samples.size; ++i) {
            L += samples.entries[i].weight * evalDirectLightingAreaSample(vect, secondary, isects, its[i], sensorPos, scene, *areaLight);
        }
    }

    accWeight = math::clamp(accWeight, 0.0f, 1.0f);
    L += (1.0f - accWeight) * scene.evalEnvironment(rayDiff);

    return L;
}

Spectrum VDirectIntegrator::evalDirectLightingAreaSample(
    vtrz::Vectorizer &vect, SecondaryPipeline &secondary, FrustumBVHIsectResult &isects,
    const Intersection &its, const Point &sensorPos,
    const Scene &scene, const Emitter &emitter) const
{
    const Shape *emitterShape = emitter.getShape();
    // Only area lights so far.
    Assert(emitterShape->getClass()->derivesFrom(MTS_CLASS(TriMesh)));
    const TriMesh *emitterMesh = static_cast<const TriMesh *>(emitterShape);
    // Hack. Assume constant radiance.
    Spectrum emitterRadiance = emitter.evalPosition(PositionSamplingRecord()) * INV_PI;
    if (its.shape == emitterMesh) {
        return emitterRadiance;
    }

    Beam beam;
    if (!createSecondaryBeam(its, emitter, beam)) {
        return Spectrum(0.0f);
    }

    isects.clear();
    bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eCoarse);
    secondary.clear();
    secondary.vpTrans = beam.projTrans * beam.viewTrans;
    secondary.apex = beam.frustum.apex;
    secondary.run(scene, bvh, isects, static_cast<const TriMesh *>(emitter.getShape()));

    // Actually no emitter geometry is inside the frustum (clipped by horizon).
    if (secondary.queryTriIndices.empty()) {
        return Spectrum(0.0f);
    }

    //////////////////////////////////////////////////////////////////////////
    // Debug
    //dumpVectorizerInput(&scene, Point2i(331, 315), AABB2(Point2(-1.0f), Point2(1.0f)), pipeline.positions, pipeline.indices, &pipeline.queryTriIndices);
    //////////////////////////////////////////////////////////////////////////
    vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
    vect.buildOcclusion(
        reinterpret_cast<vtrz::Vector3 *>(secondary.positions.data()), (uint32_t)secondary.positions.size(),
        nullptr, 0,
        secondary.queryTriIndices.data(), (uint32_t)secondary.queryTriIndices.size());

    Spectrum bsdfInt(0.0);
    if (vect.leafCount() > 0) {
        Vector N = (Vector)its.shFrame.n;
        Vector V = normalize(sensorPos - its.p);
        Float NdotV = dot(N, V);
        Transform invProjTrans = beam.projTrans.inverse();
        Transform invViewTrans = beam.viewTrans.inverse();
        BSDFPolyIntegralContext context(vect,
            invProjTrans,
            invViewTrans,
            its, NdotV);
        const BSDF *bsdf = its.getBSDF();
        bsdfInt = bsdf->evalPolyIntegral(context);
    }

    return emitterRadiance * bsdfInt;
}

Spectrum VDirectIntegrator::evalDirectLightingPoint(
    const vtrz::Vectorizer &vect, const PrimaryPipeline &primary,
    const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, uint32_t pointLightIndex) const
{
    const Emitter &light = *pointLights[pointLightIndex];
    PositionSamplingRecord pRec;
    Spectrum intensity = light.samplePosition(pRec, Point2()) * INV_FOURPI;

    Spectrum Lsum(0.0);
    float totalCoverage = vect.coverageQuery();

    vtrz::Matrix4x4 queryTransform = castMatrix4x4(primary.vpTrans.getMatrix());

    uint32_t leafCount = vect.leafCount();
    for (uint32_t leafIdx = 0; leafIdx < leafCount; ++leafIdx) {
        vtrz::LeafRegion region = vect.getLeaf(leafIdx);
        IntersectionPatch patch;
        if (!primary.buildItsPatch(region, sensorPos, scene, patch)) {
            continue;
        }
        const BSDF *bsdf = patch.avgIts.getBSDF(rayDiff);
        Spectrum Lpatch(0.0);
        for (uint32_t i = 0; i < (uint32_t)patch.positions.size(); ++i) {
            patch.avgIts.p = patch.positions[i];
            if (patch.normals.empty()) {
                patch.avgIts.shFrame = Frame(patch.avgIts.geoFrame.n);
            } else {
                patch.avgIts.shFrame = Frame(patch.normals[i]);
            }

            // ..weird convention?
            Vector wo = pRec.p - patch.avgIts.p;
            Float invDist2 = 1.0f / wo.lengthSquared();
            wo *= std::sqrt(invDist2);
            wo = patch.avgIts.shFrame.toLocal(wo);

            patch.avgIts.wi = normalize(sensorPos - patch.avgIts.p);
            patch.avgIts.wi = patch.avgIts.shFrame.toLocal(patch.avgIts.wi);
            BSDFSamplingRecord bRec(patch.avgIts, wo);
            Lpatch += bsdf->eval(bRec) * intensity * invDist2;
        }
        SAssert(Lpatch.isValid());
        if (Lpatch.isZero()) {
            continue;
        }
        Lpatch /= (Float)patch.positions.size();

        Float visibleCoverage(0.0);
        // Determine which faces and clipping again...
        SecondaryPipeline pipeline;
        for (uint32_t face = 0; face < 6; ++face) {
            pipeline.clear();
            pipeline.apex = pRec.p;
            pipeline.vpTrans = pointShadows[pointLightIndex].transforms[face];
            pipeline.add(patch);
            const vtrz::Vectorizer &shadow = pointShadows[pointLightIndex].vects[face];
            const uint16_t *meshIds = pointShadows[pointLightIndex].meshIds[face].data();
            const uint32_t *primIds = pointShadows[pointLightIndex].primIds[face].data();
            const vtrz::Vector3 *worldPositions = (const vtrz::Vector3 *)pointShadows[pointLightIndex].worldPositions[face].data();
            const float *wInvs = pointShadows[pointLightIndex].wInvs[face].data();
            for (uint32_t i = 0; i < (uint32_t)pipeline.positions.size() / 3; ++i) {
                vtrz::Tri3 tri = {
                    toVec3(pipeline.positions[3 * i]),
                    toVec3(pipeline.positions[3 * i + 1]),
                    toVec3(pipeline.positions[3 * i + 2])
                };
                visibleCoverage += shadow.triangleCoverageQuery(
                    tri, patch.meshId, patch.avgIts.primIndex,
                    meshIds, primIds, worldPositions, wInvs, queryTransform);
            }
        }
        visibleCoverage = math::clamp(visibleCoverage, 0.0f, patch.coverage);
        Lpatch *= visibleCoverage;
        Lsum += Lpatch;
    }
    Lsum /= totalCoverage;
    return Lsum;
}

void VDirectIntegrator::renderBlockV2(
    const Scene *scene, const Sensor *sensor, Sampler *sampler, ImageBlock *block,
    const bool &stop, const std::vector< TPoint2<uint8_t> > &points) const
{
    block->clear();

    SAssert(sensor->getClass()->derivesFrom(MTS_CLASS(PerspectiveCamera)));
    ViewInfo view(*sensor);

    FrustumBVHIsectResult isects;
    PrimaryPipeline primary;
    vtrz::Vectorizer vect;
    SecondaryPipelineInc secondary;
    IncrementalContext incCtx;
    incCtx.scene = scene;
    incCtx.pipeline = &secondary;
    incCtx.vect = &vect;

    for (auto pt : points) {
        if (stop) {
            break;
        }

        Point2i pixel = Point2i(pt) + Vector2i(block->getOffset());
        //////////////////////////////////////////////////////////////////////////
        // Debug
        // if (pixel.x != 30 || pixel.y != 80) continue;
        //////////////////////////////////////////////////////////////////////////
        Beam beam = view.createPrimaryBeam(pixel);
        isects.clear();
        bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eExact);
        primary.clear();
        primary.vpTrans = beam.projTrans * beam.viewTrans;
        primary.apex = beam.frustum.apex;
        primary.run(*scene, bvh, isects);

        //////////////////////////////////////////////////////////////////////////
        // Debug
        //dumpVectorizerInput(scene, pixel, AABB2(Point2(-1.0f), Point2(1.0f)), primary.positions, nullptr);
        //////////////////////////////////////////////////////////////////////////
        vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
        vect.build(
            reinterpret_cast<vtrz::Vector3 *>(primary.positions.data()), (uint32_t)primary.positions.size(),
            nullptr, 0);

        Spectrum L;
        if (onlyCoverage) {
            L = Spectrum(Float(vect.coverageQuery()));
        } else {
            L = multiSample ?
                evalDirectLightingMultiV2(primary, incCtx, view.sensorPos, *scene, view.persp, pixel) :
                evalDirectLightingV2(primary, incCtx, view.sensorPos, *scene, view.persp, pixel);
        }

        // Do I actually need pixel filtering?
        Point2 pixelCenter(Point2(pixel) + Vector2(0.5, 0.5));
        // No alpha.
        block->put(pixelCenter, L, 1.0);
        if (outputRaw) {
            Vector2i res = scene->getFilm()->getCropSize();
            uint32_t offset = (pixel.y * res.x + pixel.x) * 3;
            L.toLinearRGB(rawPixels[offset], rawPixels[offset + 1], rawPixels[offset + 2]);
        }
    }
}

Spectrum VDirectIntegrator::evalDirectLightingV2(
    const PrimaryPipeline &primary, IncrementalContext &incCtx,
    const Point &sensorPos, const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel) const
{
    RayDifferential rayDiff;
    persp.sampleRayDifferential(rayDiff, Point2(pixel) + Point2(0.5, 0.5), Point2(0.0), Float(0.0));
    // Do not consider rayDiff scaling for now.

    Spectrum L(0.0f);
    vtrz::PointSample sample = incCtx.vect->pointSample();

    if (sample.weight > 0.0f) {
        Intersection its = primary.buildItsFromSample(sample, sensorPos, scene);
        its.getBSDF(rayDiff); // Calculate its partials here.
        uint32_t emitterCount = (uint32_t)scene.getEmitters().size();
        for (uint32_t emitterIndex = 0; emitterIndex < emitterCount; ++emitterIndex) {
            const Emitter &emitter = *(scene.getEmitters()[emitterIndex]);
            if (emitter.isEnvironmentEmitter()) continue;
            L += evalDirectLightingSampleV2(incCtx, its, sensorPos, scene, emitter);
        }
        L *= sample.weight;
    }

    L += (1.0f - sample.weight) * scene.evalEnvironment(rayDiff);

    return L;
}

Spectrum VDirectIntegrator::evalDirectLightingMultiV2(
    const PrimaryPipeline &primary, IncrementalContext &incCtx,
    const Point &sensorPos, const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel) const
{
    RayDifferential rayDiff;
    persp.sampleRayDifferential(rayDiff, Point2(pixel) + Point2(0.5, 0.5), Point2(0.0), Float(0.0));
    // Do not consider rayDiff scaling for now.

    Spectrum L(0.0f);
    float accWeight = 0.0f;
    vtrz::PointSampleArray samples = incCtx.vect->pointSampleMulti(primary.matIds.data(),
        reinterpret_cast<const vtrz::Vector3 *>(primary.normals.data()));

    Intersection *its = (Intersection *)alloca(samples.size * sizeof(Intersection));
    for (uint8_t i = 0; i < samples.size; ++i) {
        its[i] = primary.buildItsFromSample(samples.entries[i], sensorPos, scene);
        its[i].getBSDF(rayDiff); // Calculate its partials here.
        accWeight += samples.entries[i].weight;
    }

    // Slightly better cache performance if loop this way?
    uint32_t emitterCount = (uint32_t)scene.getEmitters().size();
    for (uint32_t emitterIndex = 0; emitterIndex < emitterCount; ++emitterIndex) {
        const Emitter &emitter = *(scene.getEmitters()[emitterIndex]);
        if (emitter.isEnvironmentEmitter()) continue;
        for (uint8_t i = 0; i < samples.size; ++i) {
            L += samples.entries[i].weight * evalDirectLightingSampleV2(incCtx, its[i], sensorPos, scene, emitter);
        }
    }

    accWeight = math::clamp(accWeight, 0.0f, 1.0f);
    L += (1.0f - accWeight) * scene.evalEnvironment(rayDiff);

    return L;
}

Spectrum VDirectIntegrator::evalDirectLightingSampleV2(
    IncrementalContext &incCtx, const Intersection &its,
    const Point &sensorPos, const Scene &scene, const Emitter &emitter) const
{
    const Shape *emitterShape = emitter.getShape();
    // Only area lights so far.
    Assert(emitterShape->getClass()->derivesFrom(MTS_CLASS(TriMesh)));
    const TriMesh *emitterMesh = static_cast<const TriMesh *>(emitterShape);
    // Hack. Assume constant radiance.
    Spectrum emitterRadiance = emitter.evalPosition(PositionSamplingRecord()) * INV_PI;
    if (its.shape == emitterMesh) {
        return emitterRadiance;
    }

    Beam beam;
    if (!createSecondaryBeam(its, emitter, beam)) {
        return Spectrum(0.0f);
    }
    incCtx.frustum = beam.frustum;
    incCtx.pipeline->vpTrans = beam.projTrans * beam.viewTrans;
    incCtx.pipeline->apex = beam.frustum.apex;
    incCtx.queryMesh = emitterMesh;

    incCtx.pipeline->clear();
    incCtx.pipeline->run(emitterMesh);

    constexpr uint32_t kVHiZResolution = 16;
    incCtx.vect->reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), kVHiZResolution, vtrz::DepthFunc::eGreater);

    incCtx.vect->addOccludees(
        reinterpret_cast<vtrz::Vector3 *>(incCtx.pipeline->positions.data()),
        (uint32_t)incCtx.pipeline->positions.size());
    incCtx.vect->finishOccludees();

    bvh.vectorizeOcclusion(incCtx);
    incCtx.vect->finishOcclusion();

    Spectrum bsdfInt(0.0);
    if (incCtx.vect->leafCount() > 0) {
        Vector N = (Vector)its.shFrame.n;
        Vector V = normalize(sensorPos - its.p);
        Float NdotV = dot(N, V);
        Transform invProjTrans = beam.projTrans.inverse();
        Transform invViewTrans = beam.viewTrans.inverse();
        BSDFPolyIntegralContext context(*incCtx.vect,
            invProjTrans,
            invViewTrans,
            its, NdotV);
        const BSDF *bsdf = its.getBSDF();
        bsdfInt = bsdf->evalPolyIntegral(context);
    }

    return emitterRadiance * bsdfInt;
}

Spectrum VDirectIntegrator::Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const
{
    Log(EError, "VectDirectIntegrator doesn't really have a meaningful Li() method. This is likely a bug.");
    return Spectrum(0.0);
}

Spectrum VDirectIntegrator::E(const Scene *scene, const Intersection &its,
    const Medium *medium, Sampler *sampler, int nSamples,
    bool includeIndirect) const
{
    Log(EError, "Not implemented yet, but should?");
    return Spectrum(0.0);
}

bool VDirectIntegrator::preprocess(const Scene *scene, RenderQueue *queue,
    const RenderJob *job, int sceneResID, int sensorResID, int samplerResID)
{
    bvh.build(*scene);

    uint16_t label = 0;
    std::unordered_set<BSDF *> set;
    auto &meshes = scene->getMeshes();
    for (auto &mesh : meshes) {
        BSDF *bsdf = mesh->getBSDF();
        if (!set.count(bsdf)) {
            bsdf->setLabel(label++);
            set.insert(bsdf);
        }
    }

    if (outputRaw) {
        Vector2i res = scene->getFilm()->getCropSize();
        rawPixels = std::make_unique<float[]>(3 * res.x * res.y);
    }

    areaLights.clear();
    pointLights.clear();
    for (uint32_t i = 0; i < (uint32_t)scene->getEmitters().size(); ++i) {
        const Emitter *emitter = scene->getEmitters()[i].get();
        if (emitter->isOnSurface()) areaLights.push_back(emitter);
        else if (emitter->isDegenerate()) pointLights.push_back(emitter);
    }

    computePointShadows(*scene);

    return true;
}

void VDirectIntegrator::computePointShadows(const Scene &scene)
{
    pointShadows.resize(pointLights.size());
    for (uint32_t lightIdx = 0; lightIdx < (uint32_t)pointLights.size(); ++lightIdx) {
        const Emitter &light = *pointLights[lightIdx];
        PositionSamplingRecord pRec;
        light.samplePosition(pRec, Point2());
        Vector directions[6] = {
            Vector(1.0, 0.0, 0.0),  // +x
            Vector(-1.0, 0.0, 0.0), // -x
            Vector(0.0, 1.0, 0.0),  // +y
            Vector(0.0, -1.0, 0.0), // -y
            Vector(0.0, 0.0, 1.0),  // +z
            Vector(0.0, 0.0, -1.0), // -z
        };
        for (uint32_t face = 0; face < 6; ++face) {
            Beam beam;
            Vector up(0.0, 1.0, 0.0);
            if (face == 2 || face == 3) {
                up = Vector3(0.0, 0.0, 1.0);
            }
            beam.viewTrans = Transform::lookAt(pRec.p, pRec.p + directions[face], up).inverse();
            Matrix4x4 flipViewMtx;
            flipViewMtx.setIdentity();
            flipViewMtx.m[0][0] = -1.0;
            flipViewMtx.m[2][2] = -1.0;
            beam.viewTrans = Transform(flipViewMtx) * beam.viewTrans;
            Vector right(
                beam.viewTrans.getMatrix()(0, 0),
                beam.viewTrans.getMatrix()(0, 1),
                beam.viewTrans.getMatrix()(0, 2)
            );
            up = Vector3(
                beam.viewTrans.getMatrix()(1, 0),
                beam.viewTrans.getMatrix()(1, 1),
                beam.viewTrans.getMatrix()(1, 2)
            );
            Vector frustumCornerDirs[4];
            frustumCornerDirs[0] = directions[face] - right - up;
            frustumCornerDirs[1] = directions[face] + right - up;
            frustumCornerDirs[2] = directions[face] + right + up;
            frustumCornerDirs[3] = directions[face] - right + up;
            beam.frustum = Frustum(pRec.p, directions[face], frustumCornerDirs, kSecondaryNearClip, FrustumAABBOption::eCoarse);
            beam.projTrans = Transform(revInfPerspective(kSecondaryNearClip,
                0.5F * M_PI, 1.0f, AABB2(Point2(-1.0f), Point2(1.0f))));

            FrustumBVHIsectResult isects;
            bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eCoarse);
            PrimaryPipeline pipeline;
            pipeline.clear();
            pipeline.vpTrans = beam.projTrans * beam.viewTrans;
            pipeline.apex = beam.frustum.apex;
            pipeline.run(scene, bvh, isects);

            pointShadows[lightIdx].transforms[face] = pipeline.vpTrans;
            pointShadows[lightIdx].meshIds[face] = std::move(pipeline.meshIds);
            pointShadows[lightIdx].primIds[face] = std::move(pipeline.primIds);
            pointShadows[lightIdx].worldPositions[face] = std::move(pipeline.worldPositions);
            pointShadows[lightIdx].wInvs[face] = std::move(pipeline.wInvs);

            vtrz::Vectorizer &vect = pointShadows[lightIdx].vects[face];
            vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
            vect.build(
                reinterpret_cast<vtrz::Vector3 *>(pipeline.positions.data()), (uint32_t)pipeline.positions.size(),
                nullptr, 0);
        }
    }
}

void VDirectIntegrator::postprocess(const Scene *scene, RenderQueue *queue,
    const RenderJob *job, int sceneResID, int sensorResID, int samplerResID)
{
    if (outputRaw) {
        const auto &dest = scene->getDestinationFile();
        auto stem = dest.stem();

        fs::path path = dest.parent_path() / dest.stem();
        path += "-raw.bin";
        std::ofstream file(path.c_str(), std::ios::binary);
        SAssert(file);

        Vector2i res = scene->getFilm()->getCropSize();
        size_t byteSize = sizeof(float) * 3 * res.x * res.y;
        file.write(reinterpret_cast<char *>(rawPixels.get()), byteSize);
        file.close();
    }
}

//////////////////////////////////////////////////////////////////////////
MTS_IMPLEMENT_CLASS_S(VDirectIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(VDirectIntegrator, "Vectorizer direct integrator");
MTS_NAMESPACE_END