#include "util.h"
#include "pipeline.h"
#include "bvh.h"
#include <mitsuba/core/statistics.h>

MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("Path tracer", "Average path length", EAverage);

class VPathTracer final : public MonteCarloIntegrator
{
public:
    MTS_DECLARE_CLASS();

    VPathTracer(const Properties &props)
        : MonteCarloIntegrator(props) { }

    VPathTracer(Stream *stream, InstanceManager *manager)
        : MonteCarloIntegrator(stream, manager) { }

    bool preprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID) final {

        bvh.build(*scene);
        neeCtxs.resize(Scheduler::getInstance()->getLocalWorkerCount());

        return true;
    }

    inline uint32_t getWorkerIndex() const {
        Scheduler *sched = Scheduler::getInstance();
        Thread *thread = Thread::getThread();
        uint32_t workerCount = (uint32_t)sched->getWorkerCount();
        for (uint32_t idx = 0; idx < workerCount; ++idx) {
            if (thread == sched->getWorker(idx)) {
                return idx;
            }
        }
        SAssert(false);
        return 0;
    }

    Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {
        uint32_t workerIndex = getWorkerIndex();

        /* Some aliases and local variables */
        const Scene *scene = rRec.scene;
        Intersection &its = rRec.its;
        RayDifferential ray(r);
        Spectrum Li(0.0f);
        bool scattered = false;

        /* Perform the first ray intersection (or ignore if the
        intersection has already been provided). */
        rRec.rayIntersect(ray);
        ray.mint = Epsilon;

        Spectrum throughput(1.0f);
        Float eta = 1.0f;

        while (rRec.depth <= m_maxDepth || m_maxDepth < 0) {
            if (!its.isValid()) {
                /* If no intersection could be found, potentially return
                   radiance from a environment luminaire if it exists */
                if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
                    && (!m_hideEmitters || scattered))
                    Li += throughput * scene->evalEnvironment(ray);
                break;
            }

            const BSDF *bsdf = its.getBSDF(ray);

            /* Possibly include emitted radiance if requested */
            if (its.isEmitter() && (rRec.type & RadianceQueryRecord::EEmittedRadiance)
                && (!m_hideEmitters || scattered))
                Li += throughput * its.Le(-ray.d);

            /* Include radiance from a subsurface scattering model if requested */
            if (its.hasSubsurface() && (rRec.type & RadianceQueryRecord::ESubsurfaceRadiance))
                Li += throughput * its.LoSub(scene, rRec.sampler, -ray.d, rRec.depth);

            if ((rRec.depth >= m_maxDepth && m_maxDepth > 0)
                || (m_strictNormals && dot(ray.d, its.geoFrame.n)
                    * Frame::cosTheta(its.wi) >= 0)) {

                /* Only continue if:
                   1. The current path length is below the specifed maximum
                   2. If 'strictNormals'=true, when the geometric and shading
                      normals classify the incident direction to the same side */
                break;
            }

            // Direct lighting.
            Li += throughput * evalDirectLighting(its, rRec, workerIndex);


            Float bsdfPdf;
            BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
            Spectrum bsdfWeight = bsdf->sample(bRec, bsdfPdf, rRec.nextSample2D());
            if (bsdfWeight.isZero())
                break;

            scattered |= bRec.sampledType != BSDF::ENull;

            /* Prevent light leaks due to the use of shading normals */
            const Vector wo = its.toWorld(bRec.wo);
            Float woDotGeoN = dot(its.geoFrame.n, wo);
            if (m_strictNormals && woDotGeoN * Frame::cosTheta(bRec.wo) <= 0)
                break;

            /* Keep track of the throughput and relative
            refractive index along the path */
            throughput *= bsdfWeight;
            eta *= bRec.eta;

            /* Set the recursive query type. Stop if no surface was hit by the
               BSDF sample or if indirect illumination was not requested */
            if (!its.isValid() || !(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance))
                break;
            rRec.type = RadianceQueryRecord::ERadianceNoEmission;

            if (rRec.depth++ >= m_rrDepth) {
                /* Russian roulette: try to keep path weights equal to one,
                   while accounting for the solid angle compression at refractive
                   index boundaries. Stop with at least some probability to avoid
                   getting stuck (e.g. due to total internal reflection) */

                Float q = std::min(throughput.max() * eta * eta, (Float) 0.95f);
                if (rRec.nextSample1D() >= q)
                    break;
                throughput /= q;
            }

            /* Trace a ray in this direction */
            ray = Ray(its.p, wo, ray.time);
            rRec.rayIntersect(ray);
        }

        /* Store statistics */
        avgPathLength.incrementBase();
        avgPathLength += rRec.depth;

        return Li;
    }

    Spectrum evalDirectLighting(const Intersection &its, RadianceQueryRecord &rRec, uint32_t workerIndex) const {
        const Scene &scene = *rRec.scene;
        // Pick an area light.
        Float emPdf;
        // its.shape->getEmitter()
        const Emitter &emitter = scene.selectOneEmitter(rRec.nextSample1D(), emPdf);
        SAssert(emitter.isOnSurface());
        const Shape *emitterShape = emitter.getShape();
        if (emitterShape == its.shape) {
            // TODO: avoid my self.
            return Spectrum(0.0);
        }

        const TriMesh *emitterMesh = static_cast<const TriMesh *>(emitterShape);

        // Hack. Assume constant radiance.
        Spectrum emitterRadiance = emitter.evalPosition(PositionSamplingRecord()) * INV_PI;

        Beam beam;
        if (!createSecondaryBeam(its, emitter, beam)) {
            return Spectrum(0.0f);
        }

        NEEContext &nee = neeCtxs[workerIndex];
        FrustumBVHIsectResult &isects = nee.isects;
        SecondaryPipeline &secondary = nee.pipeline;
        vtrz::Vectorizer &vect = nee.vect;

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

        vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
        vect.buildOcclusion(
            reinterpret_cast<vtrz::Vector3 *>(secondary.positions.data()), (uint32_t)secondary.positions.size(),
            nullptr, 0,
            secondary.queryTriIndices.data(), (uint32_t)secondary.queryTriIndices.size());

        Spectrum bsdfInt(0.0);
        if (vect.leafCount() > 0) {
            Float NdotV = its.wi.z;
            Transform invProjTrans = beam.projTrans.inverse();
            Transform invViewTrans = beam.viewTrans.inverse();
            BSDFPolyIntegralContext context(vect,
                invProjTrans,
                invViewTrans,
                its, NdotV);
            const BSDF *bsdf = its.getBSDF();
            bsdfInt = bsdf->evalPolyIntegral(context);
        }

        return emitterRadiance * bsdfInt / emPdf;
    }

private:
    BVH bvh;

    struct NEEContext
    {
        FrustumBVHIsectResult isects;
        SecondaryPipeline pipeline;
        vtrz::Vectorizer vect;
    };
    mutable std::vector<NEEContext> neeCtxs;
};

//////////////////////////////////////////////////////////////////////////
MTS_IMPLEMENT_CLASS_S(VPathTracer, false, MonteCarloIntegrator)
MTS_EXPORT_PLUGIN(VPathTracer, "Vectorizer hybrid path tracer");
MTS_NAMESPACE_END