#include "util.h"
#include "bvh.h"
#include "pipelinead.h"
#include "scenead.h"
#include <mitsuba/core/bitmap.h>
#include <mitsuba/render/renderproc.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>
#include <boost/filesystem/path.hpp>
#include <unordered_set>

MTS_NAMESPACE_BEGIN

class VDirectADIntegrator final : public SamplingIntegrator
{
public:
    MTS_DECLARE_CLASS();

    VDirectADIntegrator(const Properties &props);

    VDirectADIntegrator(Stream *stream, InstanceManager *manager);

    bool preprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID) final;

    void postprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) final;

    void computePointShadows(const Scene &scene);

    //bool render(Scene *scene, RenderQueue *queue, const RenderJob *job,
    //    int sceneResID, int sensorResID, int samplerResID) final;

    void renderBlock(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points) const final;

    vtrz::DVector3 evalDirectLighting(
        vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary,
        SecondaryPipelineAD &secondary, FrustumBVHIsectResult &result,
        const Point &sensorPos, const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel) const;

    vtrz::DVector3 evalDirectLightingArea(vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary,
        SecondaryPipelineAD &secondary, FrustumBVHIsectResult &result,
        const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const;

    vtrz::DVector3 evalDirectLightingAreaMulti(vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary,
        SecondaryPipelineAD &secondary, FrustumBVHIsectResult &result,
        const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const;

    vtrz::DVector3 evalDirectLightingAreaSample(
        vtrz::VectorizerGrad &vect, SecondaryPipelineAD &secondary, FrustumBVHIsectResult &result,
        const IntersectionGrad &itsGrad, const Point &sensorPos, const Scene &scene, const Emitter &emitter) const;

    vtrz::DVector3 evalDirectLightingPoint(
        const vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary,
        const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, uint32_t pointLightIndex) const;

    void writePixel(const Point2i &pixel, const vtrz::DVector3 &L, ImageBlock &block) const;

    Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const final;

    Spectrum E(const Scene *scene, const Intersection &its,
        const Medium *medium, Sampler *sampler, int nSamples,
        bool includeIndirect) const final;

private:
    bool onlyCoverage = false;
    bool multiSample = false;
    float gradientScale = 1.0f;
    enum OutputGradientMode : int
    {
        EComponent = 1 << 0,
        EMagnitude = 1 << 1,
    };
    OutputGradientMode outputGradientMode;

    std::vector<std::string> transformADInfo;

    enum class MeshDisplacementType
    {
        EConstant,
        ESine,
        EFromData,
    };
    struct PerVertexADInfo
    {
        bool enabled = false;
        std::string meshName;
        MeshDisplacementType displaceType;
        Spectrum params;
        fs::path coeffsPath;
        std::vector<float> coefficients;
        bool outputCoeffs = false;
    };
    PerVertexADInfo perVertexADInfo;

    std::vector<std::string> pointLightADInfo;

    uint32_t totalGradDim = 0;
    SceneAD sceneAD;

    mutable ref_vector<Bitmap> gradImages;

    BVH bvh;
};

// Mesh transform gradient:
// Format: (no space allowed)
// "transformAD0": "shapeRefId:tx+|ty-|tz+|r(x,y,z)|sx+|sy-|sz+"
static vtrz::DMatrix4x4 parseTransformDerivative(std::string &token, uint32_t gradDim) {
    // Don't care about values.
    SAssert(token.size() > 0 && (token[0] == 't' || token[0] == 'r' || token[0] == 's'));
    if (token[0] == 't') {
        // translation
        SAssert(token[1] == 'x' || token[1] == 'y' || token[1] == 'z');
        SAssert(token[2] == '+' || token[2] == '-');
        if      (token[1] == 'x' && token[2] == '+') return vtrz::makeTranslate(vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(), vtrz::dfloat());
        else if (token[1] == 'x' && token[2] == '-') return vtrz::makeTranslate(-vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(), vtrz::dfloat());
        else if (token[1] == 'y' && token[2] == '+') return vtrz::makeTranslate(vtrz::dfloat(), vtrz::dfloat(0.0f, gradDim), vtrz::dfloat());
        else if (token[1] == 'y' && token[2] == '-') return vtrz::makeTranslate(vtrz::dfloat(), -100.0f * vtrz::dfloat(0.0f, gradDim), vtrz::dfloat());
        else if (token[1] == 'z' && token[2] == '+') return vtrz::makeTranslate(vtrz::dfloat(), vtrz::dfloat(), vtrz::dfloat(0.0f, gradDim));
        else/*(token[1] == 'z' && token[2] == '-')*/ return vtrz::makeTranslate(vtrz::dfloat(), vtrz::dfloat(), -vtrz::dfloat(0.0f, gradDim));
    } else if (token[0] == 'r') {
        // rotation
        std::string axisAngle = token.substr(2);
        Vector field;
        size_t start = 0;
        size_t end = 0;
        for (uint32_t i = 0; i < 3; ++i) {
            end = axisAngle.find(i < 2 ? ',' : ')', start);
            SAssert(end != std::string::npos);
            field[i] = std::stof(axisAngle.substr(start, end - start));
            start = end + 1;
        }
        SAssert(field.lengthSquared() > 0);
        field = normalize(field);
        vtrz::DVector3 axis(field[0], field[1], field[2]);
        vtrz::dfloat angle(0.0f, gradDim);
        return vtrz::makeRotate(angle, axis);
    } else {
        // token[0] == 's'
        // scaling
        SAssert(token[1] == 'x' || token[1] == 'y' || token[1] == 'z' || token[1] == 'a');
        SAssert(token[2] == '+' || token[2] == '-');
        if      (token[1] == 'x' && token[2] == '+') return vtrz::makeScale(vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(), vtrz::dfloat());
        else if (token[1] == 'x' && token[2] == '-') return vtrz::makeScale(-vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(), vtrz::dfloat());
        else if (token[1] == 'y' && token[2] == '+') return vtrz::makeScale(vtrz::dfloat(), vtrz::dfloat(0.0f, gradDim), vtrz::dfloat());
        else if (token[1] == 'y' && token[2] == '-') return vtrz::makeScale(vtrz::dfloat(), -vtrz::dfloat(0.0f, gradDim), vtrz::dfloat());
        else if (token[1] == 'z' && token[2] == '+') return vtrz::makeScale(vtrz::dfloat(), vtrz::dfloat(), vtrz::dfloat(0.0f, gradDim));
        else if (token[1] == 'z' && token[2] == '-') return vtrz::makeScale(vtrz::dfloat(), vtrz::dfloat(), -vtrz::dfloat(0.0f, gradDim));
        // 'a' means all axes.
        else if (token[1] == 'a' && token[2] == '+') return vtrz::makeScale(vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(0.0f, gradDim));
        else /*if (token[1] == 'a' && token[2] == '-')*/ return vtrz::makeScale(-vtrz::dfloat(0.0f, gradDim), -vtrz::dfloat(0.0f, gradDim), -vtrz::dfloat(0.0f, gradDim));
    }
}

static vtrz::DMatrix4x4 parseTransformAD(std::string &str, uint32_t &gradDim) {

    vtrz::DMatrix4x4 grad(0.0f);

    constexpr char delimiter = '|';
    size_t last = 0;
    size_t next = 0;
    while ((next = str.find(delimiter, last)) != std::string::npos) {
        if (gradDim >= vtrz::kMaxGradDim) {
            break;
        }
        grad += parseTransformDerivative(str.substr(last, next - last), gradDim++);
        last = next + 1;
    }
    if (gradDim < vtrz::kMaxGradDim) {
        grad += parseTransformDerivative(str.substr(last), gradDim++);
    }
    // Don't care about values.
    return grad;
}

//////////////////////////////////////////////////////////////////////////
VDirectADIntegrator::VDirectADIntegrator(const Properties &props) :
    SamplingIntegrator(props)
{
    onlyCoverage = props.getBoolean("onlycoverage", false);
    multiSample = props.getBoolean("multisample", false);
    gradientScale = props.getFloat("gradscale", 1.0f);
    std::string mode = props.getString("outputgradmode", "component");
    outputGradientMode = (OutputGradientMode)0;
    if (mode == "none") {
        outputGradientMode = (OutputGradientMode)0;
    } else if (mode == "component") {
        outputGradientMode = OutputGradientMode::EComponent;
    } else if (mode == "magnitude") {
        outputGradientMode = OutputGradientMode::EMagnitude;
    } else if (mode == "both") {
        outputGradientMode = OutputGradientMode(OutputGradientMode::EComponent | OutputGradientMode::EMagnitude);
    } else {
        SLog(EError, "Wrong output gradient mode.");
    }
    for (uint32_t i = 0; ; ++i) {
        std::string key = "transformAD" + std::to_string(i);
        if (props.hasProperty(key)) {
            transformADInfo.push_back(props.getString(key));
        } else {
            break;
        }
    }

    //ref<FileResolver> fileResolver = Thread::getThread()->getFileResolver()->clone();
    //for (uint32_t i = 0; ; ++i) {
    //    std::string key = "perVertexAD" + std::to_string(i);
    //    if (props.hasProperty(key)) {
    //        std::string str = props.getString(key);
    //        size_t n = str.find(':');
    //        SAssert(n != std::string::npos);
    //        PerVertexADInfo info;
    //        info.meshName = str.substr(0, n);
    //        info.jacobianPath = fileResolver->resolve(str.substr(n + 1));
    //        perVertexADInfo.push_back(std::move(info));
    //    } else {
    //        break;
    //    }
    //}

    if (props.hasProperty("perVertexAD")) {
        perVertexADInfo.enabled = true;
        perVertexADInfo.meshName = props.getString("perVertexAD");
        SAssert(props.hasProperty("meshDispType"));
        perVertexADInfo.displaceType = MeshDisplacementType(props.getInteger("meshDispType"));
        if (perVertexADInfo.displaceType != MeshDisplacementType::EFromData) {
            SAssert(props.hasProperty("meshDispParams"));
            perVertexADInfo.params = props.getSpectrum("meshDispParams");
        } else {
            SAssert(props.hasProperty("meshDispCoeffs"));
            ref<FileResolver> fileResolver = Thread::getThread()->getFileResolver()->clone();
            perVertexADInfo.coeffsPath = fileResolver->resolve(props.getString("meshDispCoeffs"));
        }
        perVertexADInfo.outputCoeffs = props.getBoolean("outputVertexCoeffs", false);
    }

    for (uint32_t i = 0; ; ++i) {
        std::string key = "pointLightAD" + std::to_string(i);
        if (props.hasProperty(key)) {
            pointLightADInfo.push_back(props.getString(key));
        } else {
            break;
        }
    }
}

VDirectADIntegrator::VDirectADIntegrator(Stream *stream, InstanceManager *manager)
    : SamplingIntegrator(stream, manager)
{ }

bool VDirectADIntegrator::preprocess(
    const Scene *scene, RenderQueue *queue, const RenderJob *job,
    int sceneResID, int sensorResID, int samplerResID)
{
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

    totalGradDim = 0;
    sceneAD.indexMap.clear();
    sceneAD.meshADs.clear();

    SAssert(transformADInfo.empty() || !perVertexADInfo.enabled);

    for (const std::string &info : transformADInfo) {
        size_t n = info.find(':');
        SAssert(n != std::string::npos);
        std::string meshName = info.substr(0, n);
        n += 1;
        SAssert(info.length() > n && info[n] == 'l' || info[n] == 'w');
        TransformADType type = info[n] == 'l' ? TransformADType::eLocal : TransformADType::eWorld;
        n += 1;
        SAssert(info.length() > n && info[n] == ':');
        vtrz::DMatrix4x4 transAD = parseTransformAD(info.substr(n + 1), totalGradDim);

        for (uint32_t i = 0; i < meshes.size(); ++i) {
            if (meshes[i]->getID() != meshName) continue;
            sceneAD.indexMap.insert({ i, (uint32_t)sceneAD.meshADs.size() });
            sceneAD.meshADs.push_back(TriMeshAD(transAD, type, *meshes[i]));
            break;
        }
    }

    //for (const auto &info : perVertexADInfo) {
    //    for (uint32_t i = 0; i < meshes.size(); ++i) {
    //        if (meshes[i]->getID() != info.meshName) continue;
    //        sceneAD.indexMap.insert({ i, (uint32_t)sceneAD.meshADs.size() });
    //        // Hack: if not face normal, we need to recalculate smooth normal after update on vertex positions.
    //        meshes[i]->computeNormals(true);
    //        sceneAD.meshADs.push_back(TriMeshAD(info.jacobianPath, *meshes[i]));
    //        totalGradDim += (uint32_t)meshes[i]->getVertexCount();
    //        break;
    //    }
    //}

    if (perVertexADInfo.enabled) {
        for (uint32_t i = 0; i < meshes.size(); ++i) {
            if (meshes[i]->getID() != perVertexADInfo.meshName) continue;
            sceneAD.indexMap.insert({ i, (uint32_t)sceneAD.meshADs.size() });
            switch (perVertexADInfo.displaceType) {
            case MeshDisplacementType::EConstant: {
                float t = perVertexADInfo.params[0];
                sceneAD.meshADs.push_back(SimpleMeshDisplacement::constant(t, *meshes[i],
                    perVertexADInfo.coefficients));
                break;
            }
            case MeshDisplacementType::ESine: {
                float t = perVertexADInfo.params[0];
                float fphi = perVertexADInfo.params[1];
                float ftheta = perVertexADInfo.params[2];
                sceneAD.meshADs.push_back(SimpleMeshDisplacement::sine(t, fphi, ftheta, *meshes[i],
                    perVertexADInfo.coefficients));
                break;
            }
            case MeshDisplacementType::EFromData: {
                sceneAD.meshADs.push_back(SimpleMeshDisplacement::fromData(perVertexADInfo.coeffsPath, *meshes[i],
                    perVertexADInfo.coefficients));
                break;
            }
            default: SAssert(false); break;
            }
            totalGradDim += (uint32_t)meshes[i]->getVertexCount();
            break;
        }
    }

    auto &refObjs = scene->getReferencedObjects();
    for (auto &obj : refObjs) {
        if (obj->getClass()->derivesFrom(MTS_CLASS(BSDF))) {
            BSDF &bsdf = (BSDF &)(*obj);
            uint32_t dim = bsdf.gradDim();
            if (dim > 0) {
                SAssert(!perVertexADInfo.enabled);
            }
            bsdf.applyGrad(totalGradDim);
            totalGradDim += dim;
        }
    }

    // Need to build BVH after mesh displacements (if any).
    bvh.build(*scene);

    sceneAD.areaLights.clear();
    sceneAD.pointLights.clear();
    for (uint32_t i = 0; i < (uint32_t)scene->getEmitters().size(); ++i) {
        const Emitter *emitter = scene->getEmitters()[i].get();
        if (emitter->isOnSurface() && !emitter->isEnvironmentEmitter()) sceneAD.areaLights.push_back(emitter);
        else if (emitter->isDegenerate()) {
            PointLightAD light;
            light.light = emitter;
            PositionSamplingRecord pRec;
            Spectrum intensity = emitter->samplePosition(pRec, Point2()) * INV_FOURPI;
            vtrz::Vector3 rgb;
            intensity.toLinearRGB(rgb.x, rgb.y, rgb.z);
            light.intensity.setValue(rgb);
            light.position.setValue(toVec3(pRec.p));
            sceneAD.pointLights.push_back(light);
        }
    }

    for (const std::string &info : pointLightADInfo) {
        size_t n = info.find(':');
        SAssert(n != std::string::npos);
        std::string lightName = info.substr(0, n);
        for (auto &pointLight : sceneAD.pointLights) {
            if (pointLight.light->getID() == lightName) {
                std::string var = info.substr(n + 1);
                if (var == "+x") {
                    pointLight.position.x.grad[totalGradDim++] = 1.0f;
                } else if (var == "-x") {
                    pointLight.position.x.grad[totalGradDim++] = -1.0f;
                } else if (var == "+y") {
                    pointLight.position.y.grad[totalGradDim++] = 1.0f;
                } else if (var == "-y") {
                    pointLight.position.y.grad[totalGradDim++] = -1.0f;
                } else if (var == "+z") {
                    pointLight.position.z.grad[totalGradDim++] = 1.0f;
                } else if (var == "-z") {
                    pointLight.position.z.grad[totalGradDim++] = -1.0f;
                }
                else {
                    SAssert(false);
                }
                break;
            }
        }
    }

    computePointShadows(*scene);

    gradImages.resize(totalGradDim);
    for (uint32_t i = 0; i < totalGradDim; ++i) {
        gradImages[i] = new Bitmap(Bitmap::EPixelFormat::ERGB, Bitmap::EComponentFormat::EFloat32,
            scene->getFilm()->getCropSize());
        gradImages[i]->clear();
    }

    return true;
}

void VDirectADIntegrator::computePointShadows(const Scene &scene)
{
    sceneAD.pointShadows.resize(sceneAD.pointLights.size());
    for (uint32_t lightIdx = 0; lightIdx < (uint32_t)sceneAD.pointLights.size(); ++lightIdx) {
        const PointLightAD &light = sceneAD.pointLights[lightIdx];

        Vector directions[6] = {
            Vector(1.0, 0.0, 0.0),  // +x
            Vector(-1.0, 0.0, 0.0), // -x
            Vector(0.0, 1.0, 0.0),  // +y
            Vector(0.0, -1.0, 0.0), // -y
            Vector(0.0, 0.0, 1.0),  // +z
            Vector(0.0, 0.0, -1.0), // -z
        };
        for (uint32_t face = 0; face < 6; ++face) {
            BeamGrad beam;
            Vector up(0.0, 1.0, 0.0);
            if (face == 2 || face == 3) {
                up = Vector3(0.0, 0.0, 1.0);
            }
            beam.viewTrans = vtrz::makeLookupView(light.position, toVec3(directions[face]), toVec3(up));
            Vector right(
                beam.viewTrans.arr[0][0].value,
                beam.viewTrans.arr[0][1].value,
                beam.viewTrans.arr[0][2].value
            );
            up = Vector3(
                beam.viewTrans.arr[1][0].value,
                beam.viewTrans.arr[1][1].value,
                beam.viewTrans.arr[1][2].value
            );
            Vector frustumCornerDirs[4];
            frustumCornerDirs[0] = directions[face] - right - up;
            frustumCornerDirs[1] = directions[face] + right - up;
            frustumCornerDirs[2] = directions[face] + right + up;
            frustumCornerDirs[3] = directions[face] - right + up;
            beam.frustum = Frustum(toPoint(light.position.value()), directions[face], frustumCornerDirs, kSecondaryNearClip, FrustumAABBOption::eCoarse);
            beam.projTrans = castMatrix4x4(revInfPerspective(kSecondaryNearClip,
                0.5F * M_PI, 1.0f, AABB2(Point2(-1.0f), Point2(1.0f))));

            FrustumBVHIsectResult isects;
            bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eCoarse);
            PrimaryPipelineAD pipeline;
            pipeline.clear();
            pipeline.vpTrans = beam.projTrans * beam.viewTrans;
            pipeline.apex = beam.frustum.apex;
            pipeline.run(scene, bvh, sceneAD, isects);

            sceneAD.pointShadows[lightIdx].transforms[face] = pipeline.vpTrans;
            sceneAD.pointShadows[lightIdx].meshIds[face] = std::move(pipeline.meshIds);
            sceneAD.pointShadows[lightIdx].primIds[face] = std::move(pipeline.primIds);
            sceneAD.pointShadows[lightIdx].worldPositions[face] = std::move(pipeline.worldPositions);
            sceneAD.pointShadows[lightIdx].wInvs[face] = std::move(pipeline.wInvs);

            vtrz::VectorizerGrad &vect = sceneAD.pointShadows[lightIdx].vects[face];
            vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
            vect.build(
                pipeline.positions.data(), (uint32_t)pipeline.positions.size(),
                nullptr, 0);
        }
    }
}

void VDirectADIntegrator::postprocess(
    const Scene *scene, RenderQueue *queue, const RenderJob *job,
    int sceneResID, int sensorResID, int samplerResID)
{
    const auto &dest = scene->getDestinationFile();
    auto stem = dest.stem();
    if (outputGradientMode & OutputGradientMode::EComponent) {
        for (uint32_t i = 0; i < totalGradDim; ++i) {
            std::string postfix = "-grad" + std::to_string(i) + ".exr";
            fs::path path = dest.parent_path() / dest.stem();
            path += fs::path(std::move(postfix));
            gradImages[i]->write(path);
        }
    }
    if (outputGradientMode & OutputGradientMode::EMagnitude) {
        Vector2i res = scene->getFilm()->getCropSize();
        ref<Bitmap> gradMag = new Bitmap(Bitmap::EPixelFormat::ERGB, Bitmap::EComponentFormat::EFloat32, res);
        SAssert(gradMag->getChannelCount() == 3);
        gradMag->clear();
        float *mag = gradMag->getFloat32Data();
        uint32_t numPixels = (uint32_t)(res.x * res.y);
        for (uint32_t i = 0; i < totalGradDim; ++i) {
            float *data = gradImages[i]->getFloat32Data();
            for (uint32_t j = 0; j < numPixels; ++j) {
                for (uint32_t k = 0; k < 3; ++k) {
                    mag[j * 3 + k] += data[j * 3 + k] * data[j * 3 + k];
                }
            }
        }
        for (uint32_t j = 0; j < numPixels; ++j) {
            for (uint32_t k = 0; k < 3; ++k) {
                mag[j * 3 + k] = std::sqrt(mag[j * 3 + k]);
            }
        }
        std::string postfix = "-gradmag.exr";
        fs::path path = dest.parent_path() / dest.stem();
        path += fs::path(std::move(postfix));
        gradMag->write(path);
    }

    if (perVertexADInfo.enabled && perVertexADInfo.outputCoeffs) {
        fs::path path = dest.parent_path() / fs::path("coeffs.bin");
        ref<FileStream> fs = new FileStream(path, FileStream::ETruncWrite);
        fs->writeUInt((uint32_t)perVertexADInfo.coefficients.size());
        fs->writeFloatArray(perVertexADInfo.coefficients.data(), perVertexADInfo.coefficients.size());
    }
}

void VDirectADIntegrator::renderBlock(
    const Scene *scene, const Sensor *sensor, Sampler *sampler,
    ImageBlock *block, const bool &stop, const std::vector< TPoint2<uint8_t> > &points) const
{
    block->clear();

    SAssert(sensor->getClass()->derivesFrom(MTS_CLASS(PerspectiveCamera)));
    ViewInfo view(*sensor);

    FrustumBVHIsectResult isects;
    PrimaryPipelineAD primary;
    SecondaryPipelineAD secondary;
    vtrz::VectorizerGrad vect;

    for (auto pt : points) {
        if (stop) {
            break;
        }

        Point2i pixel = Point2i(pt) + Vector2i(block->getOffset());
        //////////////////////////////////////////////////////////////////////////
        // Debug
        // if (pixel.x != 1119 || pixel.y != 413) continue;
        //////////////////////////////////////////////////////////////////////////
        Beam beam = view.createPrimaryBeam(pixel);
        isects.clear();
        bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eExact);
        primary.clear();
        primary.vpTrans = vtrz::DMatrix4x4(castMatrix4x4((beam.projTrans * beam.viewTrans).getMatrix()));
        primary.apex = beam.frustum.apex;
        primary.run(*scene, bvh, sceneAD, isects);

        //////////////////////////////////////////////////////////////////////////
        // Debug
        //dumpVectorizerInput(scene, pixel, AABB2(Point2(-1.0f), Point2(1.0f)), pipeline.positions, nullptr, nullptr, &pipeline.grads);
        //////////////////////////////////////////////////////////////////////////

        vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
        vect.build(
            primary.positions.data(), (uint32_t)primary.positions.size(),
            nullptr, 0);

        vtrz::DVector3 L;
        if (onlyCoverage) {
            vtrz::dfloat c = vect.coverageQuery();
            if (c.value > 0.999f) c.zeroGrad();
            L = vtrz::DVector3(c);
        } else {
            L = evalDirectLighting(vect, primary, secondary, isects, view.sensorPos, *scene, view.persp, pixel);
        }

        writePixel(pixel, L, *block);
    }
}

void VDirectADIntegrator::writePixel(const Point2i &pixel, const vtrz::DVector3 &L, ImageBlock &block) const
{
    // Do I actually need pixel filtering?
    Point2 pixelCenter(Point2(pixel) + Vector2(0.5, 0.5));
    Spectrum value;
    value.fromLinearRGB(L.x.value, L.y.value, L.z.value);
    block.put(pixelCenter, value, 1.0f);
    for (uint32_t i = 0; i < totalGradDim; ++i) {
        float *data = gradImages[i]->getFloat32Data();
        uint32_t offset = (pixel.y * gradImages[i]->getWidth() + pixel.x) * gradImages[i]->getChannelCount();
        data[offset] = L.x.grad[i] * gradientScale;
        data[offset + 1] = L.y.grad[i] * gradientScale;
        data[offset + 2] = L.z.grad[i] * gradientScale;
    }
}


vtrz::DVector3 VDirectADIntegrator::evalDirectLighting(
    vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary, SecondaryPipelineAD &secondary,
    FrustumBVHIsectResult &isects, const Point &sensorPos, const Scene &scene,
    const PerspectiveCamera &persp, const Point2i &pixel) const
{
    RayDifferential rayDiff;
    persp.sampleRayDifferential(rayDiff, Point2(pixel) + Point2(0.5, 0.5), Point2(0.0), Float(0.0));
    // Do not consider rayDiff scaling for now.

    vtrz::DVector3 L(0.0f);
    for (uint32_t pointLightIndex = 0; pointLightIndex < (uint32_t)sceneAD.pointLights.size(); ++pointLightIndex) {
        L += evalDirectLightingPoint(vect, primary, sensorPos, scene, rayDiff, pointLightIndex);
    }

    if (!multiSample) {
        L += evalDirectLightingArea(vect, primary, secondary, isects, sensorPos, scene, rayDiff, pixel);
    } else {
        L += evalDirectLightingAreaMulti(vect, primary, secondary, isects, sensorPos, scene, rayDiff, pixel);
    }

    return L;
}

vtrz::DVector3 VDirectADIntegrator::evalDirectLightingArea(
    vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary,
    SecondaryPipelineAD &secondary, FrustumBVHIsectResult &isects,
    const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const
{
    vtrz::PointSampleGrad sample = vect.pointSample();
    vtrz::DVector3 L(0.0f);

    if (sample.weight.value > 0.0f) {
        IntersectionGrad itsGrad = primary.buildItsFromSample(sample, sensorPos, scene, sceneAD);
        itsGrad.its.getBSDF(rayDiff); // Calculate its partials here.
        for (const Emitter *areaLight : sceneAD.areaLights) {
            L += evalDirectLightingAreaSample(vect, secondary, isects, itsGrad, sensorPos, scene, *areaLight);
        }
        L *= sample.weight;
    }

    vtrz::DVector3 Lenv;
    scene.evalEnvironment(rayDiff).toLinearRGB(Lenv.x.value, Lenv.y.value, Lenv.z.value);
    L += (vtrz::dfloat(1.0f) - sample.weight) * Lenv;

    return L;
}

vtrz::DVector3 VDirectADIntegrator::evalDirectLightingAreaMulti(
    vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary,
    SecondaryPipelineAD &secondary, FrustumBVHIsectResult &isects,
    const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff, const Point2i &pixel) const
{
    vtrz::DVector3 L(0.0f);
    vtrz::dfloat accWeight = 0.0f;

    vtrz::PointSampleArrayGrad samples = vect.pointSampleMulti(primary.matIds.data(),
        reinterpret_cast<const vtrz::Vector3 *>(primary.normals.data()));

    IntersectionGrad *itsGrad = (IntersectionGrad *)alloca(samples.size * sizeof(IntersectionGrad));
    for (uint8_t i = 0; i < samples.size; ++i) {
        itsGrad[i] = primary.buildItsFromSample(samples.entries[i], sensorPos, scene, sceneAD);
        itsGrad[i].its.getBSDF(rayDiff); // Calculate its partials here.
        accWeight += samples.entries[i].weight;
    }

    // Slightly better cache performance if loop this way?
    for (const Emitter *areaLight : sceneAD.areaLights) {
        for (uint8_t i = 0; i < samples.size; ++i) {
            L += samples.entries[i].weight * evalDirectLightingAreaSample(vect, secondary, isects, itsGrad[i], sensorPos, scene, *areaLight);
        }
    }

    accWeight.value = math::clamp(accWeight.value, 0.0f, 1.0f);
    vtrz::DVector3 Lenv;
    scene.evalEnvironment(rayDiff).toLinearRGB(Lenv.x.value, Lenv.y.value, Lenv.z.value);
    L += (vtrz::dfloat(1.0f) - accWeight) * Lenv;

    return L;
}

vtrz::DVector3 VDirectADIntegrator::evalDirectLightingAreaSample(
    vtrz::VectorizerGrad &vect, SecondaryPipelineAD &secondary, FrustumBVHIsectResult &isects,
    const IntersectionGrad &itsGrad, const Point &sensorPos,
    const Scene &scene, const Emitter &emitter) const
{
    const Shape *emitterShape = emitter.getShape();
    // Only area lights so far.
    Assert(emitterShape->getClass()->derivesFrom(MTS_CLASS(TriMesh)));
    const TriMesh *emitterMesh = static_cast<const TriMesh *>(emitterShape);
    // Hack. Assume constant radiance.
    Spectrum emitterRadiance = emitter.evalPosition(PositionSamplingRecord()) * INV_PI;
    vtrz::DVector3 emitterRadianceGrad;
    emitterRadiance.toLinearRGB(emitterRadianceGrad.x.value, emitterRadianceGrad.y.value, emitterRadianceGrad.z.value);
    if (itsGrad.its.shape == emitterMesh) {
        return emitterRadianceGrad;
    }

    BeamGrad beam;
    if (!createSecondaryBeamGrad(itsGrad, emitter, beam)) {
        return vtrz::DVector3(0.0f);
    }

    isects.clear();
    bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eCoarse);
    secondary.clear();
    secondary.vpTrans = beam.projTrans * beam.viewTrans;
    secondary.apex = beam.frustum.apex;
    secondary.run(scene, bvh, sceneAD, isects, static_cast<const TriMesh *>(emitter.getShape()));

    // Actually no emitter geometry is inside the frustum (clipped by horizon).
    if (secondary.queryTriIndices.empty()) {
        return vtrz::DVector3(0.0f);
    }

    ////////////////////////////////////////////////////////////////////////
    // Debug
    //dumpVectorizerInput(&scene, Point2i(17, 78), AABB2(Point2(-1.0f), Point2(1.0f)), pipeline.positions, nullptr, &pipeline.queryTriIndices, &pipeline.grads);
    ////////////////////////////////////////////////////////////////////////
    vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
    vect.buildOcclusion(
        secondary.positions.data(), (uint32_t)secondary.positions.size(),
        nullptr, 0,
        secondary.queryTriIndices.data(), (uint32_t)secondary.queryTriIndices.size());

    vtrz::DVector3 bsdfInt(0.0f);
    if (vect.leafCount() > 0) {
        vtrz::DVector3 N = itsGrad.dShFrame.n;
        vtrz::DVector3 V = vtrz::normalize(
            vtrz::DVector3(sensorPos.x, sensorPos.y, sensorPos.z) - itsGrad.dp);
        vtrz::dfloat NdotV = vtrz::dot(N, V);
        vtrz::Matrix4x4 invProjTrans = beam.projTrans.inverse();
        vtrz::DMatrix4x4 invViewTrans = beam.viewTrans.inverse();
        BSDFPolyIntegralGradContext context(vect,
            invProjTrans, invViewTrans,
            itsGrad.its, itsGrad.dShFrame.s, itsGrad.dShFrame.t, itsGrad.dShFrame.n,
            NdotV);
        const BSDF *bsdf = itsGrad.its.getBSDF();
        bsdf->evalPolyIntegralGrad(context, bsdfInt);
        // bsdf->evalPolyIntegralGrad(context);
    }

    return emitterRadianceGrad * bsdfInt;
}

vtrz::DVector3 VDirectADIntegrator::evalDirectLightingPoint(
    const vtrz::VectorizerGrad &vect, const PrimaryPipelineAD &primary,
    const Point &sensorPos, const Scene &scene, const RayDifferential &rayDiff,  uint32_t pointLightIndex) const
{
    const PointLightAD &light = sceneAD.pointLights[pointLightIndex];
    const PointShadowAD &shadow = sceneAD.pointShadows[pointLightIndex];
    vtrz::DVector3 sensorPosAD(toVec3(sensorPos));

    vtrz::DVector3 Lsum(0.0);
    vtrz::dfloat totalCoverage = vect.coverageQuery();

    vtrz::Matrix4x4 queryTransform = primary.vpTrans.value();

    uint32_t leafCount = vect.leafCount();

    for (uint32_t leafIdx = 0; leafIdx < leafCount; ++leafIdx) {
        vtrz::LeafRegionGrad region = vect.getLeaf(leafIdx);
        IntersectionPatchGrad patch;
        if (!primary.buildItsPatch(region, sensorPos, scene, sceneAD, patch)) {
            continue;
        }
        const BSDF *bsdf = patch.avgItsGrad.its.getBSDF(rayDiff);
        vtrz::DVector3 Lpatch(0.0);
        for (uint32_t i = 0; i < (uint32_t)patch.positions.size(); ++i) {
            patch.avgItsGrad.dp = patch.positions[i];
            patch.avgItsGrad.its.p = toPoint(patch.avgItsGrad.dp.value());
            if (!patch.normals.empty()) {
                const vtrz::DVector3 &dN = patch.normals[i];
                vtrz::DVector3 dV = vtrz::normalize(sensorPosAD - patch.avgItsGrad.dp);
                vtrz::DVector3 dT1 = vtrz::normalize(dV - dN * vtrz::dot(dV, dN));
                vtrz::DVector3 dT2 = vtrz::cross(dN, dT1);
                patch.avgItsGrad.dShFrame = DFrame(dT1, dT2, dN);
            }

            // ..weird convention?
            vtrz::DVector3 wo = light.position - patch.avgItsGrad.dp;
            vtrz::dfloat invDist2 = 1.0f / vtrz::length2(wo);
            wo *= vtrz::sqrt(invDist2);
            wo = patch.avgItsGrad.dShFrame.toLocal(wo);

            vtrz::DVector3 wi = vtrz::normalize(sensorPosAD - patch.avgItsGrad.dp);
            wi = patch.avgItsGrad.dShFrame.toLocal(wi);

            BSDFEvalGradContext ctx(patch.avgItsGrad.dp, wo, wi, patch.avgItsGrad.its);
            vtrz::DVector3 F;
            bsdf->evalGrad(ctx, F);

            Lpatch += F * light.intensity * invDist2;
        }
        SAssert(!vtrz::isNan(Lpatch));
        Lpatch /= (Float)patch.positions.size();

        vtrz::dfloat visibleCoverage(0.0);
        // Determine which faces and clipping again...
        SecondaryPipelineAD pipeline;
        for (uint32_t face = 0; face < 6; ++face) {
            pipeline.clear();
            pipeline.apex = toPoint(light.position.value());
            pipeline.vpTrans = shadow.transforms[face];
            pipeline.add(patch);
            const vtrz::VectorizerGrad &shadowVect = shadow.vects[face];
            const uint16_t *meshIds = shadow.meshIds[face].data();
            const uint32_t *primIds = shadow.primIds[face].data();
            const vtrz::DVector3 *worldPositions = shadow.worldPositions[face].data();
            const vtrz::dfloat *wInvs = shadow.wInvs[face].data();
            for (uint32_t i = 0; i < (uint32_t)pipeline.positions.size() / 3; ++i) {
                vtrz::Tri3Grad tri = {
                    pipeline.positions[3 * i],
                    pipeline.positions[3 * i + 1],
                    pipeline.positions[3 * i + 2]
                };
                visibleCoverage += shadowVect.triangleCoverageQuery(
                    tri, patch.meshId, patch.avgItsGrad.its.primIndex,
                    meshIds, primIds, worldPositions, wInvs, queryTransform);
            }
        }

        visibleCoverage.value = math::clamp(visibleCoverage.value, 0.0f, patch.coverage.value);
        constexpr Float kPenumbraThreshold = 1e-2f;
        if (visibleCoverage.value <= kPenumbraThreshold || visibleCoverage.value + kPenumbraThreshold >= patch.coverage.value) {
            visibleCoverage.zeroGrad();
        }
        Lpatch *= visibleCoverage;
        Lsum += Lpatch;
    }

    Lsum /= totalCoverage;
    return Lsum;
}


Spectrum VDirectADIntegrator::Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const
{
    Log(EError, "VtrzSpatialGradientIntegrator doesn't really have a meaningful Li() method. This is likely a bug.");
    return Spectrum(0.0);
}

Spectrum VDirectADIntegrator::E(const Scene *scene, const Intersection &its,
    const Medium *medium, Sampler *sampler, int nSamples, bool includeIndirect) const
{
    Log(EError, "Not implemented yet, but should?");
    return Spectrum(0.0);
}

//////////////////////////////////////////////////////////////////////////
MTS_IMPLEMENT_CLASS_S(VDirectADIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(VDirectADIntegrator, "Vectorizer gradient integrator");
MTS_NAMESPACE_END