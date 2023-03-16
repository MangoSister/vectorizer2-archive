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

class PrimaryVisibilityAD final : public SamplingIntegrator
{
public:
    MTS_DECLARE_CLASS();

    PrimaryVisibilityAD(const Properties &props);

    PrimaryVisibilityAD(Stream *stream, InstanceManager *manager);

    bool preprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID) final;

    void postprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
        int sceneResID, int sensorResID, int samplerResID) final;

    void renderBlock(const Scene *scene, const Sensor *sensor,
        Sampler *sampler, ImageBlock *block, const bool &stop,
        const std::vector< TPoint2<uint8_t> > &points) const final;

    Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const final { return Spectrum(0.0); }

    void writePixel(const Point2i &pixel, const vtrz::DVector3 &L, ImageBlock &block) const;

private:
    uint32_t totalGradDim = 0;
    SceneAD sceneAD;
    std::vector<std::string> transformADInfo;
    mutable ref_vector<Bitmap> gradImages;

    BVH bvh;
};

PrimaryVisibilityAD::PrimaryVisibilityAD(const Properties &props) :
    SamplingIntegrator(props)
{
    for (uint32_t i = 0; ; ++i) {
        std::string key = "transformAD" + std::to_string(i);
        if (props.hasProperty(key)) {
            transformADInfo.push_back(props.getString(key));
        } else {
            break;
        }
    }
}

PrimaryVisibilityAD::PrimaryVisibilityAD(Stream *stream, InstanceManager *manager) :
    SamplingIntegrator(stream, manager)
{

}

static vtrz::DMatrix4x4 parseTransformDerivative(std::string &token, uint32_t gradDim) {
    // Don't care about values.
    SAssert(token.size() > 0 && (token[0] == 't' || token[0] == 'r' || token[0] == 's'));
    if (token[0] == 't') {
        // translation
        SAssert(token[1] == 'x' || token[1] == 'y' || token[1] == 'z');
        SAssert(token[2] == '+' || token[2] == '-');
        if (token[1] == 'x' && token[2] == '+') return vtrz::makeTranslate(vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(), vtrz::dfloat());
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
        if (token[1] == 'x' && token[2] == '+') return vtrz::makeScale(vtrz::dfloat(0.0f, gradDim), vtrz::dfloat(), vtrz::dfloat());
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

bool PrimaryVisibilityAD::preprocess(
    const Scene *scene, RenderQueue *queue, const RenderJob *job,
    int sceneResID, int sensorResID, int samplerResID)
{
    totalGradDim = 0;
    sceneAD.indexMap.clear();
    sceneAD.meshADs.clear();

    auto &meshes = scene->getMeshes();
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

    bvh.build(*scene);

    gradImages.resize(totalGradDim);
    for (uint32_t i = 0; i < totalGradDim; ++i) {
        gradImages[i] = new Bitmap(Bitmap::EPixelFormat::ERGB, Bitmap::EComponentFormat::EFloat32,
            scene->getFilm()->getCropSize());
        gradImages[i]->clear();
    }

    return true;
}

void PrimaryVisibilityAD::postprocess(
    const Scene *scene, RenderQueue *queue, const RenderJob *job,
    int sceneResID, int sensorResID, int samplerResID)
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

void PrimaryVisibilityAD::renderBlock(
    const Scene *scene, const Sensor *sensor, Sampler *sampler, ImageBlock *block,
    const bool &stop, const std::vector< TPoint2<uint8_t> > &points) const
{
    block->clear();

    SAssert(sensor->getClass()->derivesFrom(MTS_CLASS(PerspectiveCamera)));
    ViewInfo view(*sensor);

    FrustumBVHIsectResult isects;
    PrimaryPipelineAD primary;
    SecondaryPipelineAD secondary;
    vtrz::VectorizerGrad vect;

    for (auto pt : points) {
        if (stop) break;

        Point2i pixel = Point2i(pt) + Vector2i(block->getOffset());
        Beam beam = view.createPrimaryBeam(pixel);
        isects.clear();
        bvh.frustumIntersect(beam.frustum, isects, FrustumAABBOption::eExact);
        primary.clear();
        primary.vpTrans = vtrz::DMatrix4x4(castMatrix4x4((beam.projTrans * beam.viewTrans).getMatrix()));
        primary.apex = beam.frustum.apex;
        primary.run(*scene, bvh, sceneAD, isects);

        vect.reset(vtrz::AABB2(vtrz::Vector2(-1.0f), vtrz::Vector2(1.0f)), vtrz::DepthFunc::eGreater);
        vect.build(
            primary.positions.data(), (uint32_t)primary.positions.size(),
            nullptr, 0);

        vtrz::DVector3 L(0.0f);

        uint32_t leafCount = vect.leafCount();
        for (uint32_t leafIdx = 0; leafIdx < leafCount; ++leafIdx) {
            vtrz::LeafRegionGrad region = vect.getLeaf(leafIdx);
            vtrz::dfloat coverage(0.0f);
            for (uint32_t v = 0; v < region.vertCount; ++v) {
                vtrz::DVector2 vert = vtrz::cast(region.poly[v]);
                vtrz::DVector2 vnext = vtrz::cast(region.poly[(v + 1) % region.vertCount]);
                coverage += vtrz::cross(vert, vnext);
            }

            uint32_t primIndex = primary.primIds[region.triId];
            size_t h = std::hash<uint32_t>()(primIndex);
            vtrz::DVector3 color(Float(h % 256) / 255.0f, Float((h >> 8) % 256) / 255.0f, Float((h >> 16) % 256) / 255.0f);
            L += color * coverage;
        }

        L *= 0.125f;

        writePixel(pixel, L, *block);
    }
}

void PrimaryVisibilityAD::writePixel(const Point2i &pixel, const vtrz::DVector3 &L, ImageBlock &block) const
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

//////////////////////////////////////////////////////////////////////////
MTS_IMPLEMENT_CLASS_S(PrimaryVisibilityAD, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(PrimaryVisibilityAD, "Primary Visibility AD");
MTS_NAMESPACE_END