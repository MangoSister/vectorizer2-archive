#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/texture.h>
#include <mitsuba/hw/basicshader.h>
#include <mitsuba/core/warp.h>
#include <mitsuba/render/diffutil.h>
#include "disneylitedata.h"

// !New: Vectorizer
#include <vectorizer/vectorizer.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

MTS_NAMESPACE_BEGIN

using namespace disneylite;

template <typename T, typename U>
constexpr U lerp(T t, U v1, U v2) {
    return ((T)1 - t) * v1 + t * v2;
}

// Schlick Fresnel approximation for metals (complex IOR):
// Lazaniy, Istvan, and Laszlo Szirmay-Kalos. "Fresnel term approximations for metals." (2005).

class DisneyLite : public BSDF {
public:
    DisneyLite(const Properties &props)
        : BSDF(props) {
        /* For better compatibility with other models, support both
           'reflectance' and 'diffuseReflectance' as parameter names */
        m_baseColor = new ConstantSpectrumTexture(props.getSpectrum("baseColor", Spectrum(1.0f)));
        m_metallic = new ConstantFloatTexture(props.getFloat("metallic", Float(1.0f)));
        m_roughness = new ConstantFloatTexture(props.getFloat("roughness", Float(0.5f)));
        m_specular = new ConstantFloatTexture(props.getFloat("specular", Float(0.5f)));

        m_complexIOR = props.getBoolean("complexIOR", false);
        m_eta = new ConstantSpectrumTexture(props.getSpectrum("eta", Spectrum(1.0f)));
        m_k = new ConstantSpectrumTexture(props.getSpectrum("k", Spectrum(0.0f)));

        m_grad = props.getString("grad", std::string());

        configure();
    }

    DisneyLite(Stream *stream, InstanceManager *manager)
        : BSDF(stream, manager) {
        m_baseColor = static_cast<Texture *>(manager->getInstance(stream));
        m_metallic = static_cast<Texture *>(manager->getInstance(stream));
        m_roughness = static_cast<Texture *>(manager->getInstance(stream));
        m_specular = static_cast<Texture *>(manager->getInstance(stream));

        m_complexIOR = stream->readBool();
        m_eta = static_cast<Texture *>(manager->getInstance(stream));
        m_k = static_cast<Texture *>(manager->getInstance(stream));

        m_grad = stream->readString();

        configure();
    }

    void configure() {
        BSDF::configure();

        // All of them should have values within [0, 1].
        m_baseColor = ensureEnergyConservation(m_baseColor, "baseColor", 1.0f);
        m_metallic = ensureEnergyConservation(m_metallic, "metallic", 1.0f);
        m_roughness = ensureEnergyConservation(m_roughness, "roughness", 1.0f);
        m_specular = ensureEnergyConservation(m_specular, "specular", 1.0f);

        bool spatialVarying =
            !m_baseColor->isConstant() || !m_metallic->isConstant() ||
            !m_roughness->isConstant() || !m_specular->isConstant();
        bool delta = m_roughness->getMaximum().max() < kMinRoughness;
        bool nodiffuse = m_metallic->getMinimum().min() > 0.99f;

        m_components.clear();
        m_components.push_back(
            (delta ? EDiffuseReflection : EGlossyReflection) |
            (spatialVarying ? ESpatiallyVarying : 0) |
            EFrontSide);
        if (!nodiffuse) {
            m_components.push_back(
                EDiffuseReflection |
                (spatialVarying ? ESpatiallyVarying : 0) |
                EFrontSide);
        }

        m_usesRayDifferentials =
            m_baseColor->usesRayDifferentials() ||
            m_metallic->usesRayDifferentials() ||
            m_roughness->usesRayDifferentials() ||
            m_specular->usesRayDifferentials();

        {
            LockGuard guard(m_globalLookupTableMutex);
            if (m_globalLookupTable == nullptr) {
                m_globalLookupTable = new LookupTable();
            }
            m_lookupTable = m_globalLookupTable;
        }
    }

    Spectrum getDiffuseReflectance(const Intersection &its) const {
        NotImplementedError("getDiffuseReflectance");
    }

    Spectrum eval(const BSDFSamplingRecord &bRec, EMeasure measure) const {
        // TODO: This is now the code from diffuse.cpp. Actually implement this?
        if (!(bRec.typeMask & EDiffuseReflection) || measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Spectrum(0.0f);

        return m_baseColor->eval(bRec.its)
            * (INV_PI * Frame::cosTheta(bRec.wo));
    }

    Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure) const {
        // TODO: This is now the code from diffuse.cpp. Actually implement this?
        if (!(bRec.typeMask & EDiffuseReflection) || measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

        return warp::squareToCosineHemispherePdf(bRec.wo);
    }

    Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &sample) const {
        // TODO: This is now the code from diffuse.cpp. Actually implement this?
        if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
            return Spectrum(0.0f);

        bRec.wo = warp::squareToCosineHemisphere(sample);
        bRec.eta = 1.0f;
        bRec.sampledComponent = 0;
        bRec.sampledType = EDiffuseReflection;
        return m_baseColor->eval(bRec.its);
    }

    Spectrum sample(BSDFSamplingRecord &bRec, Float &pdf, const Point2 &sample) const {
        // TODO: This is now the code from diffuse.cpp. Actually implement this?
        if (!(bRec.typeMask & EDiffuseReflection) || Frame::cosTheta(bRec.wi) <= 0)
            return Spectrum(0.0f);

        bRec.wo = warp::squareToCosineHemisphere(sample);
        bRec.eta = 1.0f;
        bRec.sampledComponent = 0;
        bRec.sampledType = EDiffuseReflection;
        pdf = warp::squareToCosineHemispherePdf(bRec.wo);
        return m_baseColor->eval(bRec.its);
    }

     Spectrum evalPolyIntegral(const BSDFPolyIntegralContext &context) const {
         Spectrum baseColor = m_baseColor->eval(context.its);
         Float metallic = m_metallic->eval(context.its)[0];
         Float roughness = m_roughness->eval(context.its)[0];
         roughness = math::clamp(roughness, kMinRoughness, Float(1.0));
         Float specular = m_specular->eval(context.its)[0];



         const Vector &T1 = context.its.shFrame.s;
         const Vector &T2 = context.its.shFrame.t;
         const Vector &N = (Vector)context.its.shFrame.n;
         Matrix3x3 basis;
         basis.m[0][0] = T1.x;   basis.m[0][1] = T1.y;   basis.m[0][2] = T1.z;
         basis.m[1][0] = T2.x;   basis.m[1][1] = T2.y;   basis.m[1][2] = T2.z;
         basis.m[2][0] = N.x;    basis.m[2][1] = N.y;    basis.m[2][2] = N.z;

         LookupEntry lookup = m_lookupTable->eval(roughness, context.NdotV);
         Matrix3x3 ltcInvMtx;
         ltcInvMtx.m[0][0] = lookup.m00;  ltcInvMtx.m[0][1] = 0;       ltcInvMtx.m[0][2] = lookup.m20;
         ltcInvMtx.m[1][0] = 0;           ltcInvMtx.m[1][1] = 1;       ltcInvMtx.m[1][2] = 0;
         ltcInvMtx.m[2][0] = lookup.m02;  ltcInvMtx.m[2][1] = 0;       ltcInvMtx.m[2][2] = lookup.m22;

         ltcInvMtx = ltcInvMtx * basis;

         Spectrum specularFactor;
         if (!m_complexIOR) {
             Spectrum R0 = lerp(metallic, Spectrum(specular * kMaxDielectricR0), baseColor);
             specularFactor = R0 * lookup.nd + (Spectrum(1.0) - R0)  * lookup.fd;
         } else {
             Spectrum eta = m_eta->eval(context.its);
             Spectrum k = m_k->eval(context.its);

             Spectrum A = (eta - Spectrum(1.0)) * (eta - Spectrum(1.0));
             Spectrum B = 4.0 * eta * 1.0;
             Spectrum C = k * k;
             Spectrum D = (eta + Spectrum(1.0)) * (eta + Spectrum(1.0));
             Spectrum P1 = (A + C) / (D + C);
             Spectrum P2 = B / (D + C);
             specularFactor = P1 * lookup.nd + P2 * lookup.fd;
         }
         Spectrum diffuseFactor = baseColor * INV_PI * (Float(1.0) - metallic);

         NeumaierSum diffuseIntegral;
         NeumaierSum specularIntegral;

         uint32_t leafCount = context.vect.leafCount();

         std::vector<Vector> localPoly;
         for (uint32_t i = 0; i < leafCount; ++i) {
             vtrz::LeafRegion region = context.vect.getLeaf(i);
             localPoly.resize(region.vertCount);
             if (!diffuseFactor.isZero()) {
                 for (uint32_t v = 0; v < region.vertCount; ++v) {
                     vtrz::Vector2 p = vtrz::cast(region.poly[v]);
                     localPoly[v] = context.transformToLocalSphere(Vector2(p.x, p.y));
                 }
                 diffuseIntegral += sphericalPolyFormFactor(localPoly.data(), region.vertCount);
             }
             if (!specularFactor.isZero()) {
                 for (uint32_t v = 0; v < region.vertCount; ++v) {
                     vtrz::Vector2 p = vtrz::cast(region.poly[v]);
                     localPoly[v] = context.transformToWorldDir(Vector2(p.x, p.y));
                     localPoly[v] = ltcInvMtx * localPoly[v];
                     localPoly[v] = normalize(localPoly[v]);
                 }

                 Vector vsum = sphericalPolyVecFormFactor(localPoly.data(), region.vertCount);
                 Float len = vsum.length();
                 if (len > 0.0) {
                     // if (behind) z = vsum.z / len; // Need to check which side if we want to support double-sided lights.
                     Float z = -vsum.z / len;
                     Float elevation = z * 0.5f + 0.5f;
                     Float scale = m_lookupTable->evalSphereFormFactor(elevation, len);
                     specularIntegral += scale * len;
                 }
             }
         }

         Spectrum bsdfIntSpec = (Float)specularIntegral * specularFactor;
         Spectrum bsdfIntDiffuse = (Float)diffuseIntegral * diffuseFactor;
         return bsdfIntSpec + bsdfIntDiffuse;
     }

     void evalPolyIntegralGrad(const BSDFPolyIntegralGradContext &context, vtrz::DVector3 &bsdfIntAD) const {
         vtrz::DVector3 baseColor;
         m_baseColor->eval(context.its).toLinearRGB(baseColor.x.value, baseColor.y.value, baseColor.z.value);
         vtrz::dfloat metallic(m_metallic->eval(context.its)[0]);
         vtrz::dfloat roughness(m_roughness->eval(context.its)[0]);
         vtrz::dfloat specular(m_specular->eval(context.its)[0]);

         roughness.grad = m_roughnessGrad.grad;

         const vtrz::DVector3 &T1 = context.shFrameSGrad;
         const vtrz::DVector3 &T2 = context.shFrameTGrad;
         const vtrz::DVector3 &N = context.shFrameNGrad;
         vtrz::DMatrix3x3 basis;
         basis.arr[0][0] = T1.x;   basis.arr[0][1] = T1.y;   basis.arr[0][2] = T1.z;
         basis.arr[1][0] = T2.x;   basis.arr[1][1] = T2.y;   basis.arr[1][2] = T2.z;
         basis.arr[2][0] = N.x;    basis.arr[2][1] = N.y;    basis.arr[2][2] = N.z;

         LookupEntryAD lookup = m_lookupTable->evalAD(roughness, context.NdotV);

         vtrz::DMatrix3x3 ltcInvMtx;
         ltcInvMtx.arr[0][0] = lookup.m00;  ltcInvMtx.arr[0][1] = 0;       ltcInvMtx.arr[0][2] = lookup.m20;
         ltcInvMtx.arr[1][0] = 0;           ltcInvMtx.arr[1][1] = 1;       ltcInvMtx.arr[1][2] = 0;
         ltcInvMtx.arr[2][0] = lookup.m02;  ltcInvMtx.arr[2][1] = 0;       ltcInvMtx.arr[2][2] = lookup.m22;

         ltcInvMtx = ltcInvMtx * basis;

         vtrz::DVector3 specularFactor;
         if (!m_complexIOR) {
             vtrz::DVector3 R0 = lerp(metallic, vtrz::DVector3(specular * kMaxDielectricR0), baseColor);
             specularFactor = R0 * lookup.nd + (vtrz::DVector3(1.0) - R0)  * lookup.fd;
         } else {
             vtrz::DVector3 eta;
             m_eta->eval(context.its).toLinearRGB(eta.x.value, eta.y.value, eta.z.value);
             vtrz::DVector3 k;
             m_k->eval(context.its).toLinearRGB(k.x.value, k.y.value, k.z.value);

             vtrz::DVector3 A = (eta - 1.0) * (eta - 1.0);
             vtrz::DVector3 B = 4.0 * eta * 1.0;
             vtrz::DVector3 C = k * k;
             vtrz::DVector3 D = (eta + 1.0) * (eta + 1.0);
             vtrz::DVector3 P1 = (A + C) / (D + C);
             vtrz::DVector3 P2 = B / (D + C);
             specularFactor = P1 * lookup.nd + P2 * lookup.fd;
         }
         vtrz::DVector3 diffuseFactor = baseColor * INV_PI * (vtrz::dfloat(1.0) - metallic);

         vtrz::dfloat diffuseIntegral;
         vtrz::dfloat specularIntegral;

         uint32_t leafCount = context.vect.leafCount();

         std::vector<vtrz::DVector3> localPoly;
         for (uint32_t i = 0; i < leafCount; ++i) {
             vtrz::LeafRegionGrad region = context.vect.getLeaf(i);
             localPoly.resize(region.vertCount);

             for (uint32_t v = 0; v < region.vertCount; ++v) {
                 vtrz::DVector2 p = vtrz::cast(region.poly[v]);
                 context.transformToLocalSphere(p, localPoly[v]);
             }
             vtrz::dfloat di;
             sphericalPolyFormFactor(localPoly.data(), region.vertCount, di);
             diffuseIntegral += di;

             for (uint32_t v = 0; v < region.vertCount; ++v) {
                 vtrz::DVector2 p = vtrz::cast(region.poly[v]);
                 context.transformToWorldDir(p, localPoly[v]);
                 localPoly[v] = ltcInvMtx * localPoly[v];
                 localPoly[v] = vtrz::normalize(localPoly[v]);
             }

             vtrz::DVector3 vsum;
             sphericalPolyVecFormFactor(localPoly.data(), region.vertCount, vsum);
             vtrz::dfloat len = vtrz::length(vsum);
             if (len.value > 0.0) {
                 // if (behind) z = vsum.z / len; // Need to check which side if we want to support double-sided lights.
                 vtrz::dfloat z = -vsum.z / len;
                 vtrz::dfloat elevation = z * 0.5f + 0.5f;
                 vtrz::dfloat scale = m_lookupTable->evalSphereFormFactorAD(elevation, len);
                 specularIntegral += scale * len;
             }
         }

         vtrz::DVector3 bsdfIntSpec = specularIntegral * specularFactor;
         vtrz::DVector3 bsdfIntDiffuse = diffuseIntegral * diffuseFactor;
         bsdfIntAD = bsdfIntSpec + bsdfIntDiffuse;
     }

     uint32_t gradDim() const {
         if (m_grad.empty()) {
             return 0;
         }
         using namespace boost::algorithm;
         std::vector<std::string> tokens;
         split(tokens, m_grad, boost::is_any_of("|"));
         return (uint32_t)tokens.size();
     }

     void applyGrad(uint32_t globalChannelOffset) {
         m_roughnessGrad = vtrz::dfloat(0.0f);

         using namespace boost::algorithm;
         std::vector<std::string> tokens;
         split(tokens, m_grad, boost::is_any_of("|"));
         for (const auto &token : tokens) {
             if (token.empty()) continue;
             SAssert(token[0] == 'r');
             if (token[0] == 'r') {
                 SAssert(token.size() == 2 && (token[1] == '+' || token[1] == '-'));
                 if (token[1] == '+') m_roughnessGrad = vtrz::dfloat(0.0f, globalChannelOffset);
                 else if (token[1] == '-') m_roughnessGrad = -vtrz::dfloat(0.0f, globalChannelOffset);
             }
         }
     }

    void addChild(const std::string &name, ConfigurableObject *child) {
        if (child->getClass()->derivesFrom(MTS_CLASS(Texture))
            && (name == "baseColor")) {
            m_baseColor = static_cast<Texture *>(child);
        } else if (child->getClass()->derivesFrom(MTS_CLASS(Texture))
            && (name == "metallic")) {
            m_metallic = static_cast<Texture *>(child);
        } else if (child->getClass()->derivesFrom(MTS_CLASS(Texture))
            && (name == "roughness")) {
            m_roughness = static_cast<Texture *>(child);
        } else if (child->getClass()->derivesFrom(MTS_CLASS(Texture))
            && (name == "specular")) {
            m_specular = static_cast<Texture *>(child);
        } else {
            BSDF::addChild(name, child);
        }
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        BSDF::serialize(stream, manager);

        manager->serialize(stream, m_baseColor.get());
        manager->serialize(stream, m_metallic.get());
        manager->serialize(stream, m_roughness.get());
        manager->serialize(stream, m_metallic.get());
    }

    Float getRoughness(const Intersection &its, int component) const {
        Assert(component == 0 || component == 1);

        if (component == 0)
            return m_roughness->eval(its).average();
        else
            return std::numeric_limits<Float>::infinity();
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "DisneyLite[" << endl
            << "  id = \"" << getID() << "\"," << endl
            << "  baseColor = " << indent(m_baseColor->toString()) << endl
            << "  metallic = " << indent(m_metallic->toString()) << endl
            << "  roughness = " << indent(m_roughness->toString()) << endl
            << "  specular = " << indent(m_specular->toString()) << endl
            << "]";
        return oss.str();
    }

    Shader *createShader(Renderer *renderer) const;

    MTS_DECLARE_CLASS()
private:
    ref<Texture> m_baseColor;
    ref<Texture> m_metallic;
    ref<Texture> m_roughness;
    ref<Texture> m_specular;

    ref<Texture> m_eta;
    ref<Texture> m_k;
    bool m_complexIOR;

    static constexpr Float kMinRoughness = Float(0.01);
    static constexpr Float kMaxDielectricR0 = Float(0.08);

    ref<const LookupTable> m_lookupTable;

    static ref<const LookupTable> m_globalLookupTable;
    static ref<Mutex> m_globalLookupTableMutex;

    std::string m_grad;
    // ignore values.
    vtrz::dfloat m_roughnessGrad;
};

ref<const LookupTable> DisneyLite::m_globalLookupTable = nullptr;
ref<Mutex> DisneyLite::m_globalLookupTableMutex = new Mutex();

// ================ Hardware shader implementation ================

class DisneyLiteShader : public Shader {
public:
    DisneyLiteShader(Renderer *renderer, const Texture *reflectance)
        : Shader(renderer, EBSDFShader), m_reflectance(reflectance) {
        m_reflectanceShader = renderer->registerShaderForResource(m_reflectance.get());
    }

    bool isComplete() const {
        return m_reflectanceShader.get() != NULL;
    }

    void cleanup(Renderer *renderer) {
        renderer->unregisterShaderForResource(m_reflectance.get());
    }

    void putDependencies(std::vector<Shader *> &deps) {
        deps.push_back(m_reflectanceShader.get());
    }

    void generateCode(std::ostringstream &oss,
            const std::string &evalName,
            const std::vector<std::string> &depNames) const {
        oss << "vec3 " << evalName << "(vec2 uv, vec3 wi, vec3 wo) {" << endl
            << "    if (cosTheta(wi) < 0.0 || cosTheta(wo) < 0.0)" << endl
            << "        return vec3(0.0);" << endl
            << "    return " << depNames[0] << "(uv) * inv_pi * cosTheta(wo);" << endl
            << "}" << endl
            << endl
            << "vec3 " << evalName << "_diffuse(vec2 uv, vec3 wi, vec3 wo) {" << endl
            << "    return " << evalName << "(uv, wi, wo);" << endl
            << "}" << endl;
    }

    MTS_DECLARE_CLASS()
private:
    ref<const Texture> m_reflectance;
    ref<Shader> m_reflectanceShader;
};

Shader *DisneyLite::createShader(Renderer *renderer) const {
    return new DisneyLiteShader(renderer, m_baseColor.get());
}

MTS_IMPLEMENT_CLASS(DisneyLiteShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(DisneyLite, false, BSDF)
MTS_EXPORT_PLUGIN(DisneyLite, "A more practical version of Disney's Principle BRDF, aka the \"meta\" BRDF...")
MTS_NAMESPACE_END
