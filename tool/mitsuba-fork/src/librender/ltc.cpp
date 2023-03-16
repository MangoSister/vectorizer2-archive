#include <mitsuba/render/ltc.h>
#include <vectorizer/vectorizer.h>

MTS_NAMESPACE_BEGIN

void BSDFPolyIntegralGradContext::transformToLocalSphere(const vtrz::DVector2 &ndcPos, vtrz::DVector3 &localDir) const
{
    vtrz::DVector3 viewDir = (vtrz::DVector3)(invProjTrans * vtrz::DVector4(ndcPos.x, ndcPos.y, 1.0f, 1.0f));
    vtrz::DVector3 worldDir = (vtrz::DVector3)(invViewTrans * vtrz::DVector4(viewDir.x, viewDir.y, viewDir.z, 0.0f));
    localDir = vtrz::DVector3(
        vtrz::dot(shFrameSGrad, worldDir),
        vtrz::dot(shFrameTGrad, worldDir),
        vtrz::dot(shFrameNGrad, worldDir));
    localDir = vtrz::normalize(localDir);
    // vtrz::DVector3 localDir = normalize(context.its.shFrame.toLocal(worldDir));
    // Clip to upper hemisphere?
    if (localDir.z.value < 0.0f) {
        localDir.z.value = 0.0f;
        localDir.setValue(vtrz::normalize(localDir.value())); // normalize value only?
    }
}

void BSDFPolyIntegralGradContext::transformToWorldDir(const vtrz::DVector2 &ndcPos, vtrz::DVector3 &worldDir) const
{
    vtrz::DVector3 viewDir = (vtrz::DVector3)(invProjTrans * vtrz::DVector4(ndcPos.x, ndcPos.y, 1.0, 1.0));
    worldDir = (vtrz::DVector3)(invViewTrans * vtrz::DVector4(viewDir.x, viewDir.y, viewDir.z, 0.0f));
}

Float sphericalPolyFormFactor(const Vector verts[], uint32_t vertCount) {
    Float I(0.0);
    for (uint32_t i = 0; i < vertCount; ++i) {
        const Vector &pi = verts[i];
        const Vector &pj = verts[(i + 1) % vertCount];
        Float theta = math::safe_acos(dot(pi, pj));
        Float z = pi.x * pj.y - pi.y * pj.x;
        I += z * ((theta > 0.001f) ? theta / sin(theta) : 1.0f);
    }

    return abs(I) * 0.5f;
}

void sphericalPolyFormFactor(const vtrz::DVector3 verts[], uint32_t vertCount, vtrz::dfloat &I)
{
    I = vtrz::dfloat(0.0f);
    for (uint32_t i = 0; i < vertCount; ++i) {
        const vtrz::DVector3 &pi = verts[i];
        const vtrz::DVector3 &pj = verts[(i + 1) % vertCount];
        vtrz::dfloat theta = vtrz::acosSafe(vtrz::dot(pi, pj));
        vtrz::dfloat z = pi.x * pj.y - pi.y * pj.x;
        I += z * ((theta.value > 0.001f) ? theta / vtrz::sin(theta) : vtrz::dfloat(1.0f));
    }

    I = vtrz::abs(I) * 0.5f;
}

static inline Vector3 edgeIntegralFit(const Vector3 &v1, const Vector3 &v2) {
    Float x = dot(v1, v2);
    Float y = abs(x);

    Float a = 0.8543985f + (0.4965155f + 0.0145206f * y) * y;
    Float b = 3.4175940f + (4.1616724f + y) * y;
    Float v = a / b;

    Float theta_sintheta = (x > 0.0f) ? v : (0.5f / math::safe_sqrt(1.0f - x * x) - v);

    return cross(v1, v2) * theta_sintheta;
}

static inline vtrz::DVector3 edgeIntegralFit(const vtrz::DVector3 &v1, const vtrz::DVector3 &v2) {
    vtrz::dfloat x = vtrz::dot(v1, v2);
    vtrz::dfloat y = vtrz::abs(x);

    vtrz::dfloat a = 0.8543985f + (0.4965155f + 0.0145206f * y) * y;
    vtrz::dfloat b = 3.4175940f + (4.1616724f + y) * y;
    vtrz::dfloat v = a / b;

    vtrz::dfloat theta_sintheta = (x.value > 0.0f) ? v : (0.5f / vtrz::sqrtSafe(1.0f - x * x) - v);

    return vtrz::cross(v1, v2) * theta_sintheta;
}

Vector3 sphericalPolyVecFormFactor(const Vector3 verts[], uint32_t vertCount) {
    Vector3 V(0.0f);
    for (uint32_t i = 0; i < vertCount; ++i) {
        V += edgeIntegralFit(verts[i], verts[(i + 1) % vertCount]);
    }
    return V;
}

void sphericalPolyVecFormFactor(const vtrz::DVector3 verts[], uint32_t vertCount, vtrz::DVector3 &V)
{
    V = vtrz::DVector3(0.0f);
    for (uint32_t i = 0; i < vertCount; ++i) {
        V += edgeIntegralFit(verts[i], verts[(i + 1) % vertCount]);
    }
}

MTS_NAMESPACE_END