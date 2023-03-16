#if !defined(__LTC_HELPER_H)
#define __LTC_HELPER_H

#include <mitsuba/mitsuba.h>
#include <mitsuba/render/shape.h>
#include <mitsuba/render/diffutil.h>

namespace vtrz
{
class Vectorizer;
class VectorizerGrad;
struct Matrix4x4;
template <typename T>
struct dreal;
using dfloat = dreal<float>;
struct DVector2;
struct DVector3;
struct DMatrix4x4;
}

MTS_NAMESPACE_BEGIN

// https://en.wikipedia.org/wiki/Kahan_summation_algorithm.
struct NeumaierSum
{
    Float sum = Float(0.0);
    Float correct = Float(0.0);

    void reset() {
        sum = Float(0.0);
        correct = Float(0.0);
    }

    void add(Float item) {
        Float t = sum + item;
        if (std::abs(sum) > std::abs(item)) {
            correct += (sum - t) + item;
        } else {
            correct += (item - t) + sum;
        }
        sum = t;
    }

    Float result() const {
        return sum + correct;
    }

    NeumaierSum &operator+=(Float item) {
        add(item);
        return *this;
    }

    explicit operator Float() const {
        return result();
    }
};

struct MTS_EXPORT_RENDER BSDFEvalGradContext
{
    const vtrz::DVector3 &p;
    const vtrz::DVector3 &wo;
    const vtrz::DVector3 &wi;

    const Intersection &its;

    BSDFEvalGradContext(
        const vtrz::DVector3 &p, const vtrz::DVector3 &wo, const vtrz::DVector3 &wi, const Intersection &its) :
        p(p), wo(wo), wi(wi), its(its) {}
};

struct MTS_EXPORT_RENDER BSDFPolyIntegralContext
{
	const vtrz::Vectorizer &vect;

	const Transform &invProjTrans;
	const Transform &invViewTrans;
	const Intersection &its;
	const Float NdotV;

	BSDFPolyIntegralContext(
		const vtrz::Vectorizer &vect,
		const Transform &invProjTrans,
		const Transform &invViewTrans,
		const Intersection &its,
		const Float NdotV) :
		vect(vect),
		invProjTrans(invProjTrans),
		invViewTrans(invViewTrans),
		its(its),
		NdotV(NdotV) {}

	Vector transformToLocalSphere(const Vector2 &ndcPos) const {
		Vector4 viewDir = invProjTrans.getMatrix() * Vector4(ndcPos.x, ndcPos.y, 1.0, 1.0);
		Vector worldDir = invViewTrans(Vector(viewDir.x, viewDir.y, viewDir.z));
		Vector localDir = normalize(its.shFrame.toLocal(worldDir));
		// Clip to upper hemisphere?
		if (localDir.z < Float(0.0)) {
			localDir.z = Float(0.0);
			localDir = normalize(localDir);
		}
		return localDir;
	}

	Vector transformToWorldDir(const Vector2 &ndcPos) const {
		Vector4 viewDir = invProjTrans.getMatrix() * Vector4(ndcPos.x, ndcPos.y, 1.0, 1.0);
		Vector worldDir = invViewTrans(Vector(viewDir.x, viewDir.y, viewDir.z));
		return worldDir;
	}
};

struct MTS_EXPORT_RENDER BSDFPolyIntegralGradContext
{
    const vtrz::VectorizerGrad &vect;

    const vtrz::Matrix4x4 &invProjTrans;
    const vtrz::DMatrix4x4 &invViewTrans;
    const Intersection &its;

    const vtrz::DVector3 &shFrameSGrad;
    const vtrz::DVector3 &shFrameTGrad;
    const vtrz::DVector3 &shFrameNGrad;
    const vtrz::dfloat &NdotV;

    BSDFPolyIntegralGradContext(
        const vtrz::VectorizerGrad &vect,
        const vtrz::Matrix4x4 &invProjTrans,
        const vtrz::DMatrix4x4 &invViewTrans,
        const Intersection &its,
        const vtrz::DVector3 &shFrameSGrad,
        const vtrz::DVector3 &shFrameTGrad,
        const vtrz::DVector3 &shFrameNGrad,
        const vtrz::dfloat &NdotV) :
        vect(vect),
        invProjTrans(invProjTrans),
        invViewTrans(invViewTrans),
        its(its),
        shFrameSGrad(shFrameSGrad),
        shFrameTGrad(shFrameTGrad),
        shFrameNGrad(shFrameNGrad),
        NdotV(NdotV) {}

    void transformToLocalSphere(const vtrz::DVector2 &ndcPos, vtrz::DVector3 &localDir) const;

    void transformToWorldDir(const vtrz::DVector2 &ndcPos, vtrz::DVector3 &worldDir) const;
};

 // Assume vertices are arranged CCW, and vertices are already normalized to unit sphere.
 // Assume normal is (0, 0, 1).
MTS_EXPORT_RENDER Float sphericalPolyFormFactor(const Vector verts[], uint32_t vertCount);
MTS_EXPORT_RENDER void sphericalPolyFormFactor(const vtrz::DVector3 verts[], uint32_t vertCount, vtrz::dfloat &I);

MTS_EXPORT_RENDER Vector3 sphericalPolyVecFormFactor(const Vector verts[], uint32_t vertCount);
MTS_EXPORT_RENDER void sphericalPolyVecFormFactor(const vtrz::DVector3 verts[], uint32_t vertCount, vtrz::DVector3 &V);

MTS_NAMESPACE_END

#endif /* __LTC_HELPER_H */
