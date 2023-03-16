#pragma once

#include <mitsuba/render/scene.h>
#include <vectorizer/vectorizer.h>
#include <vector>

MTS_NAMESPACE_BEGIN

// https://math.stackexchange.com/questions/859454/maximum-number-of-vertices-in-intersection-of-triangle-with-box/
static constexpr uint32_t kMaxClipVertexCount = 9;
static constexpr uint32_t kClipPlaneCount = 5;

template <typename T, typename U>
constexpr U lerp(T t, U v1, U v2) {
    return ((T)1 - t) * v1 + t * v2;
}

constexpr Float sgn(Float x) {
    return x > Float(0.0) ? Float(1.0) : x < Float(0.0) ? Float(-1.0) : Float(0.0);
}

inline vtrz::Vector3 toVec3(const Point &p) {
    return vtrz::Vector3(p.x, p.y, p.z);
}

inline vtrz::Vector3 toVec3(const Vector &v) {
    return vtrz::Vector3(v.x, v.y, v.z);
}

inline vtrz::Vector3 toVec3(const Normal &n) {
    return vtrz::Vector3(n.x, n.y, n.z);
}

inline Point toPoint(const vtrz::Vector3 &v) {
    return Point(v.x, v.y, v.z);
}

inline Vector toVector(const vtrz::Vector3 &v) {
    return Vector(v.x, v.y, v.z);
}

inline Normal toNormal(const vtrz::Vector3 &v) {
    return Normal(v.x, v.y, v.z);
}

inline vtrz::Matrix4x4 castMatrix4x4(const Matrix4x4 &m) {
    vtrz::Matrix4x4 ret;
    for (uint32_t i = 0; i < 4; ++i) {
        for (uint32_t j = 0; j < 4; ++j) {
            ret.arr[i][j] = m(i, j);
        }
    }
    return ret;
}

inline vtrz::DVector4 mul(const Matrix4x4 &m, const vtrz::DVector4 &v) {
    vtrz::DVector4 ret;
    for (uint32_t i = 0; i < 4; ++i)
        for (uint32_t j = 0; j < 4; ++j)
            ret[i] += m(i, j) * v[j];
    return ret;
}

inline vtrz::DMatrix4x4 fromFrame(const Frame &frame) {
    return vtrz::DMatrix4x4(
        frame.s.x, frame.t.x, frame.n.x, 0,
        frame.s.y, frame.t.y, frame.n.y, 0,
        frame.s.z, frame.t.z, frame.n.z, 0,
        0, 0, 0, 1
    );
}

struct DFrame
{
    vtrz::DVector3 s, t, n;

    DFrame() = default;
    DFrame(const vtrz::DVector3 &s, const vtrz::DVector3 &t, const vtrz::DVector3 &n)
        : s(s), t(t), n(n) { }

    inline vtrz::DVector3 toLocal(const vtrz::DVector3 &v) const {
        return vtrz::DVector3(vtrz::dot(v, s), vtrz::dot(v, t), vtrz::dot(v, n));
    }

    inline vtrz::DVector3 toWorld(const vtrz::DVector3 &v) const {
        return s * v.x + t * v.y + n * v.z;
    }
};

struct IntersectionGrad
{
    Intersection its;

    vtrz::DVector3 dp;
    vtrz::DVector2 duv;
    DFrame dShFrame;
};

struct IntersectionPatch
{
    std::vector<Point> positions;
    std::vector<Normal> normals;
    Intersection avgIts;
    uint16_t meshId;
    float coverage;
};

struct IntersectionPatchGrad
{
    std::vector<vtrz::DVector3> positions;
    std::vector<vtrz::DVector3> normals;
    IntersectionGrad avgItsGrad;
    uint16_t meshId;
    vtrz::dfloat coverage;
};

void dumpVectorizerInput(
    const Scene *scene, const Point2i &pixel,
    const AABB2 &bound,
    const std::vector<Point> &vertexBuffer,
    const std::vector<uint32_t> *indexBuffer = nullptr,
    const std::vector<uint32_t> *queryTriIndices = nullptr,
    const std::vector<vtrz::DVector2> *vertexGradBuffer = nullptr);

// Haines, Eric, and Tomas Akenine-Möller, eds. Ray Tracing Gems:
// High-Quality and Real-Time Rendering with DXR and Other APIs. Apress, 2019.
// Chapter 6.
inline int floatAsInt(float x) {
    return *reinterpret_cast<int *>(&x);
}

inline Float intAsFloat(int x) {
    return *reinterpret_cast<Float *>(&x);
}

inline Point offsetRayOrigin(const Point &p, const Vector &d, const Vector &n)
{
    constexpr Float origin = 1.0f / 32.0f;
    constexpr Float floatScale = 1.0f / 65536.0f;
    constexpr Float intScale = 256.0f;

    Vector nf = dot(n, d) >= 0.0 ? n : -n;

    int ofi[3] = { (int)(intScale * nf.x), (int)(intScale * nf.y), (int)(intScale * nf.z) };

    Vector pi(
        intAsFloat(floatAsInt(p.x) + ((p.x < 0) ? -ofi[0] : ofi[0])),
        intAsFloat(floatAsInt(p.y) + ((p.y < 0) ? -ofi[1] : ofi[1])),
        intAsFloat(floatAsInt(p.z) + ((p.z < 0) ? -ofi[2] : ofi[2])));

    return Point(
        std::abs(p.x) < origin ? p.x + floatScale * nf.x : pi.x,
        std::abs(p.y) < origin ? p.y + floatScale * nf.y : pi.y,
        std::abs(p.z) < origin ? p.z + floatScale * nf.z : pi.z);
}


inline Matrix4x4 revInfPerspective(float near, float vFov, float aspect, const AABB2 &ndcBound) {
    SAssert(near > 0.0f && vFov > 0.0f && vFov < M_PI && aspect > 0.0f);
    // Do reverse infinite projection for better depth precision.
    Vector2 exts = ndcBound.getExtents();
    Point2 offset((ndcBound.max.x + ndcBound.min.x) / exts.x, (ndcBound.max.y + ndcBound.min.y) / exts.y);
    Vector2 scale(2.0f / exts.x, 2.0f / exts.y);
    float cotHalfVFov = 1.0f / std::tan(vFov * 0.5f);
    return Matrix4x4(
        scale.x * cotHalfVFov / aspect, 0.0f, offset.x, 0.0f,
        0.0f, scale.y * cotHalfVFov, offset.y, 0.0f,
        0.0f, 0.0f, 0.0f, near,
        0.0f, 0.0f, -1.0f, 0.0f
    );
}

inline Matrix4x4 revInfPerspective2(float near, float tanHalfHFov, float tanHalfVFov, const AABB2 &ndcBound) {
    SAssert(near > 0.0f && tanHalfHFov > 0.0f && tanHalfVFov > 0.0f);
    // Do reverse infinite projection for better depth precision.
    Vector2 exts = ndcBound.getExtents();
    Point2 offset((ndcBound.max.x + ndcBound.min.x) / exts.x, (ndcBound.max.y + ndcBound.min.y) / exts.y);
    Vector2 scale(2.0f / exts.x, 2.0f / exts.y);
    return Matrix4x4(
        scale.x / tanHalfHFov, 0.0f, offset.x, 0.0f,
        0.0f, scale.y / tanHalfVFov, offset.y, 0.0f,
        0.0f, 0.0f, 0.0f, near,
        0.0f, 0.0f, -1.0f, 0.0f
    );
}

inline Matrix4x4 glPerspective(float near, float far, float vFov, float aspect, const AABB2 &ndcBound) {
    SAssert(near > 0.0f && far > near && vFov > 0.0f && vFov < M_PI && aspect > 0.0f);
    Vector2 exts = ndcBound.getExtents();
    Point2 offset((ndcBound.max.x + ndcBound.min.x) / exts.x, (ndcBound.max.y + ndcBound.min.y) / exts.y);
    Vector2 scale(2.0f / exts.x, 2.0f / exts.y);
    float cotHalfVFov = 1.0f / std::tan(vFov * 0.5f);
    Float recip = 1.0f / (near - far);
    return Matrix4x4(
        scale.x * cotHalfVFov / aspect, 0.0f, offset.x, 0.0f,
        0.0f, scale.y * cotHalfVFov, offset.y, 0.0f,
        0.0f, 0.0f, (far + near) * recip, 2 * far * near * recip,
        0.0f, 0.0f, -1.0f, 0.0f
    );
}

struct VertexAttribute
{
    Point worldPos;
    Normal normal;
    Point2 texcoord;
};

struct Beam
{
    Transform viewTrans;
    Transform projTrans;
    Frustum frustum;
};

struct ViewInfo
{
    ViewInfo(const Sensor &sensor);
    Beam createPrimaryBeam(const Point2i &pixel, bool revInfProj = true) const;

    const PerspectiveCamera &persp;
    Transform viewTrans;
    Point sensorPos;
    Vector2 resolution;
    Float nearClip;
    Float vFov;
    Float aspect;
};

constexpr Float kBeamOriginEps(0.0001);
constexpr Float kSecondaryNearClip(0.0001);
constexpr Float kMaxBeamCosHalfAngle(0.9995);
bool createSecondaryBeam(const Intersection &its, const Emitter &emitter, Beam &beam);
bool createSecondaryBeam(const IntersectionPatch &patch, const Point &lightPos, Beam &beam);
Beam createSecondaryBeamOblique(const Point &origin, const Point *patch, uint32_t vertCount);

struct BeamGrad
{
    vtrz::DMatrix4x4 viewTrans;
    vtrz::Matrix4x4 projTrans;
    Frustum frustum;
};

bool createSecondaryBeamGrad(const IntersectionGrad &itsGrad, const Emitter &emitter, BeamGrad &beam);

inline bool backfaceCull(const Point &apex, const Point &p0, const Point &p1, const Point &p2)
{
    Vector n = (cross(p1 - p0, p2 - p1));
    Vector d = apex - p0;
    // Enable backface culling.
    if (dot(d, n) <= 0.0f) {
        return true;
    }

    // Avoid anything behind or (almost) self-intersecting or (almost) parallel.
    n = normalize(n);
    constexpr Float kNearThreshold(1e-6); // Should this be consistent with near clipping plane?
    if (dot(d, n) <= kNearThreshold) {
        return true;
    }
    return false;
}

enum ClipFlag
{
    ELeft = 1 << 0,
    ERight = 1 << 1,
    EBottom = 1 << 2,
    ETop = 1 << 3,
    ENear = 1 << 4,
};

struct BoxOccluder
{
    vtrz::AABB2 occluder;
    float depth;
    bool valid = false;
};

inline BoxOccluder makeBoxOccluder(const AABB &aabb, const Point &apex, const Transform &vpTrans)
{
    BoxOccluder ret;
    // TODO: DepthFunc.
    ret.depth = 0.0f;
    const Matrix4x4 &mat = vpTrans.getMatrix();
    for (uint8_t i = 0; i < 8; ++i) {
        Vector4 p(
            i & 1 ? aabb.max.x : aabb.min.x,
            i & 2 ? aabb.max.y : aabb.min.y,
            i & 4 ? aabb.max.z : aabb.min.z, 1.0f);
        p = mat * p;
        if (p.z >= p.w) {
            return ret;
        }
        float invW = 1.0f / p.w;
        p.x *= invW;
        p.y *= invW;
        p.z *= invW;
        ret.occluder.min.x = std::min(ret.occluder.min.x, p.x);
        ret.occluder.max.x = std::max(ret.occluder.max.x, p.x);
        ret.occluder.min.y = std::min(ret.occluder.min.y, p.y);
        ret.occluder.max.y = std::max(ret.occluder.max.y, p.y);
        ret.depth = std::max(ret.depth, p.z);
    }
    ret.occluder.min.x = std::max(ret.occluder.min.x, -1.0f);
    ret.occluder.max.x = std::min(ret.occluder.max.x, 1.0f);
    ret.occluder.min.y = std::max(ret.occluder.min.y, -1.0f);
    ret.occluder.max.y = std::min(ret.occluder.max.y, 1.0f);
    ret.depth = std::min(ret.depth, 1.0f);
    ret.valid = true;
    return ret;
}

MTS_NAMESPACE_END