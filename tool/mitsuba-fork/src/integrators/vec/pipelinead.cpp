#include "pipelinead.h"
#include "bvh.h"
#include "scenead.h"

MTS_NAMESPACE_BEGIN

void PrimaryPipelineAD::clear()
{
    positions.clear();
    worldPositions.clear();
    normals.clear();
    wInvs.clear();

    meshIds.clear();
    matIds.clear();
    primIds.clear();
}

template <uint32_t axis, bool side>
static inline bool inside(const vtrz::DVector4 &h) {
    if (side) {
        return h[axis].value <= h.w.value;
    } else {
        return h[axis].value >= -h.w.value;
    }
}

template <uint32_t axis, bool side>
static inline vtrz::dfloat lerpFactor(const vtrz::DVector4 &h1, const vtrz::DVector4 &h2) {
    vtrz::dfloat t;
    if (side) {
        t = (h1.w - h1[axis]) / (h1.w - h1[axis] - h2.w + h2[axis]);
    } else {
        t = (h1.w + h1[axis]) / (h1.w + h1[axis] - h2.w - h2[axis]);
    }
    t.value = math::clamp(t.value, 0.0f, 1.0f);
    return t;
}

template <uint32_t axis, bool side>
static inline vtrz::DVector4 clip(const vtrz::DVector4 &grad1, const vtrz::DVector4 &grad2) {
    vtrz::dfloat t = lerpFactor<axis, side>(grad1, grad2);
    vtrz::DVector4 grado = lerp(t, grad1, grad2);
    grado[axis].value = side ? grado.w.value : -grado.w.value;
    return grado;
}

template <uint32_t axis, bool side>
static inline void clip(
    const vtrz::DVector4 &grad1, const Normal &normal1,
    const vtrz::DVector4 &grad2, const Normal &normal2,
    vtrz::DVector4 &grado, Normal &normalo) {
    vtrz::dfloat t = lerpFactor<axis, side>(grad1, grad2);
    grado = lerp(t, grad1, grad2);
    grado[axis].value = side ? grado.w.value : -grado.w.value;
    normalo = lerp(t.value, normal1, normal2);
}

template <uint32_t axis, bool side>
static inline void clip(
    const vtrz::DVector4 &grad1, const vtrz::DVector3 &world1, const Normal &normal1,
    const vtrz::DVector4 &grad2, const vtrz::DVector3 &world2, const Normal &normal2,
    vtrz::DVector4 &grado, vtrz::DVector3 &worldo, Normal &normalo) {
    vtrz::dfloat t = lerpFactor<axis, side>(grad1, grad2);
    grado = lerp(t, grad1, grad2);
    grado[axis].value = side ? grado.w.value : -grado.w.value;
    worldo = lerp(t, world1, world2);
    normalo = lerp(t.value, normal1, normal2);
}

template <uint32_t axis, bool side>
static inline bool sutherlandHodgmanPass(
    vtrz::DVector4 *pos1, uint32_t n1,
    vtrz::DVector4 *pos2, uint32_t &n2)
{
    n2 = 0;
    pos1[n1] = pos1[0];
    bool insideStart = inside<axis, side>(pos1[0]);
    for (uint32_t j = 0; j < n1; ++j) {
        bool insideEnd = inside<axis, side>(pos1[j + 1]);
        if (insideStart && insideEnd) {
            pos2[n2] = pos1[j + 1];
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        } else if (insideStart && !insideEnd) {
            pos2[n2] = clip<axis, side>(pos1[j], pos1[j + 1]);
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        } else if (!insideStart && insideEnd) {
            pos2[n2] = clip<axis, side>(pos1[j + 1], pos1[j]);
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;

            pos2[n2] = pos1[j + 1];
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        }
        // !insideStart && !insideEnd: do nothing.

        insideStart = insideEnd;
    }
    return n2 >= 3;
}

template <uint32_t axis, bool side>
static inline bool sutherlandHodgmanPass(
    vtrz::DVector4 *pos1, Normal *normal1, uint32_t n1,
    vtrz::DVector4 *pos2, Normal *normal2, uint32_t &n2)
{
    n2 = 0;
    pos1[n1] = pos1[0];
    normal1[n1] = normal1[0];
    bool insideStart = inside<axis, side>(pos1[0]);
    for (uint32_t j = 0; j < n1; ++j) {
        bool insideEnd = inside<axis, side>(pos1[j + 1]);
        if (insideStart && insideEnd) {
            pos2[n2] = pos1[j + 1];
            normal2[n2] = normal1[j + 1];
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        } else if (insideStart && !insideEnd) {
            clip<axis, side>(pos1[j], normal1[j], pos1[j + 1], normal1[j + 1], pos2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        } else if (!insideStart && insideEnd) {
            clip<axis, side>(pos1[j + 1], normal1[j + 1], pos1[j], normal1[j], pos2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;

            pos2[n2] = pos1[j + 1];
            normal2[n2] = normal1[j + 1];
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        }

        // !insideStart && !insideEnd: do nothing.
        insideStart = insideEnd;
    }
    return n2 >= 3;
}

template <uint32_t axis, bool side>
static inline bool sutherlandHodgmanPass(
    vtrz::DVector4 *pos1, vtrz::DVector3 *world1, Normal *normal1, uint32_t n1,
    vtrz::DVector4 *pos2, vtrz::DVector3 *world2, Normal *normal2, uint32_t &n2)
{
    n2 = 0;
    pos1[n1] = pos1[0];
    normal1[n1] = normal1[0];
    world1[n1] = world1[0];
    bool insideStart = inside<axis, side>(pos1[0]);
    for (uint32_t j = 0; j < n1; ++j) {
        bool insideEnd = inside<axis, side>(pos1[j + 1]);
        if (insideStart && insideEnd) {
            pos2[n2] = pos1[j + 1];
            normal2[n2] = normal1[j + 1];
            world2[n2] = world1[j + 1];
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        } else if (insideStart && !insideEnd) {
            clip<axis, side>(pos1[j], world1[j], normal1[j], pos1[j + 1], world1[j + 1], normal1[j + 1], pos2[n2], world2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        } else if (!insideStart && insideEnd) {
            clip<axis, side>(pos1[j + 1], world1[j + 1], normal1[j + 1], pos1[j], world1[j], normal1[j], pos2[n2], world2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;

            pos2[n2] = pos1[j + 1];
            world2[n2] = world1[j + 1];
            normal2[n2] = normal1[j + 1];
            if (n2 == 0 || (pos2[n2].value() != pos2[n2 - 1].value() && pos2[n2].value() != pos2[0].value())) ++n2;
        }

        // !insideStart && !insideEnd: do nothing.
        insideStart = insideEnd;
    }
    return n2 >= 3;
}

void PrimaryPipelineAD::add(
    vtrz::DVector4 *pos1, vtrz::DVector3 *world1, Normal *normal1,
    uint32_t meshId, uint16_t matId, uint32_t primId, bool noClip)
{
    for (uint32_t i = 0; i < 3; ++i) {
        pos1[i] = vpTrans * pos1[i];
    }

    vtrz::DVector4 pos2[kMaxClipVertexCount];
    vtrz::DVector3 world2[kMaxClipVertexCount];
    Normal normal2[kMaxClipVertexCount];

    uint32_t nin = 3;
    vtrz::DVector4 *inPos = pos1;
    vtrz::DVector3 *inWorld = world1;
    Normal *inNormal = normal1;

    uint32_t nout = 0;
    vtrz::DVector4 *outPos = pos2;
    vtrz::DVector3 *outWorld = world1;
    Normal *outNormal = normal2;

    if (!noClip) {
        uint8_t vertClipFlags[3] = { 0, 0, 0 };
        for (uint32_t i = 0; i < 3; ++i) {
            vertClipFlags[i] |= inside<0, true>(inPos[i]) ? 0 : ClipFlag::ERight;
            vertClipFlags[i] |= inside<0, false>(inPos[i]) ? 0 : ClipFlag::ELeft;
            vertClipFlags[i] |= inside<1, true>(inPos[i]) ? 0 : ClipFlag::ETop;
            vertClipFlags[i] |= inside<1, false>(inPos[i]) ? 0 : ClipFlag::EBottom;
            vertClipFlags[i] |= inside<2, true>(inPos[i]) ? 0 : ClipFlag::ENear;
        }
        if (vertClipFlags[0] & vertClipFlags[1] & vertClipFlags[2]) {
            return; // All out.
        }
        uint8_t clipFlags = vertClipFlags[0] | vertClipFlags[1] | vertClipFlags[2];
        if (clipFlags & ClipFlag::ERight) {
            if (!sutherlandHodgmanPass<0, true>(inPos, inWorld, inNormal, nin, outPos, outWorld, outNormal, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos); std::swap(inWorld, outWorld); std::swap(inNormal, outNormal);
        }
        if (clipFlags & ClipFlag::ELeft) {
            if (!sutherlandHodgmanPass<0, false>(inPos, inWorld, inNormal, nin, outPos, outWorld, outNormal, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos); std::swap(inWorld, outWorld); std::swap(inNormal, outNormal);
        }
        if (clipFlags & ClipFlag::ETop) {
            if (!sutherlandHodgmanPass<1, true>(inPos, inWorld, inNormal, nin, outPos, outWorld, outNormal, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos); std::swap(inWorld, outWorld); std::swap(inNormal, outNormal);
        }
        if (clipFlags & ClipFlag::EBottom) {
            if (!sutherlandHodgmanPass<1, false>(inPos, inWorld, inNormal, nin, outPos, outWorld, outNormal, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos); std::swap(inWorld, outWorld); std::swap(inNormal, outNormal);
        }
        if (clipFlags & ClipFlag::ENear) {
            if (!sutherlandHodgmanPass<2, true>(inPos, inWorld, inNormal, nin, outPos, outWorld, outNormal, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos); std::swap(inWorld, outWorld); std::swap(inNormal, outNormal);
        }
    }
    std::swap(nin, nout); std::swap(inPos, outPos); std::swap(inWorld, outWorld); std::swap(inNormal, outNormal);


    for (uint32_t i = 0; i < nout; ++i) {
        outPos[i].x /= outPos[i].w;
        outPos[i].y /= outPos[i].w;
        outPos[i].z /= outPos[i].w;

        outPos[i].x.value = math::clamp(outPos[i].x.value, -1.0f, 1.0f);
        outPos[i].y.value = math::clamp(outPos[i].y.value, -1.0f, 1.0f);
        outPos[i].z.value = math::clamp(outPos[i].z.value, 0.0f, 1.0f);

        //if (!clipped[i]) {
            constexpr Float snapEps = 1e-5f;
            if (std::abs(outPos[i].x.value - 1.0f) <= snapEps ||
                std::abs(outPos[i].x.value + 1.0f) <= snapEps) {
                outPos[i].x.zeroGrad();
            }
            if (std::abs(outPos[i].y.value - 1.0f) <= snapEps ||
                std::abs(outPos[i].y.value + 1.0f) <= snapEps) {
                outPos[i].y.zeroGrad();
            }
        //}

        outPos[i].w = 1.0f / outPos[i].w;
    }

    for (uint32_t i = 0; i < nout - 2; ++i) {
        // Backface culling. Do it for every new triangle to avoid winding order flip due to rounding error.
        Vector2 e01(outPos[i + 1].x.value - outPos[0].x.value, outPos[i + 1].y.value - outPos[0].y.value);
        Vector2 e12(outPos[i + 2].x.value - outPos[i + 1].x.value, outPos[i + 2].y.value - outPos[i + 1].y.value);
        if (e01.x * e12.y - e01.y * e12.x <= 0.0f) {
            continue;
        }

        positions.push_back(vtrz::DVector3(outPos[0].x, outPos[0].y, outPos[0].z));
        positions.push_back(vtrz::DVector3(outPos[i + 1].x, outPos[i + 1].y, outPos[i + 1].z));
        positions.push_back(vtrz::DVector3(outPos[i + 2].x, outPos[i + 2].y, outPos[i + 2].z));

        worldPositions.push_back(outWorld[0]);
        worldPositions.push_back(outWorld[i + 1]);
        worldPositions.push_back(outWorld[i + 2]);

        normals.push_back(outNormal[0]);
        normals.push_back(outNormal[i + 1]);
        normals.push_back(outNormal[i + 2]);

        wInvs.push_back(outPos[0].w);
        wInvs.push_back(outPos[i + 1].w);
        wInvs.push_back(outPos[i + 2].w);

        meshIds.push_back(meshId);
        matIds.push_back(matId);
        primIds.push_back(primId);
    }
}

void PrimaryPipelineAD::run(
    const Scene &scene, const BVH &bvh, const SceneAD &sceneAD,
    const FrustumBVHIsectResult &isects)
{
    uint32_t initCapacity = std::min(isects.totalPrimCount, 64u);
    positions.reserve(initCapacity);
    worldPositions.reserve(initCapacity);
    normals.reserve(initCapacity);
    wInvs.reserve(initCapacity);

    meshIds.reserve(initCapacity);
    matIds.reserve(initCapacity);
    primIds.reserve(initCapacity);

    const std::vector<Primitive> &primitives = bvh.primitives();
    for (FrustumBVHIsectEntry entry : isects.entries) {
        for (uint32_t i = 0; i < entry.primCount; ++i) {
            const Primitive &prim = primitives[entry.primOffset + i];
            const TriMesh *mesh = scene.getMeshes()[prim.meshIndex];
            Triangle tri = mesh->getTriangles()[prim.primIndex];
            if (backfaceCull(apex,
                mesh->getVertexPositions()[tri.idx[0]],
                mesh->getVertexPositions()[tri.idx[1]],
                mesh->getVertexPositions()[tri.idx[2]])) {
                continue;
            }
            vtrz::DVector4 pos[kMaxClipVertexCount];
            vtrz::DVector3 world[kMaxClipVertexCount];
            Normal normal[kMaxClipVertexCount];

            auto it = sceneAD.indexMap.find(prim.meshIndex);
            if (it != sceneAD.indexMap.end()) {
                for (uint32_t v = 0; v < 3; ++v) {
                    const vtrz::DVector3 &p = sceneAD.meshADs[it->second].positions[tri.idx[v]];
                    world[v] = p;
                    pos[v] = vtrz::DVector4(p.x, p.y, p.z, vtrz::dfloat(1.0f));
                }
            } else {
                for (uint32_t v = 0; v < 3; ++v) {
                    // Fill values by default if there's no gradient.
                    const Point &p = mesh->getVertexPositions()[tri.idx[v]];
                    world[v].setValue(vtrz::Vector3(p.x, p.y, p.z));
                    pos[v].setValue(vtrz::Vector4(p.x, p.y, p.z, 1.0f));
                }
            }
            if (mesh->hasVertexNormals()) {
                for (uint32_t v = 0; v < 3; ++v) {
                    normal[v] = mesh->getVertexNormals()[tri.idx[v]];
                }
            } else {
                normal[0] = normal[1] = normal[2] = Normal(normalize(cross(
                    mesh->getVertexPositions()[tri.idx[1]] - mesh->getVertexPositions()[tri.idx[0]],
                    mesh->getVertexPositions()[tri.idx[2]] - mesh->getVertexPositions()[tri.idx[1]])));
            }
            add(pos, world, normal, (uint16_t)prim.meshIndex, mesh->getBSDF()->getLabel(), prim.primIndex, entry.included);
        }
    }
}

static vtrz::DVector3 rayTriIsectGrad(
    const vtrz::DVector3 &origin, const vtrz::DVector3 &dir,
    const vtrz::DVector3 &v0, const vtrz::DVector3 &v1, const vtrz::DVector3 &v2)
{
    // Assume ray and tri must intersect.
    vtrz::DVector3 coord;

    vtrz::DVector3 v0v1 = v1 - v0;
    vtrz::DVector3 v0v2 = v2 - v0;
    vtrz::DVector3 pvec = vtrz::cross(dir, v0v2);

    vtrz::dfloat det = vtrz::dot(v0v1, pvec);

    vtrz::dfloat invDet = vtrz::dfloat(1) / det;

    vtrz::DVector3 tvec = origin - v0;
    coord.y = vtrz::dot(tvec, pvec) * invDet;
    coord.y.value = vtrz::saturate(coord.y.value);
    //if (u < 0 || u > 1) return false;

    vtrz::DVector3 qvec = vtrz::cross(tvec, v0v1);
    coord.z = vtrz::dot(dir, qvec) * invDet;
    coord.z.value = vtrz::saturate(coord.z.value);
    //if (v < 0 || u + v > 1) return false;

    coord.x = vtrz::dfloat(1.0f) - coord.y - coord.z;
    coord.x.value = vtrz::saturate(coord.x.value);
    // vtrz::dfloat t = vtrz::dot(v0v2, qvec) * invDet;

    return coord;
}

IntersectionGrad PrimaryPipelineAD::buildItsFromSample(
    const vtrz::PointSampleGrad &entry, const Point &sensorPos,
    const Scene &scene, const SceneAD &sceneAD) const
{
    IntersectionGrad itsGrad;
    // Irrelevant fields.
    itsGrad.its.time = Float(0.0);
    itsGrad.its.color = Spectrum(0.0);
    itsGrad.its.instance = nullptr;
    itsGrad.its.wi = Vector(0.0);
    // its.primitive

    // Useful fields.
    uint32_t meshIndex = meshIds[entry.triId];
    const TriMesh *mesh = scene.getMeshes()[meshIndex];
    itsGrad.its.shape = mesh;
    uint32_t primIndex = primIds[entry.triId];
    Triangle tri = scene.getMeshes()[meshIndex]->getTriangles()[primIndex];
    uint32_t offset = entry.triId * 3;

    vtrz::DVector3 perspCoord = entry.coord;
    perspCoord.x *= wInvs[entry.triId * 3 + 0];
    perspCoord.y *= wInvs[entry.triId * 3 + 1];
    perspCoord.z *= wInvs[entry.triId * 3 + 2];
    perspCoord /= (perspCoord.x + perspCoord.y + perspCoord.z);

    {
        const vtrz::DVector3 &wp0 = worldPositions[offset + 0];
        const vtrz::DVector3 &wp1 = worldPositions[offset + 1];
        const vtrz::DVector3 &wp2 = worldPositions[offset + 2];

        itsGrad.dp = wp0 * perspCoord[0] + wp1 * perspCoord[1] + wp2 * perspCoord[2];
        itsGrad.its.p = toPoint(itsGrad.dp.value());

        Point wp0p = toPoint(wp0.value());
        Point wp1p = toPoint(wp1.value());
        Point wp2p = toPoint(wp2.value());

        itsGrad.its.geoFrame = Frame(normalize(cross(wp1p - wp0p, wp2p - wp1p)));
    }

    // Use inverse transform to obtain world position. No need to interpolate, but inverse transform is numerically error-prone?
//    {
//    Point p0(positions[offset + 0].x.value, positions[offset + 0].y.value, positions[offset + 0].z.value);
//    Point p1(positions[offset + 1].x.value, positions[offset + 1].y.value, positions[offset + 1].z.value);
//    Point p2(positions[offset + 2].x.value, positions[offset + 2].y.value, positions[offset + 2].z.value);
//    itsGrad.its.p = p0 * entry.coord.x.value + p1 * entry.coord.y.value + p2 * entry.coord.z.value;
//    itsGrad.its.p = vpTrans.inverse()(itsGrad.its.p);
//
//    const Point &op0 = mesh->getVertexPositions()[tri.idx[0]];
//    const Point &op1 = mesh->getVertexPositions()[tri.idx[1]];
//    const Point &op2 = mesh->getVertexPositions()[tri.idx[2]];
//    itsGrad.its.geoFrame = Frame(normalize(cross(op1 - op0, op2 - op1)));
//}

    auto it = sceneAD.indexMap.find(meshIndex);
    const TriMeshAD *meshAD = nullptr;
    if (it != sceneAD.indexMap.end()) {
        meshAD = &sceneAD.meshADs[it->second];
    }

    vtrz::DVector3 dp0, dp1, dp2;
    if (meshAD) {
        dp0 = meshAD->positions[tri.idx[0]];
        dp1 = meshAD->positions[tri.idx[1]];
        dp2 = meshAD->positions[tri.idx[2]];
    } else {
        const Point &op0 = mesh->getVertexPositions()[tri.idx[0]];
        const Point &op1 = mesh->getVertexPositions()[tri.idx[1]];
        const Point &op2 = mesh->getVertexPositions()[tri.idx[2]];
        dp0 = vtrz::DVector3(op0.x, op0.y, op0.z);
        dp1 = vtrz::DVector3(op1.x, op1.y, op1.z);
        dp2 = vtrz::DVector3(op2.x, op2.y, op2.z);
    }
    vtrz::DVector3 coord3dGrad = rayTriIsectGrad(
        vtrz::DVector3(sensorPos.x, sensorPos.y, sensorPos.z),
        vtrz::DVector3(itsGrad.its.p.x - sensorPos.x, itsGrad.its.p.y - sensorPos.y, itsGrad.its.p.z - sensorPos.z),
        dp0, dp1, dp2);
    Vector3 coord3d(coord3dGrad.x.value, coord3dGrad.y.value, coord3dGrad.z.value);
    {
        Vector N;
        vtrz::DVector3 dN;
        if (meshAD && meshAD->type == TriMeshADType::ePerVertexAD) {
            if (mesh->hasVertexNormals()) {
                const vtrz::DVector3 &n0 = meshAD->normals[tri.idx[0]];
                const vtrz::DVector3 &n1 = meshAD->normals[tri.idx[1]];
                const vtrz::DVector3 &n2 = meshAD->normals[tri.idx[2]];
                dN = vtrz::normalize(
                    vtrz::DVector3(n0.x, n0.y, n0.z) * coord3dGrad[0] +
                    vtrz::DVector3(n1.x, n1.y, n1.z) * coord3dGrad[1] +
                    vtrz::DVector3(n2.x, n2.y, n2.z) * coord3dGrad[2]);
            } else {
                dN = meshAD->normals[primIndex];
            }
            N.x = dN.x.value; N.y = dN.y.value; N.z = dN.z.value;
        } else {
            if (mesh->hasVertexNormals()) {
                const Normal &n0 = mesh->getVertexNormals()[tri.idx[0]];
                const Normal &n1 = mesh->getVertexNormals()[tri.idx[1]];
                const Normal &n2 = mesh->getVertexNormals()[tri.idx[2]];

                N = normalize(n0 * coord3d[0] + n1 * coord3d[1] + n2 * coord3d[2]);
                dN = vtrz::normalize(
                    vtrz::DVector3(n0.x, n0.y, n0.z) * coord3dGrad[0] +
                    vtrz::DVector3(n1.x, n1.y, n1.z) * coord3dGrad[1] +
                    vtrz::DVector3(n2.x, n2.y, n2.z) * coord3dGrad[2]);
            } else {
                N = itsGrad.its.geoFrame.n;
                dN = vtrz::DVector3(N.x, N.y, N.z);
            }
        }
        // Important: do not construct an arbitrary shading frame!
        // LTC requires the frame to be constructed this way.
        Vector V = normalize(sensorPos - itsGrad.its.p);
        Vector T1 = normalize(V - N * dot(V, N));
        Vector T2 = cross(N, T1);
        itsGrad.its.shFrame = Frame(T1, T2, N);

        vtrz::DVector3 dV = vtrz::normalize(vtrz::DVector3(sensorPos.x, sensorPos.y, sensorPos.z) - itsGrad.dp);
        vtrz::DVector3 dT1 = vtrz::normalize(dV - dN * vtrz::dot(dV, dN));
        vtrz::DVector3 dT2 = vtrz::cross(dN, dT1);
        itsGrad.dShFrame = DFrame(dT1, dT2, dN);
    }

    if (mesh->hasVertexTexcoords()) {
        const Point2 &tc0 = mesh->getVertexTexcoords()[tri.idx[0]];
        const Point2 &tc1 = mesh->getVertexTexcoords()[tri.idx[1]];
        const Point2 &tc2 = mesh->getVertexTexcoords()[tri.idx[2]];
        itsGrad.its.uv = tc0 * coord3d[0] + tc1 * coord3d[1] + tc2 * coord3d[2];
        itsGrad.duv =
            vtrz::DVector2(tc0.x, tc0.y) * coord3dGrad[0] +
            vtrz::DVector2(tc1.x, tc1.y) * coord3dGrad[1] +
            vtrz::DVector2(tc2.x, tc2.y) * coord3dGrad[2];
    } else {
        itsGrad.its.uv = Point2(0.0f);
        itsGrad.duv = vtrz::DVector2(0.0f);
    }

    if (mesh->hasUVTangents()) {
        itsGrad.its.dpdu = mesh->getUVTangents()[primIndex].dpdu;
        itsGrad.its.dpdv = mesh->getUVTangents()[primIndex].dpdv;
    } else {
        itsGrad.its.dpdu = itsGrad.its.dpdv = Vector(0.0);
    }

    itsGrad.its.hasUVPartials = false;
    return itsGrad;
}

bool PrimaryPipelineAD::buildItsPatch(
    const vtrz::LeafRegionGrad &region, const Point &sensorPos,
    const Scene &scene, const SceneAD &sceneAD,
    IntersectionPatchGrad &patch) const
{
    patch.positions.clear();
    patch.normals.clear();
    patch.avgItsGrad = {};

    uint32_t offset = region.triId * 3;
    const vtrz::DVector3 &p0 = positions[offset + 0];
    const vtrz::DVector3 &p1 = positions[offset + 1];
    const vtrz::DVector3 &p2 = positions[offset + 2];
    vtrz::Tri2Grad tri2d = {
        vtrz::DVector2(p0.x, p0.y),
        vtrz::DVector2(p1.x, p1.y),
        vtrz::DVector2(p2.x, p2.y),
    };

    patch.positions.resize(region.vertCount);
    patch.coverage = vtrz::dfloat(0.0f);
    for (uint32_t v = 0; v < region.vertCount; ++v) {
        vtrz::DVector2 vert = vtrz::cast(region.poly[v]);
        vtrz::DVector2 vnext = vtrz::cast(region.poly[(v + 1) % region.vertCount]);
        patch.coverage += vtrz::cross(vert, vnext);

        vtrz::DVector3 perspCoord = vtrz::barycentricCoordSafe(vert, tri2d);
        perspCoord.x *= wInvs[offset + 0];
        perspCoord.y *= wInvs[offset + 1];
        perspCoord.z *= wInvs[offset + 2];
        perspCoord /= (perspCoord.x + perspCoord.y + perspCoord.z);

        const vtrz::DVector3 &wp0 = worldPositions[offset + 0];
        const vtrz::DVector3 &wp1 = worldPositions[offset + 1];
        const vtrz::DVector3 &wp2 = worldPositions[offset + 2];

        patch.positions[v] = wp0 * perspCoord.x + wp1 * perspCoord.y + wp2 * perspCoord.z;
    }

    patch.coverage = patch.coverage * 0.5f * 0.25f; // TODO: invBoundArea = 1/4
    constexpr float kSmallCoverageThreshold = 1e-4;
    if (patch.coverage.value <= kSmallCoverageThreshold) {
        return false;
    }

    vtrz::DVector3 centroid(0.0f);
    for (uint32_t v = 0; v < region.vertCount; ++v) {
        centroid += patch.positions[v];
    }
    centroid /= (float)region.vertCount;

    patch.meshId = meshIds[region.triId];
    const TriMesh *mesh = scene.getMeshes()[patch.meshId];
    Triangle tri = mesh->getTriangles()[primIds[region.triId]];

    auto it = sceneAD.indexMap.find(patch.meshId);
    const TriMeshAD *meshAD = nullptr;
    if (it != sceneAD.indexMap.end()) {
        meshAD = &sceneAD.meshADs[it->second];
    }

    vtrz::DVector3 dp0, dp1, dp2;
    if (meshAD) {
        dp0 = meshAD->positions[tri.idx[0]];
        dp1 = meshAD->positions[tri.idx[1]];
        dp2 = meshAD->positions[tri.idx[2]];
    } else {
        dp0 = vtrz::DVector3(toVec3(mesh->getVertexPositions()[tri.idx[0]]));
        dp1 = vtrz::DVector3(toVec3(mesh->getVertexPositions()[tri.idx[1]]));
        dp2 = vtrz::DVector3(toVec3(mesh->getVertexPositions()[tri.idx[2]]));
    }

    vtrz::DVector3 sensorPosAD(toVec3(sensorPos));

    if (mesh->hasVertexNormals()) {
        patch.normals.resize(region.vertCount);
        for (uint32_t v = 0; v < region.vertCount; ++v) {
            vtrz::DVector3 coord3d = rayTriIsectGrad(sensorPosAD, patch.positions[v] - sensorPosAD, dp0, dp1, dp2);
            if (meshAD && meshAD->type == TriMeshADType::ePerVertexAD) {
                const vtrz::DVector3 &n0 = meshAD->normals[tri.idx[0]];
                const vtrz::DVector3 &n1 = meshAD->normals[tri.idx[1]];
                const vtrz::DVector3 &n2 = meshAD->normals[tri.idx[2]];
                patch.normals[v] = vtrz::normalize(
                    vtrz::DVector3(n0.x, n0.y, n0.z) * coord3d[0] +
                    vtrz::DVector3(n1.x, n1.y, n1.z) * coord3d[1] +
                    vtrz::DVector3(n2.x, n2.y, n2.z) * coord3d[2]);
            } else {
                const Normal &n0 = mesh->getVertexNormals()[tri.idx[0]];
                const Normal &n1 = mesh->getVertexNormals()[tri.idx[1]];
                const Normal &n2 = mesh->getVertexNormals()[tri.idx[2]];
                patch.normals[v] = vtrz::normalize(
                    vtrz::DVector3(n0.x, n0.y, n0.z) * coord3d[0] +
                    vtrz::DVector3(n1.x, n1.y, n1.z) * coord3d[1] +
                    vtrz::DVector3(n2.x, n2.y, n2.z) * coord3d[2]);
            }
        }
    }

    IntersectionGrad &itsGrad = patch.avgItsGrad;
    // Irrelevant fields.
    itsGrad.its.time = Float(0.0);
    itsGrad.its.color = Spectrum(0.0);
    itsGrad.its.instance = nullptr;
    itsGrad.its.wi = Vector(0.0);

    // Useful fields: actually only position, face normal, uv, and dpduv.
    itsGrad.its.shape = mesh;
    itsGrad.its.primIndex = primIds[region.triId];

    itsGrad.dp = centroid;
    itsGrad.its.p = toPoint(itsGrad.dp.value());

    // TODO: d geo frame?
    // toPoint(dp1.value())
    vtrz::DVector3 dN = vtrz::normalize(vtrz::cross(dp1 - dp0, dp2 - dp1));
    vtrz::DVector3 dV = vtrz::normalize(sensorPosAD - itsGrad.dp);
    vtrz::DVector3 dT1 = vtrz::normalize(dV - dN * vtrz::dot(dV, dN));
    vtrz::DVector3 dT2 = vtrz::cross(dN, dT1);
    itsGrad.dShFrame = DFrame(dT1, dT2, dN);

    itsGrad.its.geoFrame = Frame(toNormal(dN.value()));

    vtrz::DVector3 centeroidCoord3d = rayTriIsectGrad(sensorPosAD, centroid - sensorPosAD, dp0, dp1, dp2);

    if (mesh->hasVertexTexcoords()) {
        const Point2 &tc0 = mesh->getVertexTexcoords()[tri.idx[0]];
        const Point2 &tc1 = mesh->getVertexTexcoords()[tri.idx[1]];
        const Point2 &tc2 = mesh->getVertexTexcoords()[tri.idx[2]];
        itsGrad.duv =
            vtrz::DVector2(tc0.x, tc0.y) * centeroidCoord3d[0] +
            vtrz::DVector2(tc1.x, tc1.y) * centeroidCoord3d[1] +
            vtrz::DVector2(tc2.x, tc2.y) * centeroidCoord3d[2];
        itsGrad.its.uv = Point2(itsGrad.duv.x.value, itsGrad.duv.y.value);
    } else {
        itsGrad.its.uv = Point2(0.0f);
        itsGrad.duv = vtrz::DVector2(0.0f);
    }

    if (mesh->hasUVTangents()) {
        itsGrad.its.dpdu = mesh->getUVTangents()[itsGrad.its.primIndex].dpdu;
        itsGrad.its.dpdv = mesh->getUVTangents()[itsGrad.its.primIndex].dpdv;
    } else {
        itsGrad.its.dpdu = itsGrad.its.dpdv = Vector(0.0);
    }

    itsGrad.its.hasUVPartials = false;

    return true;
}

void SecondaryPipelineAD::clear()
{
    positions.clear();

    queryTriIndices.clear();
}

void SecondaryPipelineAD::add(vtrz::DVector4 *pos1, bool query, bool noClip)
{
    for (uint32_t i = 0; i < 3; ++i) {
        pos1[i] = vpTrans * pos1[i];
    }

    vtrz::DVector4 pos2[kMaxClipVertexCount];

    uint32_t nin = 3;
    vtrz::DVector4 *inPos = pos1;

    uint32_t nout = 0;
    vtrz::DVector4 *outPos = pos2;

    if (!noClip) {
        uint8_t vertClipFlags[3] = { 0, 0, 0 };
        for (uint32_t i = 0; i < 3; ++i) {
            vertClipFlags[i] |= inside<0, true>(inPos[i]) ? 0 : ClipFlag::ERight;
            vertClipFlags[i] |= inside<0, false>(inPos[i]) ? 0 : ClipFlag::ELeft;
            vertClipFlags[i] |= inside<1, true>(inPos[i]) ? 0 : ClipFlag::ETop;
            vertClipFlags[i] |= inside<1, false>(inPos[i]) ? 0 : ClipFlag::EBottom;
            vertClipFlags[i] |= inside<2, true>(inPos[i]) ? 0 : ClipFlag::ENear;
        }
        if (vertClipFlags[0] & vertClipFlags[1] & vertClipFlags[2]) {
            return; // All out.
        }
        uint8_t clipFlags = vertClipFlags[0] | vertClipFlags[1] | vertClipFlags[2];
        if (clipFlags & ClipFlag::ERight) {
            if (!sutherlandHodgmanPass<0, true>(inPos, nin, outPos, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos);
        }
        if (clipFlags & ClipFlag::ELeft) {
            if (!sutherlandHodgmanPass<0, false>(inPos, nin, outPos, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos);
        }
        if (clipFlags & ClipFlag::ETop) {
            if (!sutherlandHodgmanPass<1, true>(inPos, nin, outPos, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos);
        }
        if (clipFlags & ClipFlag::EBottom) {
            if (!sutherlandHodgmanPass<1, false>(inPos, nin, outPos, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos);
        }
        if (clipFlags & ClipFlag::ENear) {
            if (!sutherlandHodgmanPass<2, true>(inPos, nin, outPos, nout)) return;
            std::swap(nin, nout); std::swap(inPos, outPos);
        }
    }
    std::swap(nin, nout); std::swap(inPos, outPos);

    for (uint32_t i = 0; i < nout; ++i) {
        outPos[i].x /= outPos[i].w;
        outPos[i].y /= outPos[i].w;
        outPos[i].z /= outPos[i].w;

        outPos[i].x.value = math::clamp(outPos[i].x.value, -1.0f, 1.0f);
        outPos[i].y.value = math::clamp(outPos[i].y.value, -1.0f, 1.0f);
        outPos[i].z.value = math::clamp(outPos[i].z.value, 0.0f, 1.0f);

        //if (!clipped[i]) {
        //    constexpr Float snapEps = 1e-5f;
        //    if (std::abs(outputGradPtr[i].x.value - 1.0f) <= snapEps ||
        //        std::abs(outputGradPtr[i].x.value + 1.0f) <= snapEps) {
        //        outputGradPtr[i].x.zeroGrad();
        //    }
        //    if (std::abs(outputGradPtr[i].y.value - 1.0f) <= snapEps ||
        //        std::abs(outputGradPtr[i].y.value + 1.0f) <= snapEps) {
        //        outputGradPtr[i].y.zeroGrad();
        //    }
        //}
    }

    for (uint32_t i = 0; i < nout - 2; ++i) {
        // Backface culling. Do it for every new triangle to avoid winding order flip due to rounding error.
        Vector2 e01(outPos[i + 1].x.value - outPos[0].x.value, outPos[i + 1].y.value - outPos[0].y.value);
        Vector2 e12(outPos[i + 2].x.value - outPos[i + 1].x.value, outPos[i + 2].y.value - outPos[i + 1].y.value);
        if (e01.x * e12.y - e01.y * e12.x <= 0.0f) {
            continue;
        }

        if (query) {
            queryTriIndices.push_back((uint32_t)positions.size() / 3);
        }

        positions.push_back(vtrz::DVector3(outPos[0].x, outPos[0].y, outPos[0].z));
        positions.push_back(vtrz::DVector3(outPos[i + 1].x, outPos[i + 1].y, outPos[i + 1].z));
        positions.push_back(vtrz::DVector3(outPos[i + 2].x, outPos[i + 2].y, outPos[i + 2].z));
    }

}

void SecondaryPipelineAD::run(
    const Scene &scene, const BVH &bvh, const SceneAD &gradCache,
    const FrustumBVHIsectResult &isects, const TriMesh *queryMesh)
{
    const std::vector<Primitive> &primitives = bvh.primitives();
    for (FrustumBVHIsectEntry entry : isects.entries) {
        for (uint32_t i = 0; i < entry.primCount; ++i) {
            const Primitive &prim = primitives[entry.primOffset + i];
            const TriMesh *mesh = scene.getMeshes()[prim.meshIndex];
            if (mesh->getEmitter() && mesh != queryMesh && !mesh->getEmitter()->occlude()) {
                continue;
            }
            Triangle tri = mesh->getTriangles()[prim.primIndex];
            if (backfaceCull(apex,
                mesh->getVertexPositions()[tri.idx[0]],
                mesh->getVertexPositions()[tri.idx[1]],
                mesh->getVertexPositions()[tri.idx[2]])) {
                continue;
            }
            vtrz::DVector4 pos[kMaxClipVertexCount];
            auto it = gradCache.indexMap.find(prim.meshIndex);
            if (it != gradCache.indexMap.end()) {
                for (uint32_t v = 0; v < 3; ++v) {
                    const vtrz::DVector3 &p = gradCache.meshADs[it->second].positions[tri.idx[v]];
                    pos[v] = vtrz::DVector4(p.x, p.y, p.z, vtrz::dfloat(1.0f));
                }
            } else {
                for (uint32_t v = 0; v < 3; ++v) {
                    // Fill values by default if there's no gradient.
                    const Point &p = mesh->getVertexPositions()[tri.idx[v]];
                    pos[v].setValue(vtrz::Vector4(p.x, p.y, p.z, 1.0f));
                }
            }
            add(pos, mesh == queryMesh, entry.included);
        }
    }
}

void SecondaryPipelineAD::add(const IntersectionPatchGrad &patch)
{
    for (uint32_t i = 0; i < (uint32_t)patch.positions.size() - 2; ++i) {
        if (backfaceCull(apex,
            toPoint(patch.positions[0].value()),
            toPoint(patch.positions[i + 1].value()),
            toPoint(patch.positions[i + 2].value()))) {
            continue;
        }
        vtrz::DVector4 pos[kMaxClipVertexCount];
        pos[0] = vtrz::DVector4(patch.positions[0].x, patch.positions[0].y, patch.positions[0].z, 1.0f);
        pos[1] = vtrz::DVector4(patch.positions[i + 1].x, patch.positions[i + 1].y, patch.positions[i + 1].z, 1.0f);
        pos[2] = vtrz::DVector4(patch.positions[i + 2].x, patch.positions[i + 2].y, patch.positions[i + 2].z, 1.0f);
        add(pos, false, false);
    }
}

MTS_NAMESPACE_END