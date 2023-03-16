#include "pipeline.h"
#include "bvh.h"

MTS_NAMESPACE_BEGIN

void PrimaryPipeline::clear()
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
static inline bool inside(const Vector4 &h) {
    if (side) {
        return h[axis] <= h.w;
    } else {
        return h[axis] >= -h.w;
    }
}

template <uint32_t axis, bool side>
static inline Float lerpFactor(const Vector4 &h1, const Vector4 &h2) {
    Float t = 0.0f;
    if (side) {
        t = (h1.w - h1[axis]) / (h1.w - h1[axis] - h2.w + h2[axis]);
    } else {
        t = (h1.w + h1[axis]) / (h1.w + h1[axis] - h2.w - h2[axis]);
    }
    t = math::clamp(t, 0.0f, 1.0f);
    return t;
}

template <uint32_t axis, bool side>
static inline Vector4 clip(const Vector4 &h1, const Vector4 &h2) {
    Float t = lerpFactor<axis, side>(h1, h2);
    Vector4 ho = lerp(t, h1, h2);
    ho[axis] = side ? ho.w : -ho.w;
    return ho;
}

template <uint32_t axis, bool side>
static inline void clip(
    const Vector4 &h1, const Normal &normal1,
    const Vector4 &h2, const Normal &normal2,
    Vector4 &ho, Normal &normalo) {
    Float t = lerpFactor<axis, side>(h1, h2);
    ho = lerp(t, h1, h2);
    ho[axis] = side ? ho.w : -ho.w;
    normalo = lerp(t, normal1, normal2);
}

template <uint32_t axis, bool side>
static inline void clip(
    const Vector4 &h1, const Point &world1, const Normal &normal1,
    const Vector4 &h2, const Point &world2, const Normal &normal2,
    Vector4 &ho, Point &worldo, Normal &normalo) {
    Float t = lerpFactor<axis, side>(h1, h2);
    ho = lerp(t, h1, h2);
    ho[axis] = side ? ho.w : -ho.w;
    worldo = lerp(t, world1, world2);
    normalo = lerp(t, normal1, normal2);
}

template <uint32_t axis, bool side>
static inline bool sutherlandHodgmanPass(Vector4 *pos1, uint32_t n1, Vector4 *pos2, uint32_t &n2)
{
    n2 = 0;
    pos1[n1] = pos1[0];
    bool insideStart = inside<axis, side>(pos1[0]);
    for (uint32_t j = 0; j < n1; ++j) {
        bool insideEnd = inside<axis, side>(pos1[j + 1]);
        if (insideStart && insideEnd) {
            pos2[n2] = pos1[j + 1];
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        } else if (insideStart && !insideEnd) {
            pos2[n2] = clip<axis, side>(pos1[j], pos1[j + 1]);
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        } else if (!insideStart && insideEnd) {
            pos2[n2] = clip<axis, side>(pos1[j + 1], pos1[j]);
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;

            pos2[n2] = pos1[j + 1];
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        }
        // !insideStart && !insideEnd: do nothing.

        insideStart = insideEnd;
    }
    return n2 >= 3;
}

template <uint32_t axis, bool side>
static inline bool sutherlandHodgmanPass(
    Vector4 *pos1, Normal *normal1, uint32_t n1,
    Vector4 *pos2, Normal *normal2, uint32_t &n2)
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
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        } else if (insideStart && !insideEnd) {
            clip<axis, side>(pos1[j], normal1[j], pos1[j + 1], normal1[j + 1], pos2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        } else if (!insideStart && insideEnd) {
            clip<axis, side>(pos1[j + 1], normal1[j + 1], pos1[j], normal1[j], pos2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;

            pos2[n2] = pos1[j + 1];
            normal2[n2] = normal1[j + 1];
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        }

        // !insideStart && !insideEnd: do nothing.
        insideStart = insideEnd;
    }
    return n2 >= 3;
}

template <uint32_t axis, bool side>
static inline bool sutherlandHodgmanPass(
    Vector4 *pos1, Point *world1, Normal *normal1, uint32_t n1,
    Vector4 *pos2, Point *world2, Normal *normal2, uint32_t &n2)
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
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        } else if (insideStart && !insideEnd) {
            clip<axis, side>(pos1[j], world1[j], normal1[j], pos1[j + 1], world1[j + 1], normal1[j + 1], pos2[n2], world2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        } else if (!insideStart && insideEnd) {
            clip<axis, side>(pos1[j + 1], world1[j + 1], normal1[j + 1], pos1[j], world1[j], normal1[j], pos2[n2], world2[n2], normal2[n2]);
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;

            pos2[n2] = pos1[j + 1];
            world2[n2] = world1[j + 1];
            normal2[n2] = normal1[j + 1];
            if (n2 == 0 || (pos2[n2] != pos2[n2 - 1] && pos2[n2] != pos2[0])) ++n2;
        }

        // !insideStart && !insideEnd: do nothing.
        insideStart = insideEnd;
    }
    return n2 >= 3;
}

void PrimaryPipeline::add(
    Vector4 *pos1, Point *world1, Normal *normal1,
    uint16_t meshId, uint16_t matId, uint32_t primId, bool noClip)
{
    for (uint32_t i = 0; i < 3; ++i) {
        pos1[i] = vpTrans.getMatrix() * pos1[i];
    }

    Vector4 pos2[kMaxClipVertexCount];
    Point world2[kMaxClipVertexCount];
    Normal normal2[kMaxClipVertexCount];

    uint32_t nin = 3;
    Vector4 *inPos = pos1;
    Point *inWorld = world1;
    Normal *inNormal = normal1;

    uint32_t nout = 0;
    Vector4 *outPos = pos2;
    Point *outWorld = world2;
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

        outPos[i].x = math::clamp(outPos[i].x, -1.0f, 1.0f);
        outPos[i].y = math::clamp(outPos[i].y, -1.0f, 1.0f);
        outPos[i].z = math::clamp(outPos[i].z, 0.0f, 1.0f);

        outPos[i].w = 1.0f / outPos[i].w;
    }

    for (uint32_t i = 0; i < nout - 2; ++i) {
        // Backface culling. Do it for every new triangle to avoid winding order flip due to rounding error.
        Vector2 e01(outPos[i + 1].x - outPos[0].x, outPos[i + 1].y - outPos[0].y);
        Vector2 e12(outPos[i + 2].x - outPos[i + 1].x, outPos[i + 2].y - outPos[i + 1].y);
        if (e01.x * e12.y - e01.y * e12.x <= 0.0f) {
            continue;
        }

        positions.push_back(Point(outPos[0].x, outPos[0].y, outPos[0].z));
        positions.push_back(Point(outPos[i + 1].x, outPos[i + 1].y, outPos[i + 1].z));
        positions.push_back(Point(outPos[i + 2].x, outPos[i + 2].y, outPos[i + 2].z));

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

void SecondaryPipeline::clear()
{
    positions.clear();
    queryTriIndices.clear();
}

void SecondaryPipeline::add(Vector4 *pos1, bool query, bool noClip)
{
    for (uint32_t i = 0; i < 3; ++i) {
        pos1[i] = vpTrans.getMatrix() * pos1[i];
    }

    Vector4 pos2[kMaxClipVertexCount];

    uint32_t nin = 3;
    Vector4 *inPos = pos1;

    uint32_t nout = 0;
    Vector4 *outPos = pos2;

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

        outPos[i].x = math::clamp(outPos[i].x, -1.0f, 1.0f);
        outPos[i].y = math::clamp(outPos[i].y, -1.0f, 1.0f);
        outPos[i].z = math::clamp(outPos[i].z, 0.0f, 1.0f);
    }

    for (uint32_t i = 0; i < nout - 2; ++i) {
        // Backface culling. Do it for every new triangle to avoid winding order flip due to rounding error.
        Vector2 e01(outPos[i + 1].x - outPos[0].x, outPos[i + 1].y - outPos[0].y);
        Vector2 e12(outPos[i + 2].x - outPos[i + 1].x, outPos[i + 2].y - outPos[i + 1].y);
        if (e01.x * e12.y - e01.y * e12.x <= 0.0f) {
            continue;
        }

        if (query) {
            queryTriIndices.push_back((uint32_t)positions.size() / 3);
        }

        positions.push_back(Point(outPos[0].x, outPos[0].y, outPos[0].z));
        positions.push_back(Point(outPos[i + 1].x, outPos[i + 1].y, outPos[i + 1].z));
        positions.push_back(Point(outPos[i + 2].x, outPos[i + 2].y, outPos[i + 2].z));
    }
}

void PrimaryPipeline::run(const Scene &scene, const BVH &bvh, const FrustumBVHIsectResult &isects)
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

            Vector4 pos[kMaxClipVertexCount];
            Point world[kMaxClipVertexCount];
            Normal normal[kMaxClipVertexCount];
            for (uint32_t v = 0; v < 3; ++v) {
                const Point &wp = mesh->getVertexPositions()[tri.idx[v]];
                world[v] = wp;
                pos[v] = Vector4(wp.x, wp.y, wp.z, 1.0f);
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

static Vector3 rayTriIsect(
    const Point &origin, const Vector3 &dir,
    const Point &v0, const Point &v1, const Point &v2)
{
    // Assume ray and tri must intersect.
    Vector3 coord;

    Vector3 v0v1 = v1 - v0;
    Vector3 v0v2 = v2 - v0;
    Vector3 pvec = cross(dir, v0v2);

    Float det = dot(v0v1, pvec);

    Float invDet = Float(1) / det;

    Vector3 tvec = origin - v0;
    coord.y = dot(tvec, pvec) * invDet;
    coord.y = math::clamp(coord.y, 0.0f, 1.0f);
    //if (u < 0 || u > 1) return false;

    Vector3 qvec = cross(tvec, v0v1);
    coord.z = dot(dir, qvec) * invDet;
    coord.z = math::clamp(coord.z, 0.0f, 1.0f);
    //if (v < 0 || u + v > 1) return false;

    coord.x = Float(1) - coord.y - coord.z;
    coord.x = math::clamp(coord.x, 0.0f, 1.0f);
    // Float t = dot(v0v2, qvec) * invDet;

    return coord;
}

Intersection PrimaryPipeline::buildItsFromSample(const vtrz::PointSample &entry, const Point &sensorPos, const Scene &scene) const
{
    Intersection its;
    // Irrelevant fields.
    its.time = Float(0.0);
    its.color = Spectrum(0.0);
    its.instance = nullptr;
    its.wi = Vector(0.0);
    // its.primitive

    // Useful fields.
    uint32_t meshIndex = meshIds[entry.triId];
    const TriMesh *mesh = scene.getMeshes()[meshIndex];
    its.shape = mesh;
    uint32_t primIndex = primIds[entry.triId];
    Triangle tri = scene.getMeshes()[meshIndex]->getTriangles()[primIndex];
    uint32_t offset = entry.triId * 3;

    Vector perspCoord(entry.coord.x, entry.coord.y, entry.coord.z);
    {
        Float denom(0.0);
        for (uint32_t i = 0; i < 3; ++i) {
            perspCoord[i] *= wInvs[entry.triId * 3 + i];
            denom += perspCoord[i];
        }
        perspCoord *= Float(1.0) / denom;
    }

    const Point &wp0 = worldPositions[offset + 0];
    const Point &wp1 = worldPositions[offset + 1];
    const Point &wp2 = worldPositions[offset + 2];

    its.p = wp0 * perspCoord[0] + wp1 * perspCoord[1] + wp2 * perspCoord[2];
    its.geoFrame = Frame(normalize(cross(wp1 - wp0, wp2 - wp1)));
    // TODO: this is now wrong?
    Vector3 coord3d = rayTriIsect(sensorPos, its.p - sensorPos, wp0, wp1, wp2);
    {
        Vector N;
        if (mesh->hasVertexNormals()) {
            const Normal &n0 = mesh->getVertexNormals()[tri.idx[0]];
            const Normal &n1 = mesh->getVertexNormals()[tri.idx[1]];
            const Normal &n2 = mesh->getVertexNormals()[tri.idx[2]];

            N = normalize(n0 * coord3d[0] + n1 * coord3d[1] + n2 * coord3d[2]);
        } else {
            N = its.geoFrame.n;
        }
        // Important: do not construct an arbitrary shading frame!
        // LTC requires the frame to be constructed this way.
        Vector V = normalize(sensorPos - its.p);
        Vector T1 = normalize(V - N * dot(V, N));
        Vector T2 = cross(N, T1);
        its.shFrame = Frame(T1, T2, N);
    }

    if (mesh->hasVertexTexcoords()) {
        const Point2 &tc0 = mesh->getVertexTexcoords()[tri.idx[0]];
        const Point2 &tc1 = mesh->getVertexTexcoords()[tri.idx[1]];
        const Point2 &tc2 = mesh->getVertexTexcoords()[tri.idx[2]];
        its.uv = tc0 * coord3d[0] + tc1 * coord3d[1] + tc2 * coord3d[2];
    } else {
        its.uv = Point2(0.0f);
    }

    if (mesh->hasUVTangents()) {
        its.dpdu = mesh->getUVTangents()[primIndex].dpdu;
        its.dpdv = mesh->getUVTangents()[primIndex].dpdv;
    } else {
        its.dpdu = its.dpdv = Vector(0.0);
    }

    its.hasUVPartials = false;
    return its;
}

bool PrimaryPipeline::buildItsPatch(const vtrz::LeafRegion &region, const Point &sensorPos, const Scene &scene, IntersectionPatch &patch) const
{
    patch.positions.clear();
    patch.normals.clear();
    patch.avgIts = {};

    uint32_t offset = region.triId * 3;
    Point p0(positions[offset + 0].x, positions[offset + 0].y, positions[offset + 0].z);
    Point p1(positions[offset + 1].x, positions[offset + 1].y, positions[offset + 1].z);
    Point p2(positions[offset + 2].x, positions[offset + 2].y, positions[offset + 2].z);
    vtrz::Tri2 tri2d = {
        vtrz::Vector2(p0.x, p0.y),
        vtrz::Vector2(p1.x, p1.y),
        vtrz::Vector2(p2.x, p2.y),
    };

    patch.positions.resize(region.vertCount);
    patch.coverage = 0.0f;
    for (uint32_t v = 0; v < region.vertCount; ++v) {
        vtrz::Vector2 vert = vtrz::cast(region.poly[v]);
        vtrz::Vector2 vnext = vtrz::cast(region.poly[(v + 1) % region.vertCount]);
        patch.coverage += vtrz::cross(vert, vnext);

        vtrz::Vector3 perspCoord = vtrz::barycentricCoordSafe(vert, tri2d);
        perspCoord.x *= wInvs[offset + 0];
        perspCoord.y *= wInvs[offset + 1];
        perspCoord.z *= wInvs[offset + 2];
        perspCoord /= (perspCoord.x + perspCoord.y + perspCoord.z);

        const Point &wp0 = worldPositions[offset + 0];
        const Point &wp1 = worldPositions[offset + 1];
        const Point &wp2 = worldPositions[offset + 2];

        patch.positions[v] = wp0 * perspCoord.x + wp1 * perspCoord.y + wp2 * perspCoord.z;
    }
    patch.coverage = patch.coverage * 0.5f * 0.25f; // TODO: invBoundArea = 1/4
    constexpr float kSmallCoverageThreshold = 1e-4;
    if (patch.coverage <= kSmallCoverageThreshold) {
        return false;
    }

    Intersection &its = patch.avgIts;
    // Irrelevant fields.
    its.time = Float(0.0);
    its.color = Spectrum(0.0);
    its.instance = nullptr;
    its.wi = Vector(0.0);

    // Useful fields: actually only position, face normal, uv, and dpduv.
    patch.meshId = meshIds[region.triId];
    const TriMesh *mesh = scene.getMeshes()[patch.meshId];
    its.shape = mesh;
    its.primIndex = primIds[region.triId];
    Triangle tri = scene.getMeshes()[patch.meshId]->getTriangles()[its.primIndex];

    Point centroid = Point(0.0f);
    for (uint32_t v = 0; v < region.vertCount; ++v) {
        centroid += patch.positions[v];
    }
    centroid /= (float)region.vertCount;
    its.p = centroid;

    const Point &wp0 = mesh->getVertexPositions()[tri.idx[0]];
    const Point &wp1 = mesh->getVertexPositions()[tri.idx[1]];
    const Point &wp2 = mesh->getVertexPositions()[tri.idx[2]];
    its.geoFrame = Frame(normalize(cross(wp1 - wp0, wp2 - wp1)));

    if (mesh->hasVertexNormals()) {
        patch.normals.resize(region.vertCount);
        for (uint32_t v = 0; v < region.vertCount; ++v) {
            Vector3 coord3d = rayTriIsect(sensorPos, patch.positions[v] - sensorPos, wp0, wp1, wp2);
            const Normal &n0 = mesh->getVertexNormals()[tri.idx[0]];
            const Normal &n1 = mesh->getVertexNormals()[tri.idx[1]];
            const Normal &n2 = mesh->getVertexNormals()[tri.idx[2]];
            patch.normals[v] = normalize(n0 * coord3d[0] + n1 * coord3d[1] + n2 * coord3d[2]);
        }
    }

    Vector3 centeroidCoord3d = rayTriIsect(sensorPos, centroid - sensorPos, wp0, wp1, wp2);

    if (mesh->hasVertexTexcoords()) {
        const Point2 &tc0 = mesh->getVertexTexcoords()[tri.idx[0]];
        const Point2 &tc1 = mesh->getVertexTexcoords()[tri.idx[1]];
        const Point2 &tc2 = mesh->getVertexTexcoords()[tri.idx[2]];
        its.uv = tc0 * centeroidCoord3d[0] + tc1 * centeroidCoord3d[1] + tc2 * centeroidCoord3d[2];
    } else {
        its.uv = Point2(0.0f);
    }

    if (mesh->hasUVTangents()) {
        its.dpdu = mesh->getUVTangents()[its.primIndex].dpdu;
        its.dpdv = mesh->getUVTangents()[its.primIndex].dpdv;
    } else {
        its.dpdu = its.dpdv = Vector(0.0);
    }

    its.hasUVPartials = false;

    return true;
}

void SecondaryPipeline::run(
    const Scene &scene, const BVH &bvh, const FrustumBVHIsectResult &isects, const TriMesh *queryMesh)
{
    uint32_t initCapacity = std::min(isects.totalPrimCount, 64u);
    positions.reserve(initCapacity);

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
            Vector4 pos[kMaxClipVertexCount];
            for (uint32_t v = 0; v < 3; ++v) {
                Point worldPos = mesh->getVertexPositions()[tri.idx[v]];
                pos[v] = Vector4(worldPos.x, worldPos.y, worldPos.z, 1.0f);
            }
            add(pos, mesh == queryMesh, entry.included);
        }
    }
}

void SecondaryPipeline::add(const IntersectionPatch &patch)
{
    for (uint32_t i = 0; i < (uint32_t)patch.positions.size() - 2; ++i) {
        if (backfaceCull(apex,
            patch.positions[0],
            patch.positions[i + 1],
            patch.positions[i + 2])) {
            continue;
        }
        Vector4 pos[kMaxClipVertexCount];
        pos[0] = Vector4(patch.positions[0].x, patch.positions[0].y, patch.positions[0].z, 1.0f);
        pos[1] = Vector4(patch.positions[i + 1].x, patch.positions[i + 1].y, patch.positions[i + 1].z, 1.0f);
        pos[2] = Vector4(patch.positions[i + 2].x, patch.positions[i + 2].y, patch.positions[i + 2].z, 1.0f);
        add(pos, false, false);
    }
}

void AABBClipper::run(const Point &apex, const AABB &aabb, const Transform &vpTrans, bool noClip)
{
    vertCount = 0;
    if (aabb.contains(apex)) {
        return;
    }

    Vector d0 = apex - aabb.min;
    if (d0.x < 0) {
        // face -x
        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.min.z),
            Point(aabb.min.x, aabb.min.y, aabb.max.z),
            Point(aabb.min.x, aabb.max.y, aabb.max.z) });

        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.min.z),
            Point(aabb.min.x, aabb.max.y, aabb.max.z),
            Point(aabb.min.x, aabb.max.y, aabb.min.z) });
    }
    if (d0.y < 0) {
        // face -y
        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.max.z),
            Point(aabb.max.x, aabb.min.y, aabb.min.z),
            Point(aabb.max.x, aabb.min.y, aabb.max.z) });

        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.max.z),
            Point(aabb.min.x, aabb.min.y, aabb.min.z),
            Point(aabb.max.x, aabb.min.y, aabb.min.z) });
    }
    if (d0.z < 0) {
        // face -z
        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.min.z),
            Point(aabb.max.x, aabb.max.y, aabb.min.z),
            Point(aabb.max.x, aabb.min.y, aabb.min.z) });

        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.min.z),
            Point(aabb.min.x, aabb.max.y, aabb.min.z),
            Point(aabb.max.x, aabb.max.y, aabb.min.z) });
    }

    Vector d1 = apex - aabb.max;
    if (d1.x > 0) {
        // face +x
        add(vpTrans, noClip, {
            Point(aabb.max.x, aabb.min.y, aabb.max.z),
            Point(aabb.max.x, aabb.min.y, aabb.min.z),
            Point(aabb.max.x, aabb.max.y, aabb.min.z) });

        add(vpTrans, noClip, {
            Point(aabb.max.x, aabb.min.y, aabb.max.z),
            Point(aabb.max.x, aabb.max.y, aabb.min.z),
            Point(aabb.max.x, aabb.max.y, aabb.max.z) });
    }
    if (d1.y > 0) {
        // face +y
        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.max.y, aabb.max.z),
            Point(aabb.max.x, aabb.max.y, aabb.max.z),
            Point(aabb.max.x, aabb.max.y, aabb.min.z) });

        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.max.y, aabb.max.z),
            Point(aabb.max.x, aabb.max.y, aabb.min.z),
            Point(aabb.min.x, aabb.max.y, aabb.min.z) });
    }
    if (d1.z > 0) {
        // face +z
        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.max.z),
            Point(aabb.max.x, aabb.min.y, aabb.max.z),
            Point(aabb.max.x, aabb.max.y, aabb.max.z) });

        add(vpTrans, noClip, {
            Point(aabb.min.x, aabb.min.y, aabb.max.z),
            Point(aabb.max.x, aabb.max.y, aabb.max.z),
            Point(aabb.min.x, aabb.max.y, aabb.max.z) });
    }
}

void AABBClipper::add(const Transform &vpTrans, bool noClip, const std::array<Point, 3> &face)
{
    Vector4 pos1[kMaxClipVertexCount];
    Vector4 pos2[kMaxClipVertexCount];

    uint32_t nin = 3;
    Vector4 *inPos = pos1;

    uint32_t nout = 0;
    Vector4 *outPos = pos2;

    for (uint32_t i = 0; i < 3; ++i) {
        inPos[i] = vpTrans.getMatrix() * Vector4(face[i].x, face[i].y, face[i].z, 1.0);
    }

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

        outPos[i].x = math::clamp(outPos[i].x, -1.0f, 1.0f);
        outPos[i].y = math::clamp(outPos[i].y, -1.0f, 1.0f);
        outPos[i].z = math::clamp(outPos[i].z, 0.0f, 1.0f);
    }

    for (uint32_t i = 0; i < nout - 2; ++i) {
        // Backface culling. Do it for every new triangle to avoid winding order flip due to rounding error.
        Vector2 e01(outPos[i + 1].x - outPos[0].x, outPos[i + 1].y - outPos[0].y);
        Vector2 e12(outPos[i + 2].x - outPos[i + 1].x, outPos[i + 2].y - outPos[i + 1].y);
        if (e01.x * e12.y - e01.y * e12.x <= 0.0f) {
            continue;
        }

        positions[vertCount++] = Point(outPos[0].x, outPos[0].y, outPos[0].z);
        positions[vertCount++] = Point(outPos[i + 1].x, outPos[i + 1].y, outPos[i + 1].z);
        positions[vertCount++] = Point(outPos[i + 2].x, outPos[i + 2].y, outPos[i + 2].z);
    }
}

void SecondaryPipelineInc::clear()
{
    positions.clear();
}

void SecondaryPipelineInc::add(Vector4 *pos1, bool noClip)
{
    for (uint32_t i = 0; i < 3; ++i) {
        pos1[i] = vpTrans.getMatrix() * pos1[i];
    }

    Vector4 pos2[kMaxClipVertexCount];

    uint32_t nin = 3;
    Vector4 *inPos = pos1;

    uint32_t nout = 0;
    Vector4 *outPos = pos2;

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

        outPos[i].x = math::clamp(outPos[i].x, -1.0f, 1.0f);
        outPos[i].y = math::clamp(outPos[i].y, -1.0f, 1.0f);
        outPos[i].z = math::clamp(outPos[i].z, 0.0f, 1.0f);
    }

    for (uint32_t i = 0; i < nout - 2; ++i) {
        // Backface culling. Do it for every new triangle to avoid winding order flip due to rounding error.
        Vector2 e01(outPos[i + 1].x - outPos[0].x, outPos[i + 1].y - outPos[0].y);
        Vector2 e12(outPos[i + 2].x - outPos[i + 1].x, outPos[i + 2].y - outPos[i + 1].y);
        if (e01.x * e12.y - e01.y * e12.x <= 0.0f) {
            continue;
        }

        positions.push_back(Point(outPos[0].x, outPos[0].y, outPos[0].z));
        positions.push_back(Point(outPos[i + 1].x, outPos[i + 1].y, outPos[i + 1].z));
        positions.push_back(Point(outPos[i + 2].x, outPos[i + 2].y, outPos[i + 2].z));
    }
}

void SecondaryPipelineInc::run(const TriMesh *queryMesh)
{
    for (uint32_t i = 0; i < queryMesh->getTriangleCount(); ++i) {
        Triangle tri = queryMesh->getTriangles()[i];
        if (backfaceCull(apex,
            queryMesh->getVertexPositions()[tri.idx[0]],
            queryMesh->getVertexPositions()[tri.idx[1]],
            queryMesh->getVertexPositions()[tri.idx[2]])) {
            continue;
        }
        Vector4 pos[kMaxClipVertexCount];
        for (uint32_t v = 0; v < 3; ++v) {
            Point worldPos = queryMesh->getVertexPositions()[tri.idx[v]];
            pos[v] = Vector4(worldPos.x, worldPos.y, worldPos.z, 1.0f);
        }
        add(pos, false);
    }
}

void SecondaryPipelineInc::run(
    const Scene &scene, const BVH &bvh, const FrustumBVHIsectEntry &entry, const TriMesh *queryMesh)
{
    const std::vector<Primitive> &primitives = bvh.primitives();
    for (uint32_t i = 0; i < entry.primCount; ++i) {
        const Primitive &prim = primitives[entry.primOffset + i];
        const TriMesh *mesh = scene.getMeshes()[prim.meshIndex];
        if (mesh == queryMesh) {
            continue;
        }
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
        Vector4 pos[kMaxClipVertexCount];
        for (uint32_t v = 0; v < 3; ++v) {
            Point worldPos = mesh->getVertexPositions()[tri.idx[v]];
            pos[v] = Vector4(worldPos.x, worldPos.y, worldPos.z, 1.0f);
        }
        add(pos, entry.included);
    }
}

MTS_NAMESPACE_END
