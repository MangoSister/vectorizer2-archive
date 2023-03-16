#include "util.h"
#include "bcone.h"
#include <fstream>
#include <cstdio>
#include <boost/filesystem/path.hpp>

MTS_NAMESPACE_BEGIN

void dumpVectorizerInput(
    const Scene *scene, const Point2i &pixel,
    const AABB2 &bound,
    const std::vector<Point> &vertexBuffer,
    const std::vector<uint32_t> *indexBuffer,
    const std::vector<uint32_t> *queryTriIndices,
    const std::vector<vtrz::DVector2> *vertexGradBuffer)
{
    fs::path sourceStem = scene->getSourceFile().stem();

    char *name = new char[256];
    snprintf(name, 256, "%s-%d-%d.data", sourceStem.string().c_str(), pixel.x, pixel.y);
    std::ofstream file(name, std::ios::binary);
    SAssert(file);

    size_t bufSize = sizeof(AABB2) + sizeof(uint32_t) + sizeof(Point) * vertexBuffer.size();
    bufSize += sizeof(uint32_t);
    if (indexBuffer) {
        bufSize += sizeof(uint32_t) * indexBuffer->size();
    }
    bufSize += sizeof(uint32_t);
    if (queryTriIndices) {
        bufSize += sizeof(uint32_t) * queryTriIndices->size();
    }
    bufSize += sizeof(uint32_t);
    if (vertexGradBuffer) {
        bufSize += sizeof(vtrz::DVector2) * vertexGradBuffer->size();
    }
    char *buf = new char[bufSize];

    size_t offset = 0;
    memcpy(buf + offset, &bound, sizeof(AABB2));
    offset += sizeof(AABB2);

    uint32_t vertexCount = (uint32_t)vertexBuffer.size();
    memcpy(buf + offset, &vertexCount, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    memcpy(buf + offset, vertexBuffer.data(), sizeof(Point) * vertexCount);
    offset += sizeof(Point) * vertexCount;

    if (indexBuffer) {
        uint32_t indexCount = (uint32_t)indexBuffer->size();
        memcpy(buf + offset, &indexCount, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        memcpy(buf + offset, indexBuffer->data(), sizeof(uint32_t) * indexCount);
        offset += sizeof(uint32_t) * indexCount;
    } else {
        uint32_t indexCount = 0;
        memcpy(buf + offset, &indexCount, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }


    if (queryTriIndices) {
        uint32_t queryCount = (uint32_t)queryTriIndices->size();
        memcpy(buf + offset, &queryCount, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        memcpy(buf + offset, queryTriIndices->data(), sizeof(uint32_t) * queryCount);
        offset += sizeof(uint32_t) * queryCount;
    } else {
        uint32_t queryCount = 0;
        memcpy(buf + offset, &queryCount, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }

    if (vertexGradBuffer) {
        memcpy(buf + offset, &vertexCount, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        memcpy(buf + offset, vertexGradBuffer->data(), sizeof(vtrz::DVector2) * vertexCount);
        offset += sizeof(vtrz::DVector2) * vertexCount;
    } else {
        uint32_t zero = 0;
        memcpy(buf + offset, &zero, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }

    file.write(buf, bufSize);

    file.close();

    delete[] name;
    delete[] buf;
}

ViewInfo::ViewInfo(const Sensor &sensor) :
    persp(static_cast<const PerspectiveCamera &>(sensor))
{
    resolution = Vector2(sensor.getFilm()->getCropSize());
    viewTrans = persp.getViewTransform(0.0); // Don't care about motion blur for now.
    // Flip x and z so that +x points right, and +z points outward.
    Matrix4x4 flipViewMtx;
    flipViewMtx.setIdentity();
    flipViewMtx.m[0][0] = -1.0;
    flipViewMtx.m[2][2] = -1.0;
    viewTrans = Transform(flipViewMtx) * viewTrans;

    sensorPos = Point(viewTrans.getInverseMatrix().m[0][3],
        viewTrans.getInverseMatrix().m[1][3],
        viewTrans.getInverseMatrix().m[2][3]);

    nearClip = persp.getNearClip();
    vFov = degToRad(persp.getYFov());
    aspect = std::tan(0.5f * degToRad(persp.getXFov())) / std::tan(0.5f * degToRad(persp.getYFov()));
}

Beam ViewInfo::createPrimaryBeam(const Point2i &pixel, bool revInfProj) const
{
    Beam beam;
    beam.viewTrans = viewTrans;
    AABB2 bound;

    bound.min.x = (Float(2.0f) * (pixel.x)) / resolution.x - Float(1.0f);
    bound.max.x = (Float(2.0f) * ((pixel.x) + Float(1.0))) / resolution.x - Float(1.0f);
    bound.min.y = -(Float(2.0f) * ((pixel.y) + Float(1.0))) / resolution.y + Float(1.0f);
    bound.max.y = -(Float(2.0f) * (pixel.y)) / resolution.y + Float(1.0f);
    if (revInfProj) {
        beam.projTrans = Transform(revInfPerspective(nearClip, vFov, aspect, bound));
    } else {
        beam.projTrans = Transform(glPerspective(nearClip, 1000.0f, vFov, aspect, bound));
    }

    beam.frustum = persp.buildFrustum(pixel);
    return beam;
}

bool createSecondaryBeam(const Intersection &its, const Emitter &emitter, Beam &beam)
{
    Point beamOrigin = its.p + its.geoFrame.n * kBeamOriginEps;

    const TriMesh *emitterMesh = (const TriMesh *)emitter.getShape();
    // Calculate minimal beam from mesh. Even if the calculation is expensive, it saves much more rendering time than
    // creating a conservative beam from bounding sphere.
    std::vector<Vector> directions(emitterMesh->getVertexCount());
    for (uint32_t i = 0; i < emitterMesh->getVertexCount(); ++i) {
        Vector d = (emitterMesh->getVertexPositions()[i] - beamOrigin);
        directions[i] = normalize(d);
    }
    SphericalCircle cone = minBoundingCone(directions.data(), (uint32_t)directions.size());

    Vector beamCenterDir = cone.n;
    Float cos2 = cone.cosHalfAngle * cone.cosHalfAngle;
    Float tan = std::sqrt((1 - cos2) / cos2);
    Float beamSideLen = tan;
    Float beamFov = Float(2.0) * std::acos(cone.cosHalfAngle);

    Frame viewFrame;
    Vector viewXAxis = cross(its.shFrame.n, -beamCenterDir);
    if (viewXAxis.lengthSquared() == 0.0) {
        viewFrame = Frame(-beamCenterDir);
    } else {
        viewXAxis = normalize(viewXAxis);
        Vector viewYAxis = cross(-beamCenterDir, viewXAxis);
        SAssert(dot(cross(viewXAxis, viewYAxis), -beamCenterDir) > 0.0);
        viewFrame = Frame(viewXAxis, viewYAxis, -beamCenterDir);
    }
    beam.viewTrans = Transform::fromFrame(viewFrame).inverse() * Transform::translate(-(Vector)beamOrigin);

    Vector viewS = viewFrame.s * beamSideLen;
    Vector viewT = viewFrame.t * beamSideLen;
    Float d0 = dot(its.shFrame.n, (beamCenterDir - viewT));
    Float d1 = dot(its.shFrame.n, (beamCenterDir + viewT));

    Vector frustumCornerDirs[4];
    AABB2 vectBound(Point2(-1.0f), Point2(1.0f)); // or offset the projection matrix?
    if (d0 <= 0.0 && d1 <= 0.0) {
        // entire frustum under horizon.
        return false;
    } else if (d0 >= 0.0 && d1 >= 0.0) {
        // entire emitter frustum above horizon.
        frustumCornerDirs[0] = beamCenterDir - viewS - viewT;
        frustumCornerDirs[1] = beamCenterDir + viewS - viewT;
        frustumCornerDirs[2] = beamCenterDir + viewS + viewT;
        frustumCornerDirs[3] = beamCenterDir - viewS + viewT;
    } else {
        Float tClip = math::lerp(d0 / (d0 - d1), Float(-1.0), Float(1.0));
        if (d0 < 0) {
            frustumCornerDirs[0] = beamCenterDir - viewS + tClip * viewT;
            frustumCornerDirs[1] = beamCenterDir + viewS + tClip * viewT;
            frustumCornerDirs[2] = beamCenterDir + viewS + viewT;
            frustumCornerDirs[3] = beamCenterDir - viewS + viewT;
            vectBound.min.y = tClip;
        } else {
            frustumCornerDirs[0] = beamCenterDir - viewS - viewT;
            frustumCornerDirs[1] = beamCenterDir + viewS - viewT;
            frustumCornerDirs[2] = beamCenterDir + viewS + tClip * viewT;
            frustumCornerDirs[3] = beamCenterDir - viewS + tClip * viewT;
            vectBound.max.y = tClip;
        }
        SAssert(dot(beamCenterDir, cross(frustumCornerDirs[1] - frustumCornerDirs[0], frustumCornerDirs[2] - frustumCornerDirs[1])) < 0);
        SAssert(dot(beamCenterDir, cross(frustumCornerDirs[2] - frustumCornerDirs[0], frustumCornerDirs[3] - frustumCornerDirs[2])) < 0);
    }

    beam.frustum = Frustum(beamOrigin, beamCenterDir, frustumCornerDirs, kSecondaryNearClip, FrustumAABBOption::eCoarse);

    // Float far = shadingToEmitterDist + emitterBSphere.radius;
    // TODO: far clip is actually useful here.
    beam.projTrans = Transform(revInfPerspective(kSecondaryNearClip, beamFov, 1.0f, vectBound));

    return true;
}

bool createSecondaryBeam(const IntersectionPatch &patch, const Point &lightPos, Beam &beam)
{
    if (dot(lightPos - patch.positions[0], patch.avgIts.geoFrame.n) <= 0.0) {
        return false;
    }

    Point beamOrigin = lightPos;
    // Calculate minimal beam from mesh. Even if the calculation is expensive, it saves much more rendering time than
    // creating a conservative beam from bounding sphere.
    std::vector<Vector> directions(patch.positions.size());
    for (uint32_t i = 0; i < (uint32_t)patch.positions.size(); ++i) {
        Vector d = (patch.positions[i] - beamOrigin);
        directions[i] = normalize(d);
    }
    SphericalCircle cone = minBoundingCone(directions.data(), (uint32_t)directions.size());

    Vector beamCenterDir = cone.n;
    cone.cosHalfAngle = std::min(cone.cosHalfAngle, kMaxBeamCosHalfAngle);
    Float cos2 = cone.cosHalfAngle * cone.cosHalfAngle;
    SAssert(cos2 >= 0.0f && cos2 <= 1.0f);
    Float tan = std::sqrt((1 - cos2) / cos2);
    Float beamSideLen = tan;
    SAssert(beamSideLen > 0.0f);
    Float beamFov = Float(2.0) * std::acos(cone.cosHalfAngle);
    SAssert(beamFov > 0.0f);

    Frame viewFrame = Frame(-beamCenterDir);
    beam.viewTrans = Transform::fromFrame(viewFrame).inverse() * Transform::translate(-(Vector)beamOrigin);

    Vector frustumCornerDirs[4];
    AABB2 vectBound(Point2(-1.0f), Point2(1.0f)); // or offset the projection matrix?
    Vector viewS = viewFrame.s * beamSideLen;
    Vector viewT = viewFrame.t * beamSideLen;
    frustumCornerDirs[0] = beamCenterDir - viewS - viewT;
    frustumCornerDirs[1] = beamCenterDir + viewS - viewT;
    frustumCornerDirs[2] = beamCenterDir + viewS + viewT;
    frustumCornerDirs[3] = beamCenterDir - viewS + viewT;

    beam.frustum = Frustum(beamOrigin, beamCenterDir, frustumCornerDirs, kSecondaryNearClip, FrustumAABBOption::eCoarse);

    // Float far = shadingToEmitterDist + emitterBSphere.radius;
    // TODO: far clip is actually useful here.
    beam.projTrans = Transform(revInfPerspective(kSecondaryNearClip, beamFov, 1.0f, vectBound));

    return true;
}

Beam createSecondaryBeamOblique(const Point &origin, const Point *patch, uint32_t vertCount)
{
    Beam beam;

    // Calculate minimal beam from mesh. Even if the calculation is expensive, it saves much more rendering time than
    // creating a conservative beam from bounding sphere.
    std::vector<Vector> directions(vertCount);
    for (uint32_t i = 0; i < vertCount; ++i) {
        Vector d = patch[i] - origin;
        directions[i] = normalize(d);
    }
    SphericalCircle cone = minBoundingCone(directions.data(), (uint32_t)directions.size());

    Vector beamCenterDir = cone.n;
    cone.cosHalfAngle = std::min(cone.cosHalfAngle, 0.9995f);
    Float cos2 = cone.cosHalfAngle * cone.cosHalfAngle;
    SAssert(cos2 >= 0.0f && cos2 <= 1.0f);
    Float tan = std::sqrt((1 - cos2) / cos2);
    Float beamSideLen = tan;
    SAssert(beamSideLen > 0.0f);
    Float beamFov = Float(2.0) * std::acos(cone.cosHalfAngle);
    SAssert(beamFov > 0.0f);

    Frame viewFrame = Frame(-beamCenterDir);
    beam.viewTrans = Transform::fromFrame(viewFrame).inverse() * Transform::translate(-(Vector)origin);

    Vector frustumCornerDirs[4];
    AABB2 vectBound(Point2(-1.0f), Point2(1.0f)); // or offset the projection matrix?
    Vector viewS = viewFrame.s * beamSideLen;
    Vector viewT = viewFrame.t * beamSideLen;
    frustumCornerDirs[0] = beamCenterDir - viewS - viewT;
    frustumCornerDirs[1] = beamCenterDir + viewS - viewT;
    frustumCornerDirs[2] = beamCenterDir + viewS + viewT;
    frustumCornerDirs[3] = beamCenterDir - viewS + viewT;

    beam.frustum = Frustum(origin, beamCenterDir, frustumCornerDirs, kSecondaryNearClip, FrustumAABBOption::eCoarse);

    // Float far = shadingToEmitterDist + emitterBSphere.radius;
    // TODO: far clip is actually useful here.
    beam.projTrans = Transform(revInfPerspective(kSecondaryNearClip, beamFov, 1.0f, vectBound));
    // beam.projTrans = Transform(glPerspective(kSecondaryNearClip, 100.0f, beamFov, 1.0f, vectBound));

    //// http://www.terathon.com/code/oblique.html
    //Normal n = normalize(cross(patch[1] - patch[0], patch[2] - patch[1]));
    //Normal viewN = beam.viewTrans(n);
    //Point viewP = beam.viewTrans(patch[0]);

    //constexpr Float obliqueEps(1e-4);
    //Vector4 clipPlane(viewN.x, viewN.y, viewN.z, -dot(viewN, Vector(viewP)) - obliqueEps);

    //Vector4 q(sgn(clipPlane.x), sgn(clipPlane.y), 1.0f, 1.0f);

    //q = beam.projTrans.getInverseMatrix() * q;
    //Vector4 row = (-2.0f * q.z) / dot(clipPlane, q) * clipPlane + Vector4(0.0, 0.0, 1.0, 0.0);
    //Matrix4x4 newProjMatrix = beam.projTrans.getMatrix();
    //newProjMatrix.m[2][0] = row.x; newProjMatrix.m[2][1] = row.y; newProjMatrix.m[2][2] = row.z; newProjMatrix.m[2][3] = row.w;

    ////Float cq = dot(clipPlane, q);
    ////Float a = 2.0f * dot(beam.projTrans.getMatrix().row(3), q) / dot(clipPlane, q);
    ////Vector4 ac = newProjMatrix.row(2) + newProjMatrix.row(3);

    //beam.projTrans = Transform(newProjMatrix);
    return beam;
}

bool createSecondaryBeamGrad(const IntersectionGrad &itsGrad, const Emitter &emitter, BeamGrad &beam)
{
    vtrz::DVector3 beamOriginGrad = itsGrad.dp;
    beamOriginGrad.x.value += itsGrad.its.geoFrame.n.x * kBeamOriginEps;
    beamOriginGrad.y.value += itsGrad.its.geoFrame.n.y * kBeamOriginEps;
    beamOriginGrad.z.value += itsGrad.its.geoFrame.n.z * kBeamOriginEps;
    Point beamOrigin(beamOriginGrad.x.value, beamOriginGrad.y.value, beamOriginGrad.z.value);

    const TriMesh *emitterMesh = (const TriMesh *)emitter.getShape();
    // Calculate minimal beam from mesh. Even if the calculation is expensive, it saves much more rendering time than
    // creating a conservative beam from bounding sphere.
    std::vector<Vector> directions(emitterMesh->getVertexCount());
    for (uint32_t i = 0; i < emitterMesh->getVertexCount(); ++i) {
        Vector d = (emitterMesh->getVertexPositions()[i] - beamOrigin);
        directions[i] = normalize(d);
    }
    SphericalCircle cone = minBoundingCone(directions.data(), (uint32_t)directions.size());

    Vector beamCenterDir = cone.n;
    Float cos2 = cone.cosHalfAngle * cone.cosHalfAngle;
    Float tan = std::sqrt((1 - cos2) / cos2);
    // Avoid edge cases by slightly enlarge the frustum.
    constexpr Float kEnlargeFactor(0.999);
    tan /= kEnlargeFactor;
    Float beamSideLen = tan;
    Float beamFov = Float(2.0) * std::atan(tan);

    Frame viewFrame;
    Vector viewXAxis = cross(itsGrad.its.shFrame.n, -beamCenterDir);
    if (viewXAxis.lengthSquared() == 0.0) {
        viewFrame = Frame(-beamCenterDir);
    } else {
        viewXAxis = normalize(viewXAxis);
        Vector viewYAxis = cross(-beamCenterDir, viewXAxis);
        SAssert(dot(cross(viewXAxis, viewYAxis), -beamCenterDir) > 0.0);
        viewFrame = Frame(viewXAxis, viewYAxis, -beamCenterDir);
    }
    beam.viewTrans = fromFrame(viewFrame).transpose() * vtrz::makeTranslate(-beamOriginGrad.x, -beamOriginGrad.y, -beamOriginGrad.z);

    Vector viewS = viewFrame.s * beamSideLen;
    Vector viewT = viewFrame.t * beamSideLen;
    Float d0 = dot(itsGrad.its.shFrame.n, (beamCenterDir - viewT));
    Float d1 = dot(itsGrad.its.shFrame.n, (beamCenterDir + viewT));

    Vector frustumCornerDirs[4];
    AABB2 vectBound(Point2(-1.0f), Point2(1.0f)); // or offset the projection matrix?
    if (d0 <= 0.0 && d1 <= 0.0) {
        // entire frustum under horizon.
        return false;
    } else if (d0 >= 0.0 && d1 >= 0.0) {
        // entire emitter frustum above horizon.
        frustumCornerDirs[0] = beamCenterDir - viewS - viewT;
        frustumCornerDirs[1] = beamCenterDir + viewS - viewT;
        frustumCornerDirs[2] = beamCenterDir + viewS + viewT;
        frustumCornerDirs[3] = beamCenterDir - viewS + viewT;
    } else {
        Float tClip = math::lerp(d0 / (d0 - d1), Float(-1.0), Float(1.0));
        if (d0 < 0) {
            frustumCornerDirs[0] = beamCenterDir - viewS + tClip * viewT;
            frustumCornerDirs[1] = beamCenterDir + viewS + tClip * viewT;
            frustumCornerDirs[2] = beamCenterDir + viewS + viewT;
            frustumCornerDirs[3] = beamCenterDir - viewS + viewT;
            vectBound.min.y = tClip;
        } else {
            frustumCornerDirs[0] = beamCenterDir - viewS - viewT;
            frustumCornerDirs[1] = beamCenterDir + viewS - viewT;
            frustumCornerDirs[2] = beamCenterDir + viewS + tClip * viewT;
            frustumCornerDirs[3] = beamCenterDir - viewS + tClip * viewT;
            vectBound.max.y = tClip;
        }

        Vector cr1 = cross(frustumCornerDirs[1] - frustumCornerDirs[0], frustumCornerDirs[2] - frustumCornerDirs[1]);
        Vector cr2 = cross(frustumCornerDirs[2] - frustumCornerDirs[0], frustumCornerDirs[3] - frustumCornerDirs[2]);
        if (cr1.isZero() || cr2.isZero()) {
            // tClip almost -1 or 1. The frustum is rounded to zero volume.
            return false;
        }
        SAssert(dot(beamCenterDir, cr1) < 0);
        SAssert(dot(beamCenterDir, cr2) < 0);
    }

    beam.frustum = Frustum(beamOrigin, beamCenterDir, frustumCornerDirs, kSecondaryNearClip, FrustumAABBOption::eCoarse);

    // Float far = shadingToEmitterDist + emitterBSphere.radius;
    // TODO: far clip is actually useful here.
    beam.projTrans = castMatrix4x4(revInfPerspective(kSecondaryNearClip, beamFov, 1.0f, vectBound));

    return true;
}

MTS_NAMESPACE_END