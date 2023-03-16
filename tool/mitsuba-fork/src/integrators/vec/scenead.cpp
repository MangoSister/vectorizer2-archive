#include "scenead.h"
#include "util.h"
#include <mitsuba/core/fstream.h>
#include <mitsuba/render/trimesh.h>

MTS_NAMESPACE_BEGIN

TriMeshAD::TriMeshAD(const vtrz::DMatrix4x4 &transAD, TransformADType type, TriMesh &mesh) :
    type(TriMeshADType::eTransformAD), mesh(&mesh)
{
    uint32_t vertCount = (uint32_t)mesh.getVertexCount();
    const Point *pos = mesh.getVertexPositions();
    positions.resize(vertCount);
    if (type == TransformADType::eWorld) {
        for (uint32_t v = 0; v < vertCount; ++v) {
            for (uint32_t i = 0; i < 3; ++i) {
                positions[v][i].value = pos[v][i];
                for (uint32_t d = 0; d < vtrz::kMaxGradDim; ++d) {
                    positions[v][i].grad[d] =
                        transAD.arr[i][0].grad[d] * pos[v].x +
                        transAD.arr[i][1].grad[d] * pos[v].y +
                        transAD.arr[i][2].grad[d] * pos[v].z +
                        transAD.arr[i][3].grad[d];
                }
            }
        }
    } else {
        const Properties &props = mesh.getProperties();
        Transform toWorld = props.getTransform("toWorld");
        vtrz::DMatrix4x4 toWorldAD = castMatrix4x4(toWorld.getMatrix()) * transAD;
        Transform toLocal = toWorld.inverse();
        for (uint32_t v = 0; v < vertCount; ++v) {
            Point local = toLocal(pos[v]);
            positions[v] = vtrz::DVector3(toWorldAD * vtrz::DVector4(local.x, local.y, local.z, 1.0f));
        }
    }
}

TriMeshAD::TriMeshAD(const fs::path &vertexJacobianPath, TriMesh &mesh) :
    type(TriMeshADType::ePerVertexAD), mesh(&mesh)
{
    ref<FileStream> file = new FileStream(vertexJacobianPath);

    uint32_t vertCount = 0;
    uint32_t derCount = 0;
    file->read(&vertCount, sizeof(uint32_t));
    file->read(&derCount, sizeof(uint32_t));
    SAssert(vertCount == mesh.getVertexCount());
    SAssert(derCount == vtrz::kMaxGradDim);

    positions.resize(vertCount);
    const Point *pos = mesh.getVertexPositions();
    for (uint32_t i = 0; i < vertCount; ++i) {
        positions[i].x.value = pos[i].x;
        positions[i].y.value = pos[i].y;
        positions[i].z.value = pos[i].z;

        file->read(positions[i].x.grad.data(), sizeof(float) * derCount);
        file->read(positions[i].y.grad.data(), sizeof(float) * derCount);
        file->read(positions[i].z.grad.data(), sizeof(float) * derCount);
    }
    SAssert(file->getPos() == file->getSize());

    if (mesh.hasVertexNormals()) {
        computeSmoothNormals();
    } else {
        computeFaceNormals();
    }
}

TriMeshAD::TriMeshAD(const std::vector<float> &vertexJacobian, TriMesh &mesh) :
    type(TriMeshADType::ePerVertexAD), mesh(&mesh)
{
    uint32_t vertCount = (uint32_t)mesh.getVertexCount();
    SAssert(vertexJacobian.size() == 3 * vertCount * vertCount);
    SAssert(vertCount == vtrz::kMaxGradDim);
    const Point *pos = mesh.getVertexPositions();

    positions.resize(vertCount);
    for (uint32_t i = 0; i < vertCount; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            positions[i][j].value = pos[i][j];
            memcpy(positions[i][j].grad.data(),
                &vertexJacobian[(3 * i + j) * vertCount], sizeof(float) * vertCount);
        }
    }

    if (mesh.hasVertexNormals()) {
        computeSmoothNormals();
    } else {
        computeFaceNormals();
    }
}

void TriMeshAD::computeSmoothNormals()
{
    // Copied from trimesh.cpp

    SAssert(mesh->hasVertexNormals());
    uint32_t vertCount = (uint32_t)mesh->getVertexCount();
    normals.resize(vertCount);
    for (uint32_t i = 0; i < vertCount; ++i) {
        normals[i] = vtrz::DVector3(0.0f);
    }

    uint32_t triCount = (uint32_t)mesh->getTriangleCount();
    const Triangle *tris = mesh->getTriangles();
    for (uint32_t i = 0; i < triCount; ++i) {
        const Triangle &tri = tris[i];
        vtrz::DVector3 n(0.0f);
        for (int i = 0; i < 3; ++i) {
            const vtrz::DVector3 &v0 = positions[tri.idx[i]];
            const vtrz::DVector3 &v1 = positions[tri.idx[(i + 1) % 3]];
            const vtrz::DVector3 &v2 = positions[tri.idx[(i + 2) % 3]];
            vtrz::DVector3 sideA(v1 - v0), sideB(v2 - v0);
            if (i == 0) {
                n = vtrz::cross(sideA, sideB);
                vtrz::dfloat length = vtrz::length(n);
                if (length.value == 0)
                    break;
                n /= length;
            }
            vtrz::dfloat angle = vtrz::unitAngle(vtrz::normalize(sideA), vtrz::normalize(sideB));
            normals[tri.idx[i]] += n * angle;
        }
    }

    for (size_t i = 0; i < vertCount; i++) {
        vtrz::DVector3 &n = normals[i];
        vtrz::dfloat length = vtrz::length(n);
        SAssert(length.value > 0.0f);
        n /= length;
    }
}

void TriMeshAD::computeFaceNormals()
{
    SAssert(!mesh->hasVertexNormals());

    uint32_t triCount = (uint32_t)mesh->getTriangleCount();
    const Triangle *tris = mesh->getTriangles();
    normals.resize(triCount);
    for (uint32_t i = 0; i < triCount; ++i) {
        const Triangle &tri = tris[i];
        const vtrz::DVector3 &p0 = positions[tri.idx[0]];
        const vtrz::DVector3 &p1 = positions[tri.idx[1]];
        const vtrz::DVector3 &p2 = positions[tri.idx[2]];
        normals[i] = vtrz::normalize(vtrz::cross(p1 - p0, p2 - p1));
    }
}

TriMeshAD SimpleMeshDisplacement::constant(float t, TriMesh &mesh, std::vector<float> &coefficients)
{
    Point center = mesh.getAABB().getCenter();
    uint32_t vertCount = (uint32_t)mesh.getVertexCount();
    Point *pos = mesh.getVertexPositions();
    std::vector<float> jacobian(3 * vertCount * vertCount, 0.0f);
    coefficients.resize(vertCount);
    std::fill(coefficients.begin(), coefficients.end(), t);
    for (uint32_t i = 0; i < vertCount; ++i) {
        Vector d = pos[i] - center;
        SAssert(d.lengthSquared() > 0.0f);
        d = normalize(d);
        pos[i] += d * t;
        for (uint32_t j = 0; j < 3; ++j) {
            jacobian[(3 * i + j) * vertCount + i] = d[j];
        }
    }

    mesh.computeNormals(true);

    return TriMeshAD(jacobian, mesh);
}

TriMeshAD SimpleMeshDisplacement::sine(float t, float fphi, float ftheta, TriMesh &mesh, std::vector<float> &coefficients)
{
    Point center = mesh.getAABB().getCenter();
    uint32_t vertCount = (uint32_t)mesh.getVertexCount();
    Point *pos = mesh.getVertexPositions();
    std::vector<float> jacobian(3 * vertCount * vertCount, 0.0f);
    coefficients.resize(vertCount);
    for (uint32_t i = 0; i < vertCount; ++i) {
        Vector d = pos[i] - center;
        SAssert(d.lengthSquared() > 0.0f);
        d = normalize(d);
        Point2 sph = toSphericalCoordinates(d);
        float theta = sph.x;
        float phi = sph.y;
        coefficients[i] = t * std::sin(phi * fphi) * std::sin(theta * ftheta);
        pos[i] += d * coefficients[i];
        for (uint32_t j = 0; j < 3; ++j) {
            jacobian[(3 * i + j) * vertCount + i] = d[j];
        }
    }
    mesh.computeNormals(true);

    return TriMeshAD(jacobian, mesh);
}

TriMeshAD SimpleMeshDisplacement::fromData(const fs::path &path, TriMesh &mesh, std::vector<float> &coefficients)
{
    Point center = mesh.getAABB().getCenter();
    uint32_t vertCount = (uint32_t)mesh.getVertexCount();

    ref<FileStream> file = new FileStream(path);
    coefficients.resize(vertCount);
    uint32_t coeffsCount = file->readUInt();
    SAssert(coeffsCount == vertCount);
    file->readFloatArray(coefficients.data(), vertCount);
    SAssert(file->getPos() == file->getSize());

    Point *pos = mesh.getVertexPositions();
    std::vector<float> jacobian(3 * vertCount * vertCount, 0.0f);
    for (uint32_t i = 0; i < vertCount; ++i) {
        Vector d = pos[i] - center;
        SAssert(d.lengthSquared() > 0.0f);
        d = normalize(d);
        Point2 sph = toSphericalCoordinates(d);
        float theta = sph.x;
        float phi = sph.y;
        pos[i] += d * coefficients[i];
        for (uint32_t j = 0; j < 3; ++j) {
            jacobian[(3 * i + j) * vertCount + i] = d[j];
        }
    }
    mesh.computeNormals(true);

    return TriMeshAD(jacobian, mesh);
}

MTS_NAMESPACE_END
