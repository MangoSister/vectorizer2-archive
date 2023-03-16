#pragma once

#include <mitsuba/mitsuba.h>
#include <vectorizer/vectorizer.h>
#include <vector>
#include <unordered_map>

MTS_NAMESPACE_BEGIN

enum class TriMeshADType
{
    eTransformAD,
    ePerVertexAD,
};

enum class TransformADType
{
    eLocal,
    eWorld,
};

struct TriMeshAD
{
    TriMeshAD() = default;
    TriMeshAD(const vtrz::DMatrix4x4 &transAD, TransformADType type, TriMesh &mesh);
    TriMeshAD(const fs::path &vertexJacobianPath, TriMesh &mesh);
    TriMeshAD(const std::vector<float> &vertexJacobian, TriMesh &mesh);

    void computeSmoothNormals();
    void computeFaceNormals();

    std::vector<vtrz::DVector3> positions;
    std::vector<vtrz::DVector3> normals;
    TriMesh *mesh = nullptr;
    TriMeshADType type;
};

struct SimpleMeshDisplacement
{
    static TriMeshAD constant(float t, TriMesh &mesh, std::vector<float> &coefficients);
    static TriMeshAD sine(float t, float fphi, float ftheta, TriMesh &mesh, std::vector<float> &coefficients);
    static TriMeshAD fromData(const fs::path &path, TriMesh &mesh, std::vector<float> &coefficients);
};

struct PointLightAD
{
    const Emitter *light = nullptr;
    vtrz::DVector3 intensity;
    vtrz::DVector3 position;
};

struct PointShadowAD
{
    vtrz::DMatrix4x4 transforms[6];
    std::vector<uint16_t> meshIds[6];
    std::vector<uint32_t> primIds[6];
    std::vector<vtrz::DVector3> worldPositions[6];
    std::vector<vtrz::dfloat> wInvs[6];
    vtrz::VectorizerGrad vects[6];
};

struct SceneAD
{
    std::unordered_map<uint32_t, uint32_t> indexMap;
    std::vector<TriMeshAD> meshADs;

    std::vector<const Emitter *> areaLights;
    std::vector<PointLightAD> pointLights;
    std::vector<PointShadowAD> pointShadows;
};

MTS_NAMESPACE_END