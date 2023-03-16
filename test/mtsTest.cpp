#include <Catch2/single_include/catch2/catch.hpp>
#include "util.h"
#include "maths.h"
#include "vbvh.h"
#include "vbvhGrad.h"
#include "modelLoader.h"
#include <fstream>

using namespace vtrz;

// Read dumped data from mitsuba test scenes
void readMitsubaInputDump(const char *path,
    AABB2 &bound,
    std::vector<Vector3> &vertexBuffer,
    std::vector<uint32_t> &indexBuffer,
    std::vector<uint32_t> &queryTriIndices,
    std::vector<DVector2> &vertexGradBuffer)
{
    std::ifstream file(path, std::ios::binary);

    file.read(reinterpret_cast<char *>(&bound), sizeof(AABB2));

    uint32_t vertexCount;
    file.read(reinterpret_cast<char *>(&vertexCount), sizeof(uint32_t));

    vertexBuffer.clear();
    vertexBuffer.resize(vertexCount);
    for (uint32_t i = 0; i < vertexCount; ++i) {
        file.read(reinterpret_cast<char *>(&vertexBuffer[i]), sizeof(Vector3));
    }

    uint32_t indexCount;
    file.read(reinterpret_cast<char *>(&indexCount), sizeof(uint32_t));
    indexBuffer.clear();
    if (indexCount > 0) {
        indexBuffer.resize(indexCount);
        for (uint32_t i = 0; i < indexCount; ++i) {
            file.read(reinterpret_cast<char *>(&indexBuffer[i]), sizeof(uint32_t));
        }
    }

    uint32_t querySize;
    file.read(reinterpret_cast<char *>(&querySize), sizeof(uint32_t));
    queryTriIndices.clear();
    if (querySize > 0) {
        queryTriIndices.resize(querySize);
        for (uint32_t i = 0; i < querySize; ++i) {
            file.read(reinterpret_cast<char *>(&queryTriIndices[i]), sizeof(uint32_t));
        }
    }

    uint32_t vertexGradCount;
    file.read(reinterpret_cast<char *>(&vertexGradCount), sizeof(uint32_t));
    vertexGradBuffer.clear();
    if (vertexGradCount) {
        vertexGradBuffer.resize(vertexGradCount);
        for (uint32_t i = 0; i < vertexCount; ++i) {
            file.read(reinterpret_cast<char *>(&vertexGradBuffer[i]), sizeof(DVector2));
        }
    }
}

TEST_CASE("mitsuba-dump-1", "[mitsuba-dump]") {
	AABB2 bound;
	std::vector<Vector3> vertexBuffer;
	std::vector<uint32_t> indexBuffer;
    std::vector<uint32_t> queryTriIndices;
    std::vector<DVector2> vertexGradBuffer;

    const char *path = "C:/Users/MangoSister/Desktop/scene-movelight-17-78.data";
	readMitsubaInputDump(path, bound, vertexBuffer, indexBuffer, queryTriIndices, vertexGradBuffer);

    //VBVH vbvh;
    //vbvh.reset(bound, vtrz::DepthFunc::eGreater);
    ////vbvh.build(
    ////    vertexBuffer.data(), (uint32_t)vertexBuffer.size(),
    ////    indexBuffer.data(), (uint32_t)indexBuffer.size());

    //vbvh.buildOcclusion(
    //    vertexBuffer.data(), (uint32_t)vertexBuffer.size(),
    //    indexBuffer.data(), (uint32_t)indexBuffer.size(),
    //    queryTriIndices.data(), (uint32_t)queryTriIndices.size());

    //float coverage = vbvh.coverageQuery();

    //vbvh.exportTo("C:/Users/MangoSister/Desktop/sphere2-331-315.csv");

    //VBVHGrad vbvhGrad;
    //vbvhGrad.reset(bound, vtrz::DepthFunc::eGreater);
    ////vbvhGrad.build(
    ////    vertexBuffer.data(), vertexGradBuffer.data(), (uint32_t)vertexBuffer.size(),
    ////    nullptr, 0);
    //vbvhGrad.buildOcclusion(
    //    vertexBuffer.data(), vertexGradBuffer.data(), (uint32_t)vertexBuffer.size(),
    //    nullptr, 0,
    //    queryTriIndices.data(), (uint32_t)queryTriIndices.size());

    //dfloat coverage = vbvhGrad.coverageQuery();
    //vbvhGrad.pointSample();

    //vbvhGrad.exportTo("C:/Users/MangoSister/Desktop/scene-movelight-17-78.csv");

    SUCCEED();
}