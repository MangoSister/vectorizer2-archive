#include <Catch2/single_include/catch2/catch.hpp>
#include "vbvh.h"
#include <fstream>

using namespace vtrz;

static void dumpVHizDebugInfo(const char *outputPath,
    const VHiZ &vhiz,
    const Vector3 *occludee, uint32_t occludeeVertCount,
    const Vector3 *occluder, uint32_t occluderVertCount)
{
    size_t bufSize =
        sizeof(int) * 2 + sizeof(float) * vhiz.width * vhiz.height +
        sizeof(uint32_t) + sizeof(Vector3) * occludeeVertCount +
        sizeof(uint32_t) + sizeof(Vector3) * occluderVertCount;

    char *buf = new char[bufSize];
    size_t offset = 0;
    memcpy(buf + offset, &vhiz.width, sizeof(int));
    offset += sizeof(int);
    memcpy(buf + offset, &vhiz.height, sizeof(int));
    offset += sizeof(int);
    memcpy(buf + offset, vhiz.framebuffer.data(), sizeof(float) * vhiz.width * vhiz.height);
    offset += sizeof(float) * vhiz.width * vhiz.height;
    memcpy(buf + offset, &occludeeVertCount, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(buf + offset, occludee, sizeof(Vector3) * occludeeVertCount);
    offset += sizeof(Vector3) * occludeeVertCount;
    memcpy(buf + offset, &occluderVertCount, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(buf + offset, occluder, sizeof(Vector3) * occluderVertCount);
    offset += sizeof(Vector3) * occluderVertCount;

    ASSERT(offset == bufSize);
    std::ofstream file(outputPath, std::ios::binary);
    file.write(buf, bufSize);
    file.close();

    delete[] buf;
}


TEST_CASE("vhiz-0", "[vhiz]") {
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)), 16, DepthFunc::eGreater);

    std::array<Vector3, 6> occludee = {
        Vector3(-1.0f, -1.0f, 0.5f),
        Vector3(1.0f, -1.0f, 0.5f),
        Vector3(1.0f, 1.0f, 0.5f),

        Vector3(-1.0f, -1.0f, 0.5f),
        Vector3(1.0f, 1.0f, 0.5f),
        Vector3(-1.0f, 1.0f, 0.5f),
    };

    std::array<Vector3, 9> occluder = {
        Vector3(-0.5f, -0.5f, 0.7f),
        Vector3(0.5f, -0.5f, 0.7f),
        Vector3(0.5f, 0.5f, 0.7f),

        Vector3(-0.5f, -0.5f, 0.7f),
        Vector3(0.5f, 0.5f, 0.7f),
        Vector3(-0.5f, 0.5f, 0.7f),

        Vector3(-0.2f, -0.2f, 0.6f),
        Vector3(0.2f, 0.2f, 0.6f),
        Vector3(-0.2f, 0.2f, 0.6f),
    };

    vbvh.addOccludees(occludee.data(), (uint32_t)occludee.size());
    vbvh.finishOccludees();

    bool q1 = vbvh.queryOccluders(&occluder[0], 3);
    REQUIRE(!q1);
    if (!q1) {
        vbvh.addOccluders(&occluder[0], 3);
    }

    bool q2 = vbvh.queryOccluders(&occluder[3], 3);
    REQUIRE(!q2);
    if (!q2) {
        vbvh.addOccluders(&occluder[3], 3);
    }

    bool q3 = vbvh.queryOccluders(&occluder[6], 3);
    REQUIRE(q3);
    if (!q3) {
        vbvh.addOccluders(&occluder[6], 3);
    }

    vbvh.finishOcclusion();

    //dumpVHizDebugInfo("C:/Users/MangoSister/Desktop/vhiz-0.bin",
    //vbvh.vhiz,
    //occludee.data(), (uint32_t)occludee.size(),
    //occluder.data(), (uint32_t)occluder.size());
}