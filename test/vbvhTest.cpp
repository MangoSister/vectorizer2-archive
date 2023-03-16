#include <Catch2/single_include/catch2/catch.hpp>
#include "util.h"
#include "maths.h"
#include "vbvh.h"
#include "vbvhGrad.h"
#include "modelLoader.h"
#include <fstream>

using namespace vtrz;

TEST_CASE("subd-0", "[subd]") {
    Vector3 verts[] = {
        Vector3(0.5f, 0.0f, 0.0f),
        Vector3(0.0f, 0.5f, 0.0f),
        Vector3(-0.4f, -0.3f, 0.0f),
    };
    uint32_t indices[] = { 0, 1, 2 };
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(verts, countOf(verts), indices, countOf(indices));
    vbvh.exportTo("C:/Users/yyp05/Desktop/subd-0.csv");
    SUCCEED();
}

TEST_CASE("subd-1", "[subd]") {
    Vector3 verts[] = {
        Vector3( 0.0, 0.8, 1.0 ),
        Vector3( -0.5, 0.8, 0.3 ),
        Vector3( 0.7, -0.8, 0.0 ),
        // tri 2
        Vector3( 0.3, 0.9, 0.0 ),
        Vector3( -0.7, -0.6, 0.8 ),
        Vector3( -0.4, -0.8, 0.0 ),
        // tri 3
        Vector3( 0.7, -0.1, 0.5 ),
        Vector3( -0.8, -0.2, 0.0 ),
        Vector3( 0.8, -0.5, 0.5 ),
    };
    uint32_t indices[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(verts, countOf(verts), indices, countOf(indices));
    vbvh.exportTo("C:/Users/MangoSister/Desktop/subd-1.csv");
    SUCCEED();
}


TEST_CASE("subd-2", "[subd]") {
    ImportFromObjOptions options;
    options.filepath = "C:/vectorizer2/test/assets/grid.obj";
    SimpleModel model = importFromObj(options);
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(model.vertices.data(), model.vertices.size(), model.indices.data(), model.indices.size());
    vbvh.exportTo("C:/Users/yyp05/Desktop/subd-2.csv");
    SUCCEED();
}

TEST_CASE("subd-3", "[subd]") {
    ImportFromObjOptions options;
    options.filepath = "C:/vectorizer2/test/assets/uvsphere.obj";
    SimpleModel model = importFromObj(options);
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(model.vertices.data(), model.vertices.size(), model.indices.data(), model.indices.size());
    vbvh.exportTo("C:/Users/yyp05/Desktop/subd-3.csv");
    SUCCEED();
}

TEST_CASE("subd-4", "[subd]") {
    ImportFromObjOptions options;
    options.filepath = "C:/vectorizer2/test/assets/spot_triangulated.obj";
    SimpleModel model = importFromObj(options);

    auto start = std::chrono::high_resolution_clock::now();
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(model.vertices.data(), model.vertices.size(), model.indices.data(), model.indices.size());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedSeconds = end - start;
    INFO("Process time: " << elapsedSeconds.count() << " seconds");

    vbvh.exportTo("C:/Users/MangoSister/Desktop/subd-4.csv");
    SUCCEED();
}

TEST_CASE("subd-5", "[subd]") {
    ImportFromObjOptions options;
    options.filepath = "C:/vectorizer2/test/assets/serapis.obj";
    SimpleModel model = importFromObj(options);

    auto start = std::chrono::high_resolution_clock::now();
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(model.vertices.data(), model.vertices.size(), model.indices.data(), model.indices.size());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedSeconds = end - start;
    INFO("Process time: " << elapsedSeconds.count() << " seconds");

    vbvh.exportTo("C:/Users/yyp05/Desktop/subd-5.csv");
    SUCCEED();
}

TEST_CASE("subd-6", "[subd]") {
    for (uint32_t i = 0; i < 8; ++i) {
        ImportFromObjOptions options;
        options.filepath = "C:/vectorizer2/test/assets/bunny.obj";
        options.angle = (float)i / 8.0 * Constants::twoPi;
        options.axis = Vector3(0.0f, 1.0f, 0.0f);
        SimpleModel model = importFromObj(options);

        auto start = std::chrono::high_resolution_clock::now();
        VBVH vbvh;
        vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
        vbvh.build(model.vertices.data(), model.vertices.size(), model.indices.data(), model.indices.size());

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - start;
        printf("(%u/8): rotate %.1f degrees\n", i, radToDeg(options.angle));
        printf("Process time: %f seconds.\n", elapsedSeconds.count());
        char outputPath[256];
        sprintf(outputPath, "C:/Users/MangoSister/Desktop/subd-6-%u.csv", i);
        vbvh.exportTo(outputPath);
    }
    SUCCEED();
}

TEST_CASE("subd-7", "[subd]") {
    for (uint32_t i = 0; i < 1; ++i) {
        ImportFromObjOptions options;
        options.filepath = "C:/vectorizer2/test/assets/bunny.obj";
        options.backfaceCulling = false;
        options.angle = (float)i / 8.0 * Constants::twoPi;
        options.axis = Vector3(0.0f, 1.0f, 0.0f);
        SimpleModel model = importFromObj(options);

        auto start = std::chrono::high_resolution_clock::now();
        VBVH vbvh;
        vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
        vbvh.build(model.vertices.data(), model.vertices.size(), model.indices.data(), model.indices.size());

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - start;
        printf("(%u/8): rotate %.1f degrees\n", i, radToDeg(options.angle));
        printf("Process time: %f seconds.\n", elapsedSeconds.count());
        char outputPath[256];
        sprintf(outputPath, "C:/Users/MangoSister/Desktop/subd-7-%u.csv", i);
        vbvh.exportTo(outputPath);
    }
    SUCCEED();
}

//////////////////////////////////////////////////////////////////////////
TEST_CASE("subd-grad-1", "[subd-grad]") {
    ImportFromObjOptions options;
    options.filepath = "C:/vectorizer2/test/assets/uvsphere.obj";
    SimpleModel model = importFromObj(options);
    std::vector<DVector3> vertexGradBuffer(model.vertices.size());
    for (uint32_t i = 0; i < (uint32_t)vertexGradBuffer.size(); ++i) {
        vertexGradBuffer[i].x.value = model.vertices[i].x;
        vertexGradBuffer[i].x.grad[0] = 1.0f;
        vertexGradBuffer[i].y.value = model.vertices[i].y;
        vertexGradBuffer[i].z.value = model.vertices[i].z;
    }

    VBVHGrad vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(vertexGradBuffer.data(), model.vertices.size(), model.indices.data(), model.indices.size());

    dfloat coverage = vbvh.coverageQuery();

    SUCCEED();
}

TEST_CASE("subd-occ-1", "[subd-occ]") {
    std::vector<ImportFromObjOptions> options;
    ImportFromObjOptions o1;
    o1.filepath = "C:/vectorizer2/test/assets/uvsphere.obj";
    o1.scale = 0.2f;
    o1.occludee = false;
    options.push_back(std::move(o1));
    ImportFromObjOptions o2;
    o2.filepath = "C:/vectorizer2/test/assets/quad.obj";
    o2.axis = Vector3(1.0f, 0.0f, 0.0f);
    o2.angle = Constants::halfPi;
    o2.translation = Vector3(0.0f, 0.0f, -2.0f);
    o2.occludee = true;
    options.push_back(std::move(o2));
    SimpleModel model = importFromObj(options);

    auto start = std::chrono::high_resolution_clock::now();
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.buildOcclusion(
        model.vertices.data(), (uint32_t)model.vertices.size(),
        model.indices.data(), (uint32_t)model.indices.size(),
        model.occludeeList.data(), (uint32_t)model.occludeeList.size());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedSeconds = end - start;
    INFO("Process time: " << elapsedSeconds.count() << " seconds");

    vbvh.exportTo("C:/Users/MangoSister/Desktop/subd-occ-1.csv");
}

TEST_CASE("complexity-1", "[complexity]") {

    ImportFromObjOptions options;
    options.filepath = "C:/vectorizer2/test/assets/stanford-dragon/stanford-dragon-5k.obj";
    //options.angle = (float)i / 8.0 * Constants::twoPi;
    //options.axis = Vector3(0.0f, 1.0f, 0.0f);
    SimpleModel model = importFromObj(options);

    auto start = std::chrono::high_resolution_clock::now();
    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)));
    vbvh.build(model.vertices.data(), model.vertices.size(), model.indices.data(), model.indices.size());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedSeconds = end - start;
    printf("Process time: %f seconds.\n", elapsedSeconds.count());

    //size_t memInBytes = 0;
    //memInBytes += vbvh.nodePool.data.capacity() * sizeof(VBVH::NodeType);
    //memInBytes += vbvh.nodePool.freeList.capacity() * sizeof(uint32_t);
    //auto allocatedList = vbvh.nodePool.allocatedList();
    //for (uint32_t i = 0; i < (uint32_t)allocatedList.size(); ++i) {
    //    NodeIndex index = allocatedList[i];
    //    if (vbvh.nodePool[index].isLeaf()) {
    //        memInBytes += vbvh.nodePool[index].leaf.vertCount * sizeof(vbvh.nodePool[index].leaf.geom);
    //    }
    //}
    //memInBytes += vbvh.leafRefs.capacity() * sizeof(uint32_t);
    //printf("VBVH memory consumption: %f MB.\n", (float)memInBytes / (float)(1024 * 1024));

    char outputPath[256];
    sprintf(outputPath, "C:/Users/MangoSister/Desktop/stanford-dragon.csv");
    vbvh.exportTo(outputPath);

    SUCCEED();
}

TEST_CASE("vhiz-occ-1", "[vhiz]") {
    std::vector<ImportFromObjOptions> options;
    ImportFromObjOptions o1;
    o1.filepath = "C:/vectorizer2/test/assets/uvsphere.obj";
    o1.scale = 0.2f;
    o1.translation = Vector3(0.0f, 0.0f, -2.0f);
    o1.occludee = true;
    options.push_back(std::move(o1));
    ImportFromObjOptions o2;
    o2.filepath = "C:/vectorizer2/test/assets/quad.obj";
    o2.axis = Vector3(1.0f, 0.0f, 0.0f);
    o2.angle = Constants::halfPi;
    o2.occludee = false;
    options.push_back(std::move(o2));
    SimpleModel model = importFromObj(options);

    VBVH vbvh;
    vbvh.reset(AABB2(Vector2(-1.0f), Vector2(1.0f)), 16);

    std::vector<Vector3> occludee;
    occludee.reserve(model.occludeeList.size() * 3);
    for (uint32_t o : model.occludeeList) {
        occludee.push_back(model.vertices[model.indices[3 * o + 0]]);
        occludee.push_back(model.vertices[model.indices[3 * o + 1]]);
        occludee.push_back(model.vertices[model.indices[3 * o + 2]]);
    }
    vbvh.addOccludees(occludee.data(), (uint32_t)occludee.size());
    vbvh.finishOccludees();

    for (uint32_t o : model.occluderList) {
        Tri3 tri = {
        model.vertices[model.indices[3 * o + 0]],
        model.vertices[model.indices[3 * o + 1]],
        model.vertices[model.indices[3 * o + 2]] };
        if (!vbvh.vhiz.query(tri.data(), 3)) {
            vbvh.addOccluders(tri.data(), 3);
        }
    }
    vbvh.finishOcclusion();

    vbvh.exportTo("C:/Users/MangoSister/Desktop/vhiz-occ-1.csv");

}