#include "modelLoader.h"
#define TINYOBJLOADER_IMPLEMENTATION // Define this in only *one* .cc.
#include <tiny_obj_loader.h>
#include <algorithm>
#include <cmath>

namespace vtrz
{

SimpleModel importFromObj(const char *filepath)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filepath, nullptr);

    if (!warn.empty()) {
        fprintf(stdout, "Warning: %s.\n", warn.c_str());
    }

    if (!err.empty()) {
        fprintf(stderr, "Error: %s.\n", err.c_str());
    }

    if (!ret) {
        fprintf(stderr, "Failed to load obj: %s. Exiting.\n", filepath);
        exit(1);
    }

    assert(attrib.vertices.size() % 3 == 0);

    SimpleModel model;
    model.vertices.resize(attrib.vertices.size() / 3);
    for (size_t v = 0; v < model.vertices.size(); ++v) {
        for (uint32_t i = 0; i < 3; ++i) {
            model.vertices[v][i] = attrib.vertices[v * 3 + i];
        }
    }

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];
            ASSERT(fv == 3);
            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                model.indices.push_back(idx.vertex_index);
            }

            index_offset += fv;
        }
    }

    return model;
}

SimpleModel importFromObj(const ImportFromObjOptions &options)
{
    SimpleModel model = importFromObj(options.filepath.c_str());

    AABB3 localBound;
    for (size_t v = 0; v < model.vertices.size(); ++v) {
        localBound.expand(model.vertices[v]);
    }

    // Simple automatic camera placement.
    constexpr float near = 0.01f;
    constexpr float far = 1000.0f;
    constexpr float vFov = degToRad(60.0f);
    constexpr float aspect = 1.0f;

    float radius = std::max(0.5f * length(localBound.extents()), near);
    Vector3 center = localBound.center();
    Vector3 eye = center + Vector3(0.0f, 0.0f, std::max(radius / std::sin(0.5f * vFov), near));
    Matrix4x4 modelMatrix = makeRotate(options.angle, normalize(options.axis));
    Matrix4x4 viewMatrix = makeLookupView(eye, center - eye, Vector3(0.0f, 1.0f, 0.0f));
    Matrix4x4 projMatrix = makePerspective(near, far, vFov, aspect); // near: -1.0; far: 1.0f
    Matrix4x4 mvpMatrix = projMatrix * viewMatrix * modelMatrix;

    for (auto &v : model.vertices) {
        // Vector4 vView = viewMatrix * Vector4(v, 1.0f);
        // vec4f vProj = proj * vView;
        Vector4 vProj = mvpMatrix * Vector4(v, 1.0f);
        vProj /= vProj.w;
        ASSERT(vProj.x >= -1.0f && vProj.x <= 1.0f);
        ASSERT(vProj.y >= -1.0f && vProj.y <= 1.0f);
        ASSERT(vProj.z >= -1.0f && vProj.z <= 1.0f);
        v = Vector3(vProj);
    }

    std::vector<uint32_t> newIndices;
    for (size_t i = 0; i < model.indices.size(); i += 3) {
        Vector2 v0 = (Vector2)model.vertices[model.indices[i + 0]];
        Vector2 v1 = (Vector2)model.vertices[model.indices[i + 1]];
        Vector2 v2 = (Vector2)model.vertices[model.indices[i + 2]];

        // Backface culling.
        float crossProduct = cross(v1 - v0, v2 - v1);
        if (crossProduct == 0.0f) {
            continue;
        } else if (crossProduct < 0.0f) {
            if (options.backfaceCulling) {
                continue;
            } else {
                std::swap(v1, v2);
                std::swap(model.indices[i + 1], model.indices[i + 2]);
                ASSERT(cross(v1 - v0, v2 - v1) > 0.0f);
            }
        }

        // Index buffer after culling.
        newIndices.push_back(model.indices[i + 0]);
        newIndices.push_back(model.indices[i + 1]);
        newIndices.push_back(model.indices[i + 2]);
    }

    model.indices = std::move(newIndices);
    return model;
}

SimpleModel importFromObj(const std::vector<ImportFromObjOptions> &options)
{
    SimpleModel combinedModel;
    AABB3 worldBound;
    std::vector<bool> occludeeFlags;

    for (const auto &option : options) {
        SimpleModel model = importFromObj(option.filepath.c_str());
        Matrix4x4 modelMatrix =
            makeTranslate(option.translation.x, option.translation.y, option.translation.z) *
            makeRotate(option.angle, normalize(option.axis)) *
            makeScale(option.scale, option.scale, option.scale);
        for (Vector3 &v : model.vertices) {
            v = (Vector3)(modelMatrix * Vector4(v.x, v.y, v.z, 1.0f));
            worldBound.expand(v);
        }
        uint32_t offset = (uint32_t)combinedModel.vertices.size();
        combinedModel.vertices.insert(combinedModel.vertices.end(),
            std::make_move_iterator(model.vertices.begin()),
            std::make_move_iterator(model.vertices.end()));
        combinedModel.indices.reserve(combinedModel.indices.size() + model.indices.size());
        occludeeFlags.reserve(occludeeFlags.size() + model.indices.size() / 3);
        for (uint32_t i = 0; i < (uint32_t)model.indices.size(); ++i) {
            combinedModel.indices.push_back(model.indices[i] + offset);
            if (i % 3 == 0) {
                occludeeFlags.push_back(option.occludee);
            }
        }
    }

    constexpr float near = 0.01f;
    constexpr float far = 1000.0f;
    constexpr float vFov = degToRad(60.0f);
    constexpr float aspect = 1.0f;

    float radius = std::max(0.5f * length(worldBound.extents()), near);
    Vector3 center = worldBound.center();
    Vector3 eye = center + Vector3(0.0f, 0.0f, std::max(radius / std::sin(0.5f * vFov), near));
    Matrix4x4 viewMatrix = makeLookupView(eye, center - eye, Vector3(0.0f, 1.0f, 0.0f));
    Matrix4x4 projMatrix = makePerspective(near, far, vFov, aspect); // near: -1.0; far: 1.0f
    Matrix4x4 vpMatrix = projMatrix * viewMatrix;

    for (auto &v : combinedModel.vertices) {
        // Vector4 vView = viewMatrix * Vector4(v, 1.0f);
        // vec4f vProj = proj * vView;
        Vector4 vProj = vpMatrix * Vector4(v, 1.0f);
        vProj /= vProj.w;
        ASSERT(vProj.x >= -1.0f && vProj.x <= 1.0f);
        ASSERT(vProj.y >= -1.0f && vProj.y <= 1.0f);
        ASSERT(vProj.z >= -1.0f && vProj.z <= 1.0f);
        v = Vector3(vProj);
    }

    bool backfaceCulling = options[0].backfaceCulling; // TODO.
    std::vector<uint32_t> newIndices;
    std::vector<uint8_t> newOcclusion;
    for (size_t i = 0; i < combinedModel.indices.size(); i += 3) {
        Vector2 v0 = (Vector2)combinedModel.vertices[combinedModel.indices[i + 0]];
        Vector2 v1 = (Vector2)combinedModel.vertices[combinedModel.indices[i + 1]];
        Vector2 v2 = (Vector2)combinedModel.vertices[combinedModel.indices[i + 2]];

        // Backface culling.
        float crossProduct = cross(v1 - v0, v2 - v1);
        if (crossProduct == 0.0f) {
            continue;
        } else if (crossProduct < 0.0f) {
            if (backfaceCulling) {
                continue;
            } else {
                std::swap(v1, v2);
                std::swap(combinedModel.indices[i + 1], combinedModel.indices[i + 2]);
                ASSERT(cross(v1 - v0, v2 - v1) > 0.0f);
            }
        }

        // Index buffer after culling.
        if (occludeeFlags[i / 3]) {
            combinedModel.occludeeList.push_back((uint32_t)newIndices.size() / 3);
        } else {
            combinedModel.occluderList.push_back((uint32_t)newIndices.size() / 3);
        }
        newIndices.push_back(combinedModel.indices[i + 0]);
        newIndices.push_back(combinedModel.indices[i + 1]);
        newIndices.push_back(combinedModel.indices[i + 2]);
    }

    combinedModel.indices = std::move(newIndices);
    return combinedModel;
}

}