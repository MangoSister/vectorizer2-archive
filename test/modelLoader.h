#pragma once

#include "maths.h"
#include <vector>
#include <string>

namespace vtrz
{

struct SimpleModel
{
    std::vector<Vector3> vertices;
    std::vector<uint32_t> indices;
    std::vector<uint32_t> occludeeList;
    std::vector<uint32_t> occluderList;
};

struct ImportFromObjOptions
{
    std::string filepath;
    bool backfaceCulling = true;
    Vector3 axis = Vector3(0.0f, 1.0f, 0.0f);
    float angle = 0.0f;
    // Don't matter for single object.
    Vector3 translation = Vector3(0.0f);
    float scale = 1.0f;
    bool occludee = false;
};

SimpleModel importFromObj(const ImportFromObjOptions &options);
SimpleModel importFromObj(const std::vector<ImportFromObjOptions> &options);
}