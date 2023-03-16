#pragma once

#include "maths.h"
#include <vector>

namespace vtrz
{

template <typename TPoint>
struct NNSearchResult
{
    bool found;
    int64_t dist2;
    TPoint neighbor;
};

template <typename TPoint>
struct SpatialGrid
{
    SpatialGrid(const std::vector<TPoint> &points, const Vector2I64 bound[2], int64_t queryRadius);

    NNSearchResult<TPoint> nnSearch(const Vector2I64 &pos) const;
    void nnSearch(const Vector2I64 &pos, int ix, int iy, NNSearchResult<TPoint> &result) const;

    const std::vector<TPoint> &points;
    Vector2I64 bound[2];
    double invBoundExts[2];

    int64_t queryRadius;
    int64_t queryRadius2;

    uint32_t res[2];
    std::vector<uint32_t> voxels;
    std::vector<uint32_t> sortedIndices;
};

inline uint32_t mortonCode2(uint32_t x, uint32_t y) {
    constexpr uint32_t B[] = { 0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF };
    constexpr uint32_t S[] = { 1, 2, 4, 8 };

    x = (x | (x << S[3])) & B[3];
    x = (x | (x << S[2])) & B[2];
    x = (x | (x << S[1])) & B[1];
    x = (x | (x << S[0])) & B[0];

    y = (y | (y << S[3])) & B[3];
    y = (y | (y << S[2])) & B[2];
    y = (y | (y << S[1])) & B[1];
    y = (y | (y << S[0])) & B[0];

    return  (y << 1) | x;
}

template <typename TPoint>
SpatialGrid<TPoint>::SpatialGrid(const std::vector<TPoint> &points, const Vector2I64 bound[2], int64_t queryRadius) :
    points(points), bound{ bound[0], bound[1] }, queryRadius(queryRadius), queryRadius2(queryRadius *queryRadius)
{

    Vector2I64 exts = bound[1] - bound[0];
    invBoundExts[0] = 1.0 / (double)exts.x;
    invBoundExts[1] = 1.0 / (double)exts.y;
    res[0] = (uint32_t)std::ceil((double)exts.x / (double)queryRadius);
    res[0] = std::min(nextPowerOf2U32(res[0]), 512u);
    res[1] = (uint32_t)std::ceil((double)exts.y / (double)queryRadius);
    res[1] = std::min(nextPowerOf2U32(res[1]), 512u);

    struct SortInfo
    {
        uint32_t morton;
        uint32_t index;
    };
    std::vector<SortInfo> infos(points.size());
    for (uint32_t i = 0; i < (uint32_t)points.size(); ++i) {
        const Vector2I64 &pos = getVal(points[i]);
        uint32_t ix = clamp((uint32_t)((pos.x - bound[0].x) * invBoundExts[0] * res[0]), 0u, res[0] - 1u);
        uint32_t iy = clamp((uint32_t)((pos.y - bound[0].y) * invBoundExts[1] * res[1]), 0u, res[1] - 1u);
        infos[i].morton = mortonCode2(ix, iy);
        infos[i].index = i;
    }
    // TODO: radix sort?
    std::sort(infos.begin(), infos.end(), [](const SortInfo &x, const SortInfo &y) { return x.morton < y.morton; });

    sortedIndices.resize(infos.size());
    voxels.resize(res[0] * res[1]);
    for (uint32_t i = 0; i < (uint32_t)infos.size(); ++i) {
        sortedIndices[i] = infos[i].index;
        ++voxels[infos[i].morton];
    }
    uint32_t prefixSum = 0;
    for (uint32_t i = 0; i < (uint32_t)voxels.size(); ++i) {
        uint32_t count = voxels[i];
        voxels[i] = prefixSum;
        prefixSum += count;
    }
}

template <typename TPoint>
NNSearchResult<TPoint> SpatialGrid<TPoint>::nnSearch(const Vector2I64 &pos) const
{
    double fx = (pos.x - bound[0].x) * invBoundExts[0] * res[0];
    double fy = (pos.y - bound[0].y) * invBoundExts[1] * res[1];
    uint32_t ix = clamp((uint32_t)fx, 0u, res[0] - 1u);
    uint32_t iy = clamp((uint32_t)fy, 0u, res[1] - 1u);
    int dx = (fx - (double)ix) >= 0.5f ? 1 : -1;
    int dy = (fy - (double)iy) >= 0.5f ? 1 : -1;

    NNSearchResult<TPoint> result;
    result.found = false;
    result.dist2 = queryRadius2;

    nnSearch(pos, ix, iy, result);
    nnSearch(pos, ix, iy + dy, result);
    nnSearch(pos, ix + dx, iy, result);
    nnSearch(pos, ix + dx, iy + dy, result);

    return result;
}

template <typename TPoint>
void SpatialGrid<TPoint>::nnSearch(const Vector2I64 &pos, int ix, int iy, NNSearchResult<TPoint> &result) const
{
    if (ix < 0 || ix >= (int)res[0]) return;
    if (iy < 0 || iy >= (int)res[1]) return;
    uint32_t morton = mortonCode2(ix, iy);
    uint32_t start = voxels[morton];
    uint32_t count = ((morton == (uint32_t)voxels.size() - 1) ? (uint32_t)points.size() : voxels[morton + 1]) - voxels[morton];
    uint32_t end = start + count;
    for (uint32_t i = start; i < end; ++i) {
        const Vector2I64 &pt = getVal(points[sortedIndices[i]]);
        int64_t newDist2 = (pt.x - pos.x) * (pt.x - pos.x) + (pt.y - pos.y) * (pt.y - pos.y);
        if (newDist2 <= result.dist2) {
            result.found = true;
            result.dist2 = newDist2;
            result.neighbor = points[sortedIndices[i]];
        }
    }
}

}