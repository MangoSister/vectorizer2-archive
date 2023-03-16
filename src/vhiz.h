#pragma once

#include "maths.h"
#include "util.h"
#include "vectorizer.h"
#include <vector>

namespace vtrz
{

struct VHiZ
{
    enum class TrianglePart
    {
        eTop,
        eBottom,
    };

    void reset(const AABB2 &newBound, uint32_t newWidth, uint32_t newHeight, DepthFunc depthFunc);

    void recordOccludee(const Vector3 *poly, uint32_t vertCount);
    void recordOccludeeInterior(Tri3 tri);
    void recordOccludeeInterior(TrianglePart part, const Tri3 &tri);
    void recordOccludeeBoundary(const Vector3 &start, const Vector3 &end);

    void recordOccluder(const Vector2 *poly, uint32_t vertCount, std::vector<Vector2I> &cellsToQuery);
    void recordOccluderInterior(Tri2 tri);
    void recordOccluderInterior(TrianglePart part, const Tri2 &tri);

    bool query(const AABB2 &box, float depth) const;
    bool query(const Vector3 *vertexBuffer, uint32_t vertexCount) const;
    bool query(Tri3 tri) const;
    bool queryInterior(TrianglePart part, const Tri3 &tri) const;
    bool queryBoundary(const Vector3 &start, const Vector3 &end) const;

    template <typename T>
    T convertToFrameCoord(T v) const {
        v.x = (v.x - bound.min.x) * ndcScale.x;
        v.y = (v.y - bound.min.y) * ndcScale.y;
        return v;
    }

    inline AABB2 getCell(const Vector2I &idx) const {
        ASSERT(idx.x < width && idx.y < height);
        return AABB2(
            Vector2((float)idx.x * getCellSx - 1.0f, (float)idx.y * getCellSy - 1.0f),
            Vector2((float)idx.x * getCellSx + getCellTx, (float)idx.y * getCellSy + getCellTy));
    }

    // Debug/Visualization
    void exportTo(const char *outputPath) const;

    AABB2 bound;
    Vector2 ndcScale;
    int width, height;
    float getCellSx, getCellSy;
    float getCellTx, getCellTy;

    DepthFunc depthFunc;

    std::vector<float> framebuffer;

    std::vector<Vector2I> occluderBuffer;
};

}