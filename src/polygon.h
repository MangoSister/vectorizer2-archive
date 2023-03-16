#pragma once

#include "blockAllocator.h"
#include "dynamicArray.h"
#include "maths.h"
#include "polygonShared.h"
#include "convexHull.h"
#include <vector>

namespace vtrz
{

struct IsectPoint
{
    IsectPoint(const Vector2I64 &exp) : implicit(false), exp(exp) {}
    IsectPoint(const ImplicitPoint &imp) : implicit(true), imp(imp) {}

    bool implicit; // false: "direct/explicit" | true: "parametric/implicit"
    union {
        Vector2I64 exp;
        ImplicitPoint imp;
    };
};

inline bool operator==(const IsectPoint &p1, const IsectPoint &p2) {
    if (p1.implicit != p2.implicit) return false;
    if (!p1.implicit) {
        return p1.exp == p2.exp;
    } else {
        return p1.imp == p2.imp;
    }
}

inline bool operator!=(const IsectPoint &p1, const IsectPoint &p2) {
    return !(p1 == p2);
}

template <typename TPoint>
struct SpatialGrid;

struct ConvexIntersector
{
    ConvexIntersector(const SpatialGrid<Vector2I64> *snapGrid = nullptr);

    const SpatialGrid<Vector2I64> *snapGrid = nullptr;

    BlockAllocator allocator;

    int sizeP, sizeQ;
    const Vector2I64 *P = nullptr, *Q = nullptr;

    DynamicArray<Vector2I64> isect;
    DynamicArray<DynamicArray<IsectPoint>> diffChainsP;
    DynamicArray<DynamicArray<IsectPoint>> diffChainsQ;
    DynamicArray<DynamicArray<Vector2I64>> splitPolys;

    ConvexHull<Vector2I64> convexHull;

    void reset(int newSizeP, int newSizeQ, const Vector2I64 *newP, const Vector2I64 *newQ);
    bool intersect();
    void splitToConvex();
    bool postProcess(DynamicArray<Vector2I64> &poly);

private:
    bool intersectSpecial();
    void cleanDiffChains();
    void completeWithHole();
    void splitToConvex1(int diffChainIndex);
    void splitToConvex2(int diffChainIndex);
    void splitToConvex3();
};

}