#pragma once

#include "blockAllocator.h"
#include "dynamicArray.h"
#include "maths.h"
#include "polygonShared.h"
#include "convexHull.h"
#include "diff.h"
#include <vector>

namespace vtrz
{

struct ImplicitPointGrad
{
    // Record the input and output implicitly of segment intersection.
    // a: the index of point 1 of the seg 1.
    // b: the index of point 2 of the first seg 1.
    // s: the lerp parameter of the intersection, using the seg 1 eq.
    // c: the index of point 1 of the seg 2.
    // d: the index of point 2 of the first seg 2.
    // t: the lerp parameter of the intersection, using the seg 2 eq.

    int a, b;
    ddouble s;
    int c, d;
    // ddouble t; // t is never used.
};

inline bool operator==(const ImplicitPointGrad &p1, const ImplicitPointGrad &p2) {
    return
        (p1.a == p2.a && p1.b == p2.b && p1.s.value == p2.s.value) &&
        (p1.c == p2.c && p1.d == p2.d /*&& p1.t.value == p2.t.value*/);
}

struct IsectPointGrad
{
    IsectPointGrad(const DVector2I64 &exp) : implicit(false), exp(exp) {}
    IsectPointGrad(const ImplicitPointGrad &imp) : implicit(true), imp(imp) {}

    IsectPointGrad(const IsectPointGrad &other) : implicit(other.implicit) {
        if (!implicit) exp = other.exp;
        else imp = other.imp;
    }

    bool implicit; // false: "direct/explicit" | true: "parametric/implicit"
    union {
        DVector2I64 exp;
        ImplicitPointGrad imp;
    };
};

inline bool operator==(const IsectPointGrad &p1, const IsectPointGrad &p2) {
    if (p1.implicit != p2.implicit) return false;
    if (!p1.implicit) {
        return p1.exp.x == p2.exp.x && p1.exp.y == p2.exp.y;
    } else {
        return p1.imp == p2.imp;
    }
}

inline bool operator!=(const IsectPointGrad &p1, const IsectPointGrad &p2) {
    return !(p1 == p2);
}

template <typename TPoint>
struct SpatialGrid;

struct ConvexIntersectorGrad
{
    ConvexIntersectorGrad(const SpatialGrid<DVector2I64> *snapGrid = nullptr);

    const SpatialGrid<DVector2I64> *snapGrid = nullptr;

    BlockAllocator allocator;

    int sizeP, sizeQ;
    const DVector2I64 *P = nullptr, *Q = nullptr;
    DynamicArray<DVector2I64> isect;
    DynamicArray<DynamicArray<IsectPointGrad>> diffChainsP;
    DynamicArray<DynamicArray<IsectPointGrad>> diffChainsQ;
    DynamicArray<DynamicArray<DVector2I64>> splitPolys;

    ConvexHull<DVector2I64> convexHull;

    void reset(int newSizeP, int newSizeQ,
        const DVector2I64 *newP, const DVector2I64 *newQ);
    bool intersect();
    void splitToConvex();
    bool postProcess(DynamicArray<DVector2I64> &poly);

    IsectPointGrad createIsectPoint(int a1, int a, int b1, int b, const ddouble &s, const ddouble &t) const;

private:
    bool intersectSpecial();
    void cleanDiffChains();
    void completeWithHole();
    void splitToConvex1(int diffChainIndex);
    void splitToConvex2(int diffChainIndex);
    void splitToConvex3();
};

}