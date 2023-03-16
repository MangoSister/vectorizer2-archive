#pragma once

#include "blockAllocator.h"
#include "dynamicArray.h"
#include "polygonShared.h"

namespace vtrz
{

template <typename TPoint>
struct ConvexHull
{
    explicit ConvexHull(BlockAllocator *allocator);
    void reset();
    void build(DynamicArray<TPoint> &poly);

private:
    DynamicArray<uint32_t> refs;
    DynamicArray<TPoint> stack;
};

template <typename TPoint>
ConvexHull<TPoint>::ConvexHull(BlockAllocator *allocator) :
    refs(allocator), stack(allocator) {}

template <typename TPoint>
void ConvexHull<TPoint>::reset()
{
    refs = DynamicArray<uint32_t>(refs.allocator());
    stack = DynamicArray<TPoint>(stack.allocator());
}

template <typename TPoint>
void ConvexHull<TPoint>::build(DynamicArray<TPoint> &poly)
{
    ASSERT(poly.size() >= 3);

    refs.clear();
    stack.clear();

    // Find lowest and swap.
    int lowIndex = 0;
    Vector2I64 low = getVal(poly[0]);
    for (int i = 1; i < (int)poly.size(); ++i) {
        Vector2I64 pos = getVal(poly[i]);
        if ((pos.y < low.y) ||
            ((pos.y == low.y) && (pos.x > low.x))) {
            lowIndex = i;
            low = getVal(poly[lowIndex]);
        }
    }
    std::swap(poly[0], poly[lowIndex]);

    refs.resize(poly.size());
    for (int i = 0; i < (int)poly.size(); ++i) {
        refs[i] = i;
        // refs[i].toDelete = false;
    }

    // Sort.
    //bool needDelete = false;
    //std::sort(refs.begin() + 1, refs.end(), [&](PointRef &pi, PointRef &pj) {
    //    Vector2I64 posi = getVal(poly[pi.index]);
    //    Vector2I64 posj = getVal(poly[pj.index]);

    //    int64_t area = areaSign(low, posi, posj);
    //    if (area > 0) return true;
    //    else if (area < 0) return false;

    //    // area == 0
    //    int64_t x = std::abs(posi.x - low.x) - std::abs(posj.x - low.x);
    //    int64_t y = std::abs(posi.y - low.y) - std::abs(posj.y - low.y);

    //    needDelete = true;
    //    if ((x < 0) || (y < 0)) {
    //        pi.toDelete = true;
    //        return true;
    //    } else if ((x > 0) || (y > 0)) {
    //        pj.toDelete = true;
    //        return false;
    //    } else { /* points are coincident */
    //        if (pi.index > pj.index)
    //            pj.toDelete = true;
    //        else
    //            pi.toDelete = true;
    //        return false; // tie
    //    }
    //});

    //// Remove duplicates.
    //if (needDelete) {
    //    int newSize = 0;
    //    for (int i = 0; i < (int)poly.size(); ) {
    //        if (!refs[i].toDelete) {
    //            refs[newSize++] = refs[i];
    //        }
    //        ++i;
    //    }
    //    refs.resize(newSize);
    //}

    bool needDelete = false;
    std::sort(refs.begin() + 1, refs.end(), [&](uint32_t &pi, uint32_t &pj) {
        Vector2I64 posi = getVal(poly[pi]);
        Vector2I64 posj = getVal(poly[pj]);

        int64_t area = areaSign(low, posi, posj);
        if (area > 0) return true;
        else if (area < 0) return false;
        // area == 0
        else {
            needDelete = true;
            // area == 0
            if (posi.y > posj.y) return true;
            else if (posi.y < posj.y) return false;
            return posi.x < posj.x;
        }

    });

    // Remove duplicates.
    if (needDelete) {
        int newSize = 2;
        int extreme = 1;
        for (int i = 2; i < (int)poly.size(); ++i) {
            if (areaSign(low, getVal(poly[refs[extreme]]), getVal(poly[refs[i]])) != 0) {
                extreme = i;
                refs[newSize++] = refs[i];
            }
        }
        refs.resize(newSize);
    }

    // Actually build convex hull.
    stack.push_back(poly[refs[0]]);
    stack.push_back(poly[refs[1]]);
    int next = 2;
    while (next < (int)refs.size()) {
        ASSERT(stack.size() >= 2);
        Vector2I64 p1 = getVal(stack[stack.size() - 2]);
        Vector2I64 p2 = getVal(stack[stack.size() - 1]);
        // Is it a left turn?
        if (cross(p2 - p1, getVal(poly[refs[next]]) - p2) > 0) {
            stack.push_back(poly[refs[next++]]);
        } else {
            stack.pop_back();
        }
    }

    // Move the result.
    poly = std::move(stack);
}

}