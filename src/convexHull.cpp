//#include "convexHull.h"
//#include "polygonShared.h"
//
//namespace vtrz
//{
//
//ConvexHull::ConvexHull(BlockAllocator *allocator) :
//    refs(allocator), stack(allocator) {}
//
//uint32_t ConvexHull::buildInternal(DynamicArray<Vector2I64> &poly)
//{
//    ASSERT(poly.size() >= 3);
//
//    refs.clear();
//    stack.clear();
//
//    // Find lowest and swap.
//    uint32_t lowIndex = 0;
//    for (uint32_t i = 1; i < poly.size(); ++i) {
//        if ((poly[i].y < poly[lowIndex].y) ||
//            ((poly[i].y == poly[lowIndex].y) && (poly[i].x > poly[lowIndex].x))) {
//            lowIndex = i;
//        }
//    }
//    std::swap(poly[0], poly[lowIndex]);
//
//    refs.resize(poly.size());
//    for (uint32_t i = 0; i < poly.size(); ++i) {
//        refs[i].index = i;
//        refs[i].toDelete = false;
//    }
//
//    // Sort.
//    Vector2I64 low = poly[0];
//    bool needDelete = false;
//    std::sort(refs.begin() + 1, refs.end(), [&](PointRef &pi, PointRef &pj) {
//        int64_t area = areaSign(low, poly[pi.index], poly[pj.index]);
//        if (area > 0) return true;
//        else if (area < 0) return false;
//
//        // area == 0
//        int64_t x = std::abs(poly[pi.index].x - low.x) - std::abs(poly[pj.index].x - low.x);
//        int64_t y = std::abs(poly[pi.index].y - low.y) - std::abs(poly[pj.index].y - low.y);
//
//        needDelete = true;
//        if ((x < 0) || (y < 0)) {
//            pi.toDelete = true;
//            return true;
//        } else if ((x > 0) || (y > 0)) {
//            pj.toDelete = true;
//            return false;
//        } else { /* points are coincident */
//            if (pi.index > pj.index)
//                pj.toDelete = true;
//            else
//                pi.toDelete = true;
//            return false; // tie
//        }
//    });
//
//    // Remove duplicates.
//    if (needDelete) {
//        uint32_t newSize = 0;
//        for (uint32_t i = 0; i < poly.size(); ) {
//            if (!refs[i].toDelete) {
//                refs[newSize++] = refs[i];
//            }
//            ++i;
//        }
//        refs.resize(newSize);
//    }
//
//    // Actually build convex hull.
//    stack.push_back(refs[0].index);
//    stack.push_back(refs[1].index);
//    uint32_t next = 2;
//    while (next < refs.size()) {
//        ASSERT(stack.size() >= 2);
//        Vector2I64 p1 = poly[stack[stack.size() - 2]];
//        Vector2I64 p2 = poly[stack[stack.size() - 1]];
//        // Is it a left turn?
//        if (cross(p2 - p1, poly[refs[next].index] - p2) > 0) {
//            stack.push_back(refs[next++].index);
//        } else {
//            stack.pop_back();
//        }
//    }
//
//    // Move the result.
//    // poly = std::move(stack);
//
//    return lowIndex;
//}
//
//void ConvexHull::build(DynamicArray<Vector2I64> &poly)
//{
//    buildInternal(poly);
//
//    DynamicArray<Vector2I64> buf = poly;
//    poly.clear();
//    for (auto v : stack) {
//        poly.push_back(buf[v]);
//    }
//}
//
//void ConvexHull::reset()
//{
//    refs = DynamicArray<PointRef>(refs.allocator());
//    stack = DynamicArray<uint32_t>(stack.allocator());
//}
//
//}