#include <Catch2/single_include/catch2/catch.hpp>
#include "convexHull.h"

using namespace vtrz;

TEST_CASE("convHull-1", "[convHull]") {

    BlockAllocator allocator;
    ConvexHull<Vector2I64> ctx(&allocator);
    DynamicArray<Vector2I64> points(&allocator);
    points.push_back(Vector2I64(0, 0));
    points.push_back(Vector2I64(1, 0));
    points.push_back(Vector2I64(2, 0));
    points.push_back(Vector2I64(2, 1));
    points.push_back(Vector2I64(2, 2));
    points.push_back(Vector2I64(1, 1));
    points.push_back(Vector2I64(1, 2));
    points.push_back(Vector2I64(0, 2));
    points.push_back(Vector2I64(0, 1));

    ctx.build(points);

    DynamicArray<Vector2I64> ans(&allocator);
    ans.push_back(Vector2I64(2, 0));
    ans.push_back(Vector2I64(2, 2));
    ans.push_back(Vector2I64(0, 2));
    ans.push_back(Vector2I64(0, 0));

    REQUIRE(points == ans);
    SUCCEED();
}

TEST_CASE("convHull-2", "[convHull]") {
    BlockAllocator allocator;
    ConvexHull<Vector2I64> ctx(&allocator);
    DynamicArray<Vector2I64> points(&allocator);

    points.push_back(Vector2I64(707006, -1048576));
    points.push_back(Vector2I64(755239, -755239));
    points.push_back(Vector2I64(485325, -485325));
    points.push_back(Vector2I64(275785, -1048576));
    points.push_back(Vector2I64(755239, -755239));
    points.push_back(Vector2I64(1048576, 1028701));
    points.push_back(Vector2I64(485325, -485325));
    points.push_back(Vector2I64(707006, -1048576));
    points.push_back(Vector2I64(1048576, -1048576));
    points.push_back(Vector2I64(755239, -755239));
    points.push_back(Vector2I64(1048576, -1048576));
    points.push_back(Vector2I64(1048576, 1028701));
    points.push_back(Vector2I64(755239, -755239));
    points.push_back(Vector2I64(275785, -1048576));
    points.push_back(Vector2I64(483874, -483874));
    points.push_back(Vector2I64(370517, -370517));
    points.push_back(Vector2I64(483874, -483874));
    points.push_back(Vector2I64(1048568, 1048554));
    points.push_back(Vector2I64(568776, 1048552));
    points.push_back(Vector2I64(370517, -370517));
    points.push_back(Vector2I64(275785, -1048576));
    points.push_back(Vector2I64(485325, -485325));
    points.push_back(Vector2I64(483874, -483874));
    points.push_back(Vector2I64(485325, -485325));
    points.push_back(Vector2I64(1048576, 1028701));
    points.push_back(Vector2I64(1048576, 1048555));
    points.push_back(Vector2I64(1048568, 1048554));
    points.push_back(Vector2I64(483874, -483874));
    points.push_back(Vector2I64(275785, -1048576));
    points.push_back(Vector2I64(370517, -370517));
    points.push_back(Vector2I64(-1048543, 1048543));
    points.push_back(Vector2I64(-1048556, 1048543));
    points.push_back(Vector2I64(-1048554, -1048576));
    points.push_back(Vector2I64(370517, -370517));
    points.push_back(Vector2I64(568776, 1048552));
    points.push_back(Vector2I64(-1048543, 1048543));



    ctx.build(points);

}