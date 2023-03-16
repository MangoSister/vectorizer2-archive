#include <Catch2/single_include/catch2/catch.hpp>
#include "util.h"
#include "maths.h"

using namespace vtrz;

TEST_CASE("VLA", "[util]") {
    uint32_t size = (uint32_t)std::sqrt(100.0f);
    VLA(buf, Vector2, size);
    PreAllocVector<Vector2> arr(buf, size);
    for (uint32_t i = 0; i < size; ++i) {
        arr.push_back(Vector2(i + 1.0f));
    }
    for (uint32_t i = 0; i < size; ++i) {
        REQUIRE(arr[i].x == i + 1);
        REQUIRE(arr[i].y == i + 1);
    }
}