#include "vbvhShared.h"
#include "util.h"
#include "polygonShared.h"
#include <fstream>

namespace vtrz
{

//static inline Vector2 projToAxis(const Vector3 tri[3], Vector3 axis, Vector3 offset)
//{
//    Vector2 proj(Constants::inf, -Constants::inf);
//    for (uint32_t i = 0; i < 3; ++i) {
//        float p = dot(tri[i] - offset, axis);
//        proj.x = std::min(proj.x, p);
//        proj.y = std::max(proj.y, p);
//    }
//    return proj;
//}

// TODO: Any simpler way to do this?
//bool VBVH::depthOrder(uint32_t triIndex1, uint32_t triIndex2) const
//{
//    ASSERT(!(triIndex1 == kBackgroundTriIndex && triIndex2 == kBackgroundTriIndex));
//    if (triIndex1 == kBackgroundTriIndex) return false;
//    if (triIndex2 == kBackgroundTriIndex) return true;
//    uint32_t offset1 = 3 * triIndex1;
//    Vector3 tri1[3] = {
//        vertexBuffer[indexBuffer[offset1 + 0]],
//        vertexBuffer[indexBuffer[offset1 + 1]],
//        vertexBuffer[indexBuffer[offset1 + 2]],
//    };
//    uint32_t offset2 = 3 * triIndex2;
//    Vector3 tri2[3] = {
//        vertexBuffer[indexBuffer[offset2 + 0]],
//        vertexBuffer[indexBuffer[offset2 + 1]],
//        vertexBuffer[indexBuffer[offset2 + 2]],
//    };
//
//    Vector3 n1 = cross(tri1[1] - tri1[0], tri1[2] - tri1[1]);
//    if (n1.z > 0.0) n1 = -n1;
//    ASSERT(dot(n1, n1) > 0.0);
//    Vector3 n2 = cross(tri2[1] - tri2[0], tri2[2] - tri2[1]);
//    if (n2.z > 0.0) n2 = -n2;
//    ASSERT(dot(n2, n2) > 0.0);
//
//    Vector2 proj1 = projToAxis(tri1, n2, tri2[0]);
//    if (proj1.x > 0.0 || proj1.x == 0.0 && proj1.y > proj1.x) {
//        return true;
//    }
//    if (proj1.y < 0.0 || proj1.y == 0.0 && proj1.x < proj1.y) {
//        return false;
//    }
//    Vector2 proj2 = projToAxis(tri2, n1, tri1[0]);
//    if (proj2.x > 0.0 || proj2.x == 0.0 && proj2.y > proj2.x) {
//        return false;
//    }
//    if (proj2.y < 0.0 || proj2.y == 0.0 && proj2.x < proj2.y) {
//        return true;
//    }
//
//    Vector3 e1[3] = { tri1[1] - tri1[0], tri1[2] - tri1[1], tri1[0] - tri1[2] };
//    Vector3 e2[3] = { tri2[1] - tri2[0], tri2[2] - tri2[1], tri2[0] - tri2[2] };
//    Vector3 test1[3] = { tri1[2] - tri1[0], tri1[0] - tri1[1], tri1[1] - tri1[2] };
//
//    for (uint32_t i1 = 0; i1 < 3; ++i1) {
//        for (uint32_t i2 = 0; i2 < 3; ++i2) {
//            Vector3 n = cross(e1[i1], e2[i2]);
//            if (n.z == 0.0) continue;
//            if (n.z > 0.0) n = -n;
//
//            proj1 = Vector2(0.0f);
//            float p1 = dot(test1[i1], n);
//            proj1.x = std::min(proj1.x, p1);
//            proj1.y = std::max(proj1.y, p1);
//            proj2 = projToAxis(tri2, n, tri1[i1]);
//
//            if (proj1.x > proj2.y ||
//                (proj1.x == proj2.y && (proj1.y > proj1.x || proj2.x < proj2.y))) {
//                return true;
//            }
//            if (proj2.x > proj1.y ||
//                (proj2.x == proj1.y && (proj2.y > proj2.x || proj1.x < proj1.y))) {
//                return false;
//            }
//        }
//    }
//
//    // Arbitrarily return false if two triangles are really close. This shouldn't be critical.
//    return false;
//}

}