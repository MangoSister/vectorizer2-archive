#include <Catch2/single_include/catch2/catch.hpp>
#include "polygon.h"
#include "util.h"
#include <array>

using namespace vtrz;

//static int64_t toI64(double v, double scale) {
//    if (v < 0) {
//        return static_cast<int64_t>(v * scale - 0.5);
//    } else {
//        return static_cast<int64_t>(v * scale + 0.5);
//    }
//}
//
//TEST_CASE("wtf", "[wtf]") {
//    Vector2 P[] = {
//        Vector2(0.370804071, -0.388305098),
//        Vector2(1.00000000, 0.270587236),
//        Vector2(1.00000000, 0.529457211),
//        Vector2(0.370803982, -0.129435167),
//    };
//
//    Vector2 Q[] = {
//        Vector2(0.370804071, -0.388305098),
//        Vector2(0.618006766, -0.129435033),
//        Vector2(0.370804071, -0.129435033),
//    };
//
//    constexpr double scale = 1 << 20;
//
//    Vector2I64 Pi[countOf(P)];
//    for (uint32_t i = 0; i < countOf(P); ++i) {
//        Pi[i].x = toI64(P[i].x, scale);
//        Pi[i].y = toI64(P[i].y, scale);
//    }
//    Vector2I64 Qi[countOf(Q)];
//    for (uint32_t i = 0; i < countOf(Q); ++i) {
//        Qi[i].x = toI64(Q[i].x, scale);
//        Qi[i].y = toI64(Q[i].y, scale);
//    }
//
//
//    ConvexIntersector ctx;
//    ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//    ConvexIsectResult isect = ctx.intersect();
//    ctx.splitToConvex();
//    SUCCEED();
//}
//
//TEST_CASE("wtf-2", "[wtf]") {
//
//    Vector2I64 Pi[] = {
//        Vector2I64(-388816, -407167),
//        Vector2I64(-648027, -407167),
//        Vector2I64(-648027, -678612),
//    };
//    Vector2I64 Qi[] = {
//        Vector2I64(-388816, -678612),
//        Vector2I64(-129605, -407167),
//        Vector2I64(-388816, -407167),
//    };
//
//    ConvexIntersector ctx;
//    ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//    ConvexIsectResult isect = ctx.intersect();
//    ctx.splitToConvex();
//    SUCCEED();
//}
//
//TEST_CASE("wtf-3", "[wtf]") {
//
//    Vector2I64 Pi[] = {
//        Vector2I64(388816, -407167),
//        Vector2I64(388816, 407167),
//        Vector2I64(129605, 135722),
//        Vector2I64(129605, -407167),
//    };
//    Vector2I64 Qi[] = {
//        Vector2I64(129605, -407167),
//        Vector2I64(388816, -407167),
//        Vector2I64(388816, -135722),
//    };
//
//    ConvexIntersector ctx;
//    ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//    ConvexIsectResult isect = ctx.intersect();
//    ctx.splitToConvex();
//    SUCCEED();
//}
//
//TEST_CASE("wtf-4", "[wtf]") {
//    Vector2I64 Pi[] = {
//        Vector2I64(0, -170090),
//        Vector2I64(139376, -159475),
//        Vector2I64(140162, -149665),
//        Vector2I64(140162, -149652),
//        Vector2I64(0, 0),
//    };
//
//    Vector2I64 Qi[] = {
//        Vector2I64(140163, -149653),
//        Vector2I64(0, 0),
//        Vector2I64(0, -150789),
//    };
//
//    ConvexIntersector ctx;
//    ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//    ConvexIsectResult isect = ctx.intersect();
//    ctx.splitToConvex();
//    SUCCEED();
//}
//
//TEST_CASE("wtf-5", "[wtf]") {
//    Vector2I64 Pi[] = {
//        Vector2I64(-326854, 0),
//        Vector2I64(-247130, 285318),
//        Vector2I64(-1048576, 272870),
//        Vector2I64(-1048576, 0),
//    };
//
//    Vector2I64 Qi[] = {
//        Vector2I64(-386800, 0),
//        Vector2I64(-458803, 135155),
//        Vector2I64(-470097, 0),
//    };
//
//    ConvexIntersector ctx;
//    ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//    ConvexIsectResult isect = ctx.intersect();
//    ctx.splitToConvex();
//    SUCCEED();
//}
//
//TEST_CASE("wtf-6", "[wtf]") {
//	Vector2I64 Pi[] = {
//		Vector2I64(-320957, 366740),
//		Vector2I64(-320084, 372736),
//		Vector2I64(-323584, 372736),
//		Vector2I64(-323584, 369669),
//	};
//
//	Vector2I64 Qi[] = {
//		Vector2I64(-320958, 366740),
//		Vector2I64(-281708, 636123),
//		Vector2I64(-533869, 562722),
//	};
//
//	ConvexIntersector ctx;
//	ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//	ConvexIsectResult isect = ctx.intersect();
//	ctx.splitToConvex();
//	SUCCEED();
//}
//
//TEST_CASE("wtf-7", "[wtf]") {
//	Vector2I64 Pi[] = {
//		Vector2I64(171364, 67319),
//		Vector2I64(275762, 99181),
//		Vector2I64(252653, 126123),
//		Vector2I64(232419, 144630),
//        Vector2I64(225882, 148114),
//        Vector2I64(171235, 67329),
//	};
//
//	Vector2I64 Qi[] = {
//		Vector2I64(252653, 126123),
//		Vector2I64(241656, 133077),
//        Vector2I64(211694, 152021),
//	};
//    int64_t qa = cross(Qi[0], Qi[1]) + cross(Qi[1], Qi[2]) + cross(Qi[2], Qi[0]);
//	ConvexIntersector ctx;
//	ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//	ConvexIsectResult isect = ctx.intersect();
//	ctx.splitToConvex();
//	SUCCEED();
//}
//
//// splitToConvex2 edge case.
//TEST_CASE("wtf-8", "[wtf]") {
//	Vector2I64 Pi[] = {
//		Vector2I64(-667776, 136156),
//		Vector2I64(-618375, 94998),
//		Vector2I64(-601520, 120822),
//	};
//
//	Vector2I64 Qi[] = {
//		Vector2I64(-609704, 108285),
//		Vector2I64(-585833, 127640),
//		Vector2I64(-620311, 100605),
//	};
//	int64_t qa = cross(Qi[0], Qi[1]) + cross(Qi[1], Qi[2]) + cross(Qi[2], Qi[0]);
//	ConvexIntersector ctx;
//	ctx.reset(countOf(Pi), countOf(Qi), Pi, Qi);
//	ConvexIsectResult isect = ctx.intersect();
//	ctx.splitToConvex();
//	SUCCEED();
//}