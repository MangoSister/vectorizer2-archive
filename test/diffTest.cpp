#include <Catch2/single_include/catch2/catch.hpp>

#include "maths.h"
#include "diff.h"
#include "util.h"

using namespace vtrz;

static inline char segSegIsectGrad(
    const Vector2I64 &a, const Vector2I64 &b,
    const Vector2I64 &c, const Vector2I64 &d,
    const DVector2 &da, const DVector2 &db,
    const DVector2 &dc, const DVector2 &dd,
    double &s, double &t,
    dfloat &ds, dfloat &dt)
{
    char code = '?';    /* Return char characterizing intersection. */
    int64_t nums, numt, denom;  /* Numerator and denoninator of equations. */
    // double  s, t;       /* The two parameters of the parametric eqns. */

    denom =
        a.x * (d.y - c.y) +
        b.x * (c.y - d.y) +
        d.x * (b.y - a.y) +
        c.x * (a.y - b.y);

    /* If denom is zero, then segments are parallel: handle separately. */
    if (denom == 0)
        return '0';

    nums =
        a.x * (d.y - c.y) +
        c.x * (a.y - d.y) +
        d.x * (c.y - a.y);
    if ((nums == 0) || (nums == denom)) code = 'v';
    s = (double)nums / (double)denom;

    numt =
        -(a.x * (c.y - b.y) +
        b.x * (a.y - c.y) +
        c.x * (b.y - a.y));
    if ((numt == 0) || (numt == denom)) code = 'v';
    t = (double)numt / (double)denom;

    if ((((nums > 0) && (nums < denom)) || ((nums < 0) && (nums > denom))) &&
        (((numt > 0) && (numt < denom)) || ((numt < 0) && (numt > denom)))) {
        code = '1';
    } else if
        ((((nums > 0) && (nums > denom)) || ((nums < 0) && (nums < denom))) ||
        (((numt > 0) && (numt > denom)) || ((numt < 0) && (numt < denom)))) {
        code = '0';
    }

    if (code == '1' || code == 'v') {
        dfloat dnums, dnumt, ddenom;
        ddenom =
            da.x * (dd.y - dc.y) +
            db.x * (dc.y - dd.y) +
            dd.x * (db.y - da.y) +
            dc.x * (da.y - db.y);

        dnums =
            da.x * (dd.y - dc.y) +
            dc.x * (da.y - dd.y) +
            dd.x * (dc.y - da.y);

        dnumt =
            -(da.x * (dc.y - db.y) +
            db.x * (da.y - dc.y) +
            dc.x * (db.y - da.y));

        ds = dnums / ddenom;
        dt = dnumt / ddenom;
    }

    return code;
}

TEST_CASE("toy-1", "[diff]") {

    constexpr double scale = 1 << 20;

    Vector2 P[] = {
        Vector2(-0.4f, -0.5f),
        Vector2(0.6f, -0.5f),
        Vector2(0.6f, 0.5f),
        Vector2(-0.4f, 0.5f),
    };

    DVector2 dP[countOf(P)];
    for (uint32_t i = 0; i < countOf(P); ++i) {
        dP[i] = DVector2(dfloat(P[i].x, (uint32_t)0), dfloat(P[i].y));
    }

    Vector2I64 Pi[countOf(P)];
    for (uint32_t i = 0; i < countOf(P); ++i) {
        Pi[i].x = std::llround((double)P[i].x * scale);
        Pi[i].y = std::llround((double)P[i].y * scale);
    }

    Vector2 Q[] = {
        Vector2(0.0f, -0.70710678118f),
        Vector2(0.70710678118f, 0.0f),
        Vector2(0.0f, 0.70710678118f),
        Vector2(-0.70710678118f, 0.0f),
    };

    DVector2 dQ[countOf(Q)];
    for (uint32_t i = 0; i < countOf(Q); ++i) {
        dQ[i] = DVector2(dfloat(Q[i].x), dfloat(Q[i].y));
    }

    Vector2I64 Qi[countOf(Q)];
    for (uint32_t i = 0; i < countOf(Q); ++i) {
        Qi[i].x = std::llround((double)Q[i].x * scale);
        Qi[i].y = std::llround((double)Q[i].y * scale);
    }

    std::vector<Vector2I64> isect;
    std::vector<DVector2> dIsect;

    for (uint32_t i = 0; i < 4; ++i) {
        uint32_t pi = i;
        uint32_t pj = (i + 1) % 4;

        uint32_t qi = (i + 4 - 1) % 4;
        uint32_t qj = i;
        uint32_t qk = (i + 1) % 4;

        double s, t;
        dfloat ds, dt;

        segSegIsectGrad(
            Pi[pi], Pi[pj], Qi[qi], Qi[qj],
            dP[pi], dP[pj], dQ[qi], dQ[qj],
            s, t, ds, dt);
        Vector2I64 v = lerpRound(Pi[pi], Pi[pj], s);
        DVector2 dv = lerp(dP[pi], dP[pj], ds);
        isect.push_back(v);
        dIsect.push_back(dv);

        segSegIsectGrad(
            Pi[pi], Pi[pj], Qi[qj], Qi[qk],
            dP[pi], dP[pj], dQ[qj], dQ[qk],
            s, t, ds, dt);
        v = lerpRound(Pi[pi], Pi[pj], s);
        dv = lerp(dP[pi], dP[pj], ds);
        isect.push_back(v);
        dIsect.push_back(dv);
    }

    // convexArea2(isect.data(), (uint32_t)isect.size());
    uint32_t n = (uint32_t)isect.size();
    dfloat darea;
    for (uint32_t i = 0; i < n; ++i) {
        darea += cross(dIsect[i], dIsect[(i + 1) % n]);
    }
    darea *= 0.5f;

    SUCCEED();
}