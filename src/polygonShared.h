#pragma once

#include "maths.h"
#include "diff.h"

namespace vtrz
{

template <typename TPoint>
inline int64_t convexArea2(const TPoint *P, int n) {
    int64_t area = 0;
    for (int i = 0; i < n; ++i) {
        area += cross(getVal(P[i]), getVal(P[(i + 1) % n]));
    }
    return area;
}

template <typename TPoint>
inline bool pointInConvex(const Vector2I64 &p, const TPoint *poly, int n)
{
    for (int i = 0; i < n; ++i) {
        if (cross(getVal(poly[(i + 1) % n]) - getVal(poly[i]), p - getVal(poly[i])) < 0) {
            return false;
        }
    }
    return true;
}

template <typename TPoint>
inline bool centerInOther(const TPoint *P, const TPoint *Q, int n, int m) {
    Vector2I64 c(0);
    for (int i = 0; i < n; ++i) {
        c += getVal(P[i]);
    }
    for (int i = 0; i < m; ++i) {
        Vector2I64 a = getVal(Q[(i + 1) % m]) - getVal(Q[i]);
        Vector2I64 b = getVal(Q[i]);
        if (cross(a, c) - n * cross(a, b) < 0) {
            return false;
        }
    }
    return true;
}

struct ImplicitPoint
{
    // Record the input and output implicitly of segment intersection.
    // a: the index of point 1 of the seg 1.
    // b: the index of point 2 of the first seg 1.
    // s: the lerp parameter of the intersection, using the seg 1 eq.
    // c: the index of point 1 of the seg 2.
    // d: the index of point 2 of the first seg 2.
    // t: the lerp parameter of the intersection, using the seg 2 eq.

    int a, b;
    double s;
    int c, d;
    // double t; // t is never used.
};

inline bool operator==(const ImplicitPoint &p1, const ImplicitPoint &p2) {
    return (p1.a == p2.a && p1.b == p2.b && p1.s == p2.s) &&
        (p1.c == p2.c && p1.d == p2.d /*&& p1.imp.t == p2.imp.t*/);
}

enum class InFlag
{
    Pin,
    Qin,
    Unknown
};

inline int areaSign(const Vector2I64 &a, const Vector2I64 &b, const Vector2I64 &c)
{
    return sgn(cross(b - a, c - a));
}

inline bool collinear(const Vector2I64 &a, const Vector2I64 &b, const Vector2I64 &c)
{
    return cross(b - a, c - a) == 0;
}

/*---------------------------------------------------------------------
Returns TRUE iff point c lies on the closed segement ab.
Assumes it is already known that abc are collinear.
---------------------------------------------------------------------*/
inline bool between(const Vector2I64 &a, const Vector2I64 &b, const Vector2I64 &c)
{
    /* If ab not vertical, check betweenness on x; else on y. */
    if (a.x != b.x) {
        return ((a.x <= c.x) && (c.x <= b.x)) || ((a.x >= c.x) && (c.x >= b.x));
    } else {
        return ((a.y <= c.y) && (c.y <= b.y)) || ((a.y >= c.y) && (c.y >= b.y));
    }
}

inline char lineSegIsect(
    const Vector2I64 &a, const Vector2I64 &b, // ab is line.
    const Vector2I64 &c, const Vector2I64 &d, // cd is seg.
    double &t)
{
    // double t;           /* The parameters of cd's parametric eqns. */
    int64_t num, denom;  /* Numerator and denoninator of equations. */
    char code = '?';    /* Return char characterizing intersection. */

    denom =
        a.x * (d.y - c.y) +
        b.x * (c.y - d.y) +
        d.x * (b.y - a.y) +
        c.x * (a.y - b.y);

    if (denom == 0) {
        if (!collinear(a, b, c)) {
            return '0';
        } else {
            return 'e';
        }
    }

    num =
        -(a.x * (c.y - b.y) +
            b.x * (a.y - c.y) +
            c.x * (b.y - a.y));
    if ((num == 0) || (num == denom)) code = 'v';
    t = (double)num / (double)denom;

    if ((0.0 < t) && (t < 1.0))
        code = '1';
    else if ((0.0 > t) || (t > 1.0))
        code = '0';

    if (((num > 0) && (num < denom)) || ((num < 0) && (num > denom))) {
        code = '1';
    } else if
        (((num > 0) && (num > denom)) || ((num < 0) && (num < denom))) {
        code = '0';
    }

    return code;
}

inline char raySegIsect(
    const Vector2I64 &a, const Vector2I64 &b, // ab is ray (a is origin).
    const Vector2I64 &c, const Vector2I64 &d, // cd is seg.
    double &t)
{
    // double  s, t;       /* The two parameters of the parametric eqns. */
    int64_t nums, numt, denom;  /* Numerator and denoninator of equations. */
    char code = '?';    /* Return char characterizing intersection. */

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
    // Don't really need to know this case. The usage of raySegIsect to guaranteed to avoid
    // this case? Also only want to know if it's v for cd so that we modify how to advance
    // the testing loop.
    // if (nums == denom) code = 'v';

    numt =
        -(a.x * (c.y - b.y) +
            b.x * (a.y - c.y) +
            c.x * (b.y - a.y));
    if ((numt == 0) || (numt == denom)) code = 'v';
    t = (double)numt / (double)denom;

    if (((nums > 0 && denom > 0) || (nums < 0 && denom < 0)) &&
        (((numt > 0) && (numt < denom)) || ((numt < 0) && (numt > denom))))
        code = '1';
    else if
        (((nums > 0 && denom < 0) || (nums < 0 && denom > 0)) ||
        (((numt > 0) && (numt > denom)) || ((numt < 0) && (numt < denom))))
        code = '0';

    return code;
}

int64_t convexBoundIsectArea(
    const Vector2I64 &min, const Vector2I64 &max,
    const Vector2I64 *poly, uint32_t vertCount);

// Note we pass extent (max - min) and sum (max + min).
bool convexBoundIsectBool(
    const Vector2I64 &extent, const Vector2I64 &sum,
    const Vector2I64 *poly, uint32_t vertCount);

}