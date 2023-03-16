#include "polygongrad.h"
#include "spatialGrid.h"
#include <algorithm>

namespace vtrz
{

static inline char segSegParallelIsectGrad(
    const DVector2I64 &a, const DVector2I64 &b,
    const DVector2I64 &c, const DVector2I64 &d);

static inline char segSegIsectGrad(
    const DVector2I64 &a, const DVector2I64 &b,
    const DVector2I64 &c, const DVector2I64 &d,
    ddouble &s, ddouble &t)
{
    int64_t q1 = a.x * (d.y - c.y);
    int64_t q2 = c.x * (a.y - b.y);

    int64_t denom =
        q1 +
        b.x * (c.y - d.y) +
        d.x * (b.y - a.y) +
        q2;

    /* If denom is zero, then segments are parallel: handle separately. */
    if (denom == 0)
        return segSegParallelIsectGrad(a, b, c, d);

    int64_t nums =
        q1 +
        c.x * (a.y - d.y) +
        d.x * (c.y - a.y);

    if (((nums > 0) && (nums > denom)) || ((nums < 0) && (nums < denom))) {
        return '0';
    }

    int64_t numt =
        - a.x * (c.y - b.y)
        - b.x * (a.y - c.y)
        + q2;

    if (((numt > 0) && (numt > denom)) || ((numt < 0) && (numt < denom))) {
        return '0';
    }

    dfloat dq1 = a.d.x * (d.d.y - c.d.y);
    dfloat dq2 = c.d.x * (a.d.y - b.d.y);

    dfloat ddenom =
        dq1 +
        b.d.x * (c.d.y - d.d.y) +
        d.d.x * (b.d.y - a.d.y) +
        dq2;
    ddenom.value = (float)((double)denom * kInvScaleFactor * kInvScaleFactor);

    dfloat dnums =
        dq1 +
        c.d.x * (a.d.y - d.d.y) +
        d.d.x * (c.d.y - a.d.y);
    s.value = (double)nums / (double)denom;
    s.grad = (dnums / ddenom).grad;

    dfloat dnumt =
        - a.d.x * (c.d.y - b.d.y)
        - b.d.x * (a.d.y - c.d.y)
        + dq2;
    t.value = (double)numt / (double)denom;
    t.grad = (dnumt / ddenom).grad;

    if ((nums == 0) || (nums == denom)) return 'v';
    if ((numt == 0) || (numt == denom)) return 'v';
    return '1';
}

static inline char segSegParallelIsectGrad(
    const DVector2I64 &a, const DVector2I64 &b,
    const DVector2I64 &c, const DVector2I64 &d)
{
    Vector2I64 aval = a.value();
    Vector2I64 bval = b.value();
    Vector2I64 cval = c.value();
    Vector2I64 dval = d.value();

    if (!collinear(aval, bval, cval)) {
        return '0';
    }
    if (between(aval, bval, cval) && between(aval, bval, dval)) {
        return 'e';
    }
    if (between(cval, dval, aval) && between(cval, dval, bval)) {
        return 'e';
    }
    if (between(aval, bval, cval) && between(cval, dval, bval)) {
        return 'e';
    }
    if (between(aval, bval, cval) && between(cval, dval, aval)) {
        return 'e';
    }
    if (between(aval, bval, dval) && between(cval, dval, bval)) {
        return 'e';
    }
    if (between(aval, bval, dval) && between(cval, dval, aval)) {
        return 'e';
    }
    return '0';
}

static inline char lineSegIsectGrad(
    const DVector2I64 &a, const DVector2I64 &b, // ab is line.
    const DVector2I64 &c, const DVector2I64 &d, // cd is seg.
    ddouble &t)
{
    // double t;           /* The parameters of cd's parametric eqns. */
    int64_t num, denom;  /* Numerator and denoninator of equations. */
    dfloat dnum, ddenom;
    char code = '?';    /* Return char characterizing intersection. */

    denom =
        a.x * (d.y - c.y) +
        b.x * (c.y - d.y) +
        d.x * (b.y - a.y) +
        c.x * (a.y - b.y);

    ddenom =
        a.d.x * (d.d.y - c.d.y) +
        b.d.x * (c.d.y - d.d.y) +
        d.d.x * (b.d.y - a.d.y) +
        c.d.x * (a.d.y - b.d.y);
    ddenom.value = (float)((double)denom * kInvScaleFactor * kInvScaleFactor);

    if (denom == 0) {
        if (!collinear(a.value(), b.value(), c.value())) {
            return '0';
        } else {
            return 'e';
        }
    }

    num =
        -(a.x * (c.y - b.y) +
        b.x * (a.y - c.y) +
        c.x * (b.y - a.y));

    dnum =
        -(a.d.x * (c.d.y - b.d.y) +
        b.d.x * (a.d.y - c.d.y) +
        c.d.x * (b.d.y - a.d.y));

    if ((num == 0) || (num == denom)) code = 'v';
    t.value = (double)num / (double)denom;
    t.grad = (dnum / ddenom).grad;

    if ((0.0 < t.value) && (t.value < 1.0))
        code = '1';
    else if ((0.0 > t.value) || (t.value > 1.0))
        code = '0';

    if (((num > 0) && (num < denom)) || ((num < 0) && (num > denom))) {
        code = '1';
    } else if
        (((num > 0) && (num > denom)) || ((num < 0) && (num < denom))) {
        code = '0';
    }

    return code;
}

static inline char raySegIsectGrad(
    const DVector2I64 &a, const DVector2I64 &b, // ab is ray (a is origin).
    const DVector2I64 &c, const DVector2I64 &d, // cd is seg.
    ddouble &t)
{
    // double  s, t;       /* The two parameters of the parametric eqns. */
    int64_t nums, numt, denom;  /* Numerator and denoninator of equations. */
    dfloat dnums, dnumt, ddenom;

    char code = '?';    /* Return char characterizing intersection. */

    denom =
        a.x * (d.y - c.y) +
        b.x * (c.y - d.y) +
        d.x * (b.y - a.y) +
        c.x * (a.y - b.y);

    ddenom =
        a.d.x * (d.d.y - c.d.y) +
        b.d.x * (c.d.y - d.d.y) +
        d.d.x * (b.d.y - a.d.y) +
        c.d.x * (a.d.y - b.d.y);
    ddenom.value = (float)((double)denom * kInvScaleFactor * kInvScaleFactor);

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

    dnumt =
        -(a.d.x * (c.d.y - b.d.y) +
        b.d.x * (a.d.y - c.d.y) +
        c.d.x * (b.d.y - a.d.y));

    if ((numt == 0) || (numt == denom)) code = 'v';
    t.value = (double)numt / (double)denom;
    t.grad = (dnumt / ddenom).grad;

    if (((nums > 0 && denom > 0) || (nums < 0 && denom < 0)) &&
        (((numt > 0) && (numt < denom)) || ((numt < 0) && (numt > denom))))
        code = '1';
    else if
        (((nums > 0 && denom < 0) || (nums < 0 && denom > 0)) ||
        (((numt > 0) && (numt > denom)) || ((numt < 0) && (numt < denom))))
        code = '0';

    return code;
}

//////////////////////////////////////////////////////////////////////////

ConvexIntersectorGrad::ConvexIntersectorGrad(const SpatialGrid<DVector2I64> *snapGrid) :
    snapGrid(snapGrid),
    isect(&allocator),
    diffChainsP(&allocator),
    diffChainsQ(&allocator),
    splitPolys(&allocator),
    convexHull(&allocator) {}

void ConvexIntersectorGrad::reset(
    int newSizeP, int newSizeQ,
    const DVector2I64 *newP, const DVector2I64 *newQ)
{
    sizeP = newSizeP; sizeQ = newSizeQ;
    P = newP; Q = newQ;

    isect = DynamicArray<DVector2I64>(&allocator);
    diffChainsP = DynamicArray<DynamicArray<IsectPointGrad>>(&allocator);
    diffChainsQ = DynamicArray<DynamicArray<IsectPointGrad>>(&allocator);
    splitPolys = DynamicArray<DynamicArray<DVector2I64>>(&allocator);

    convexHull.reset();
    // Reset memory.
    allocator.reset();
}

template <typename Container, typename Element>
bool insertUnique(Container &output, const Element &elem) {
    if (output.empty() || (elem != output.back() && elem != output.front())) {
        output.push_back(elem);
        return true;
    }
    return false;
}

static inline InFlag inOut(const IsectPointGrad &p, InFlag inflag, bool initialRotate, int aHB, int bHA, ConvexIntersectorGrad &ctx);
static inline int advance(int a, int &aa, int n,
    bool inside, bool initialRotate, bool diffOpen, bool diffChain,
    const DVector2I64 &v, ConvexIntersectorGrad &ctx);
static inline int advanceColl(int a, int &aa, int n, bool diffOpen, bool diffChain,
    const DVector2I64 &v, ConvexIntersectorGrad &ctx);

bool ConvexIntersectorGrad::intersect()
{
    int     a, b;           /* indices on P and Q (resp.) */
    int     a1, b1;         /* a-1, b-1 (resp.) */
    Vector2I64 A, B;        /* directed edges on P and Q (resp.) */
    int     cross;          /* sign of z-component of A x B */
    int     bHA, aHB;       /* b in H(A); a in H(b). */
    ddouble s, t;
    InFlag  inflag;         /* {Pin, Qin, Unknown}: which inside */
    int     aa, ba;         /* # advances on a & b indices (after 1st inter.) */
    bool    initialRotate;     /* Is this the first point? (used to initialize).*/
    char    code;           /* SegSegInt return code. */
    bool    didIsect;

    /* Initialize variables. */
    int n = sizeP, n2 = n * 2;
    int m = sizeQ, m2 = m * 2;
    a = 0; b = 0; aa = 0; ba = 0;
    inflag = InFlag::Unknown;
    initialRotate = true;
    didIsect = false;

    do {
        /* Computations of key variables. */
        a1 = a - 1; if (a1 < 0) a1 = n - 1; // a1 = (a + n - 1) % n;
        b1 = b - 1; if (b1 < 0) b1 = m - 1; // b1 = (b + m - 1) % m;

        A = P[a].value() - P[a1].value();
        B = Q[b].value() - Q[b1].value();

        cross = sgn(vtrz::cross(A, B)); // areaSign(Vector2I64(0), A, B);
        aHB = areaSign(Q[b1].value(), Q[b].value(), P[a].value());
        bHA = areaSign(P[a1].value(), P[a].value(), Q[b].value());

        /* If A & B intersect, update inflag. */
        code = segSegIsectGrad(P[a1], P[a], Q[b1], Q[b], s, t);
        if (code == '1' || code == 'v') {
            didIsect = true;
            IsectPointGrad p = createIsectPoint(a1, a, b1, b, s, t);
            inflag = inOut(p, inflag, initialRotate, aHB, bHA, *this);
            if (inflag == InFlag::Qin && initialRotate) { // TODO: comment.
                aa = ba = 0;
                initialRotate = false;
            }
        }

        /*-----Advance rules-----*/

        /* Special case: A & B overlap and oppositely oriented. */
        if ((code == 'e') && (dot(A, B) < 0)) {
            return false; // ConvexIsectResult::eEdge;
        }

        /* Special case: A & B parallel and separated. */
        else if ((cross == 0) && (aHB < 0) && (bHA < 0)) {
            return false;// ConvexIsectResult::eDisjoint;
        }

        /* Special case: A & B collinear. */
        else if ((cross == 0) && (aHB == 0) && (bHA == 0)) {
            /* Advance but do not output point. */
            // TODO: need comment.
            if (inflag != InFlag::Qin)
                b = advanceColl(b, ba, m, false, false, Q[b], *this);
            else
                a = advanceColl(a, aa, n, true, true, P[a], *this);
        }

        /* Generic cases. */
        else if (cross >= 0) {
            if (bHA > 0)
                a = advance(a, aa, n, inflag == InFlag::Pin, initialRotate, inflag == InFlag::Qin, true, P[a], *this);
            else
                b = advance(b, ba, m, inflag == InFlag::Qin, initialRotate, inflag == InFlag::Qin, false, Q[b], *this);
        } else /* if ( cross < 0 ) */ {
            if (aHB > 0)
                b = advance(b, ba, m, inflag == InFlag::Qin, initialRotate, inflag == InFlag::Qin, false, Q[b], *this);
            else
                a = advance(a, aa, n, inflag == InFlag::Pin, initialRotate, inflag == InFlag::Qin, true, P[a], *this);
        }

        /* Quit when both adv. indices have cycled, or one has cycled twice. */
    } while (((aa < n) || (ba < m)) && (aa < n2) && (ba < m2));

    ASSERT(diffChainsP.size() == diffChainsQ.size());

    if (inflag != InFlag::Unknown) {
        if (isect.empty() && didIsect) return false; // ConvexIsectResult::eVertex;
        if (!postProcess(isect)) return false; // ConvexIsectResult::eDisjoint;

        // TODO: need comment to explain.
        if (inflag == InFlag::Qin) {
            diffChainsP.pop_back();
            diffChainsQ.pop_back();
        }

        // Degeneracy handling:  Q is subset of P, but one of Q's vert is on P's boundary.
        //if (diffChainsP.size() == 1 && diffChainsP[0].size() > 1 &&
        //    diffChainsP[0].front() == diffChainsP[0].back()) {
        //    // Otherwise P is subset of Q.
        //    completeWithHole(P, Q, n, m, *this); // Q is subset of P.
        //} else {
        cleanDiffChains();
        //}

        return true; // ConvexIsectResult::ePoly;
    } else {
        // Deal with special cases where The boundaries of P and Q do not cross.
        // Can be containing, disjoint, or sharing a single vertex.
        if (didIsect) {
            // only sharing a single vertex.
            return false; // ConvexIsectResult::eVertex;
        }
        return intersectSpecial();
    }
}

IsectPointGrad ConvexIntersectorGrad::createIsectPoint(int a1, int a, int b1, int b, const ddouble &s, const ddouble &t) const
{
    // ASSERT(!isNan(grad));
    if (s.value == 0.0) return IsectPointGrad(P[a1]);
    if (s.value == 1.0) return IsectPointGrad(P[a]);
    if (t.value == 0.0) return IsectPointGrad(Q[b1]);
    if (t.value == 1.0) return IsectPointGrad(Q[b]);
    ImplicitPointGrad imp;
    imp.a = a1; imp.b = a; imp.s = s;
    imp.c = b1; imp.d = b; //imp.t = t;
    return IsectPointGrad(imp);
}

static InFlag inOut(const IsectPointGrad &p, InFlag inflag, bool initialRotate, int aHB, int bHA, ConvexIntersectorGrad &ctx)
{
    InFlag newflag = inflag; /* Keep status quo. */

    /* Update inflag. */
    if (aHB > 0) {
        if (inflag == InFlag::Qin) { // TODO: comment.
            // insertUnique(ctx.diffChainsP.back(), p);
            // TODO: need comment why not using insertUnique.
            if (ctx.diffChainsP.back().empty() || (p != ctx.diffChainsP.back().back())) {
                ctx.diffChainsP.back().push_back(p);
            }
        }
        newflag = InFlag::Pin;
    } else if (bHA > 0) {
        ctx.diffChainsP.push_back(DynamicArray<IsectPointGrad>(&ctx.allocator));
        ctx.diffChainsQ.push_back(DynamicArray<IsectPointGrad>(&ctx.allocator));
        insertUnique(ctx.diffChainsP.back(), p);
        newflag = InFlag::Qin;
    }

    if (!initialRotate || newflag == InFlag::Qin) {
        if (!p.implicit) {
            insertUnique(ctx.isect, p.exp);
        } else {
            insertUnique(ctx.isect, lerpRound(ctx.P[p.imp.a], ctx.P[p.imp.b], p.imp.s));
        }
    }

    return newflag;
}

static inline int advance(int a, int &aa, int n,
    bool inside, bool initialRotate, bool diffOpen, bool diffChain,
    const DVector2I64 &v, ConvexIntersectorGrad &ctx)
{
    if (inside && !initialRotate) {
        insertUnique(ctx.isect, v);
    }
    if (diffOpen) {
        if (diffChain) insertUnique(ctx.diffChainsP.back(), v);
        else insertUnique(ctx.diffChainsQ.back(), v);
    }
    ++aa;
    ++a; if (a == n) a = 0; // a = (a + 1) % n
    return a;
}

static inline int advanceColl(int a, int &aa, int n, bool diffOpen, bool diffChain,
    const DVector2I64 &v, ConvexIntersectorGrad &ctx)
{
    if (diffOpen) {
        if (diffChain) insertUnique(ctx.diffChainsP.back(), v);
        else insertUnique(ctx.diffChainsQ.back(), v);
    }
    ++aa;
    ++a; if (a == n) a = 0; // a = (a + 1) % n
    return a;
}

void ConvexIntersectorGrad::cleanDiffChains()
{
    // Degeneracy handling: remove degenerate polys (vert count < 3).
    int n = (int)diffChainsP.size();
    for (int i = 0; i < n; ) {
        int size = (int)diffChainsP[i].size() + (int)diffChainsQ[i].size();
        if (diffChainsQ[i].size() && diffChainsP[i].front() == diffChainsQ[i].back()) --size;
        if (diffChainsQ[i].size() && diffChainsP[i].back() == diffChainsQ[i].front()) --size;
        if (size < 3) {
            std::swap(diffChainsP[i], diffChainsP[n - 1]);
            std::swap(diffChainsQ[i], diffChainsQ[n - 1]);
            diffChainsP.pop_back();
            diffChainsQ.pop_back();
            --n;
        } else {
            ++i;
        }
    }
}

void ConvexIntersectorGrad::completeWithHole()
{
    diffChainsP.clear();
    diffChainsP.push_back(DynamicArray<IsectPointGrad>(&allocator));
    diffChainsP[0].clear();
    diffChainsP[0].reserve(sizeP);
    for (int i = 0; i < sizeP; ++i) {
        diffChainsP[0].push_back(P[i]);
    }
    isect.clear();
    isect.reserve(sizeQ);
    isect.reserve(sizeQ);
    diffChainsQ.clear();
    diffChainsQ.push_back(DynamicArray<IsectPointGrad>(&allocator));
    diffChainsQ[0].clear();
    diffChainsQ[0].reserve(sizeQ);
    for (int i = 0; i < sizeQ; ++i) {
        isect.push_back(Q[i]);
        diffChainsQ[0].push_back(Q[sizeQ - 1 - i]);
    }
}

bool ConvexIntersectorGrad::intersectSpecial()
{
    isect.clear();
    diffChainsP.clear();
    diffChainsQ.clear();

    bool pcIn = centerInOther(P, Q, sizeP, sizeQ);
    bool qcIn = centerInOther(Q, P, sizeQ, sizeP);
    if (!qcIn && !pcIn) {
        return false; // ConvexIsectResult::eDisjoint;
    }
    int64_t pa = convexArea2(P, sizeP);
    int64_t qa = convexArea2(Q, sizeQ);
    if (pa > qa) {
        completeWithHole();
        return true; // ConvexIsectResult::ePoly; // Q is subset of P.
    } else {
        isect.reserve(sizeP);
        for (int i = 0; i < sizeP; ++i) {
            isect.push_back(P[i]);
        }
        return true; // ConvexIsectResult::ePoly; // P is subset of Q.
    }
}

void ConvexIntersectorGrad::splitToConvex()
{
    splitPolys.clear();

    ASSERT(diffChainsP.size() == diffChainsQ.size());
    for (int i = 0; i < (int)diffChainsP.size(); ++i) {
        if (diffChainsQ[i].empty()) {
            DynamicArray<DVector2I64> poly(&allocator);
            poly.reserve(diffChainsP[i].size());
            for (int j = 0; j < (int)diffChainsP[i].size(); ++j) {
                if (!diffChainsP[i][j].implicit) insertUnique(poly, diffChainsP[i][j].exp);
                else insertUnique(poly, lerpRound(P[diffChainsP[i][j].imp.a], P[diffChainsP[i][j].imp.b], diffChainsP[i][j].imp.s));
             }
            splitPolys.push_back(std::move(poly));
            continue;
        } else if (diffChainsQ[i].size() == 1) {
            splitToConvex1(i);
        } else if (diffChainsQ[i].size() == 2) {
            splitToConvex2(i);
        } else if (diffChainsQ[i].size() == 3) {
            ASSERT(diffChainsP.size() == 1);
            splitToConvex3();
        } else {
            ASSERT_UNREACHABLE();
        }
    }

    int n = (int)splitPolys.size();
    for (int i = 0; i < n; ) {
        if (!postProcess(splitPolys[i])) {
            std::swap(splitPolys[i], splitPolys[n - 1]);
            splitPolys.pop_back();
            --n;
        } else {
            ++i;
        }
    }
}

void ConvexIntersectorGrad::splitToConvex1(int i)
{
    ASSERT(!diffChainsQ[i][0].implicit);
    DVector2I64 la, lb;
    la = diffChainsQ[i][0].exp;
    if (!diffChainsP[i][0].implicit) {
        lb = diffChainsP[i][0].exp;
    } else {
        lb = Q[diffChainsP[i][0].imp.c];
    }

    DVector2I64 vsplit;
    int offset = 0;
    for (int j = 1; j < (int)diffChainsP[i].size() - 1; ++j) {
        ASSERT(!diffChainsP[i][j].implicit);
        DVector2I64 c;
        c = diffChainsP[i][j].exp;
        DVector2I64 d;
        if (j < (int)diffChainsP[i].size() - 2 || !diffChainsP[i][j + 1].implicit) {
            d = diffChainsP[i][j + 1].exp;
        } else {
            d = P[diffChainsP[i][j + 1].imp.b];
        }
        ddouble t;
        char code = lineSegIsectGrad(la, lb, c, d, t);
        if (code == '1' || code == 'v') {
            vsplit = lerpRound(c, d, t);
            // TODO: need comment to explain degeneracy.
            offset = (code == '1' || t.value == 0.0) ? j + 1 : j + 2;
            break;
        }
    }
    ASSERT_FATAL(offset > 0);
    DynamicArray<DVector2I64> poly(&allocator);
    poly.reserve(offset + 1);
    for (int j = 0; j < offset; ++j) {
        if (!diffChainsP[i][j].implicit) insertUnique(poly, diffChainsP[i][j].exp);
        else insertUnique(poly, lerpRound(P[diffChainsP[i][j].imp.a], P[diffChainsP[i][j].imp.b], diffChainsP[i][j].imp.s));
    }
    insertUnique(poly, vsplit);
    splitPolys.push_back(std::move(poly));

    poly.clear();
    poly.reserve((int)diffChainsP[i].size() - offset + 2);
    poly.push_back(diffChainsQ[i][0].exp);
    poly.push_back(vsplit);
    for (int j = offset; j < (int)diffChainsP[i].size(); ++j) {
        if (!diffChainsP[i][j].implicit) insertUnique(poly, diffChainsP[i][j].exp);
        else insertUnique(poly, lerpRound(P[diffChainsP[i][j].imp.a], P[diffChainsP[i][j].imp.b], diffChainsP[i][j].imp.s));

    }
    splitPolys.push_back(std::move(poly));
}

void ConvexIntersectorGrad::splitToConvex2(int i)
{
    ASSERT(!diffChainsQ[i][0].implicit && !diffChainsQ[i][1].implicit);
    DVector2I64 la = diffChainsQ[i][1].exp;
    DVector2I64 lb = diffChainsQ[i][0].exp;

    int count = 0;
    DVector2I64 vsplits[2];
\
    int offsets[2];
    for (int j = 0; j < (int)diffChainsP[i].size() - 1; ++j) {
        DVector2I64 c;
        if (!diffChainsP[i][j].implicit) {
            c = diffChainsP[i][j].exp;
        } else {
            c = P[diffChainsP[i][j].imp.a];
        }
        DVector2I64 d;
        if (!diffChainsP[i][j + 1].implicit) {
            d = diffChainsP[i][j + 1].exp;
        } else {
            d = P[diffChainsP[i][j + 1].imp.b];
        }
        ddouble t = 0.0;
        char code = raySegIsectGrad(la, lb, c, d, t);
        if (code == '1' || code == 'v') {
            vsplits[count] = lerpRound(c, d, t);
            offsets[count] = j + 1;
            if (code == 'v') ++j;
            ++count;
            if (count == 1) {
                // First we shoot from diffChainsQ[i][1].v to diffChainsQ[i][0].v.
                // After found the first intersection we reverse the direction.
                // Using lineSegIsect cannot handle the edge case well:
                // diffChainsP[i] starts and ends on the same P edge.
                std::swap(la, lb);
            }
            if (count == 2) break;
        }
    }
    ASSERT_FATAL(count == 2);
    DynamicArray<DVector2I64> poly(&allocator);
    for (int j = 0; j < offsets[0]; ++j) {
        if (!diffChainsP[i][j].implicit) insertUnique(poly, diffChainsP[i][j].exp);
        else insertUnique(poly, lerpRound(P[diffChainsP[i][j].imp.a], P[diffChainsP[i][j].imp.b], diffChainsP[i][j].imp.s));
    }
    insertUnique(poly, vsplits[0]);
    insertUnique(poly, diffChainsQ[i][0].exp);
    splitPolys.push_back(std::move(poly));

    poly.clear();
    insertUnique(poly, vsplits[0]); // This is duplicate if code is 'v' for vsplit[0].
    for (int j = offsets[0]; j < offsets[1]; ++j) {
        if (!diffChainsP[i][j].implicit) insertUnique(poly, diffChainsP[i][j].exp);
        else insertUnique(poly, lerpRound(P[diffChainsP[i][j].imp.a], P[diffChainsP[i][j].imp.b], diffChainsP[i][j].imp.s));
    }
    insertUnique(poly, vsplits[1]);
    splitPolys.push_back(std::move(poly));

    poly.clear();
    insertUnique(poly, vsplits[1]); // This is duplicate if code is 'v' for vsplit[1].
    for (int j = offsets[1]; j < (int)diffChainsP[i].size(); ++j) {
        if (!diffChainsP[i][j].implicit) insertUnique(poly, diffChainsP[i][j].exp);
        else insertUnique(poly, lerpRound(P[diffChainsP[i][j].imp.a], P[diffChainsP[i][j].imp.b], diffChainsP[i][j].imp.s));
    }
    insertUnique(poly, diffChainsQ[i][1].exp);
    splitPolys.push_back(std::move(poly));
}

void ConvexIntersectorGrad::splitToConvex3()
{
    const auto &hole = diffChainsQ[0];
    DVector2I64 p[3];
    int idx[3] = { -1, -1, -1 };
    char codes[3] = { '0', '0', '0' };
    for (int i = 0; i < sizeP; ++i) {
        DVector2I64 curr = P[i];
        DVector2I64 next = P[(i + 1) % sizeP];
        for (int j = 0; j < 3; ++j) {
            if (codes[j] != '1' && codes[j] != 'v') {
                ddouble t;
                codes[j] = raySegIsectGrad(hole[j].exp, hole[(j + 1) % 3].exp, curr, next, t);
                if (codes[j] == '1' || codes[j] == 'v') {
                    p[j] = lerpRound(curr, next, t);
                    if (codes[j] == '1') idx[j] = i;
                    else if (codes[j] == 'v') idx[j] = p[j] == curr ? i : (i + 1) % sizeP;
                }
            }
        }
    }
    ASSERT_FATAL(idx[0] >= 0 && idx[1] >= 0 && idx[2] >= 0);
    DynamicArray<DVector2I64> poly(&allocator);
    for (int s = 0; s < 3; ++s) {
        poly.clear();
        int r = (s + 2) % 3;
        insertUnique(poly, hole[s].exp);
        insertUnique(poly, p[s]);
        for (int i = idx[s]; i != idx[r]; i = (i + 1) % sizeP) {
            insertUnique(poly, P[(i + 1) % sizeP]);
        }
        if (codes[r] == '1') {
            insertUnique(poly, p[r]);
        }
        splitPolys.push_back(std::move(poly));
    }
}

bool ConvexIntersectorGrad::postProcess(DynamicArray<DVector2I64> &poly)
{
    if (poly.size() < 3) return false;

    if (snapGrid) {
        for (uint32_t i = 0; i < poly.size(); ++i) {
            NNSearchResult<DVector2I64> result = snapGrid->nnSearch(poly[i].value());
            if (result.found) {
                poly[i] = result.neighbor;
            }
        }
    }



    if (poly.size() == 3) {
        int64_t a2 = convexArea2(poly.data(), 3);
        if (a2 == 0) return false;
        if (a2 < 0) std::swap(poly[1], poly[2]);
    } else {
        // Maybe also specialize when poly size == 4?
        // TODO: it's surprisingly annoying to test convexity if self-intersecting cases are present.
        convexHull.build(poly);
    }

    int64_t area2 = convexArea2(poly.data(), (int)poly.size()); // Do we actually need to re-calculate?
    constexpr int64_t kAreaThreshold = (int64_t)kScaleFactor >> 10; // "Adaptive" to kScaleFactor or input geometry?

    if (area2 < kAreaThreshold) {
        return false;
    }

    int64_t perimeterSq = 0;
    for (uint32_t i = 0; i < poly.size(); ++i) {
        Vector2I64 e = poly[i].value() - poly[(i + 1) % poly.size()].value();
        perimeterSq += dot(e, e);
    }

    constexpr double kThinnessThreshold = 1e-3;
    if ((double)area2 / (double)perimeterSq < kThinnessThreshold) {
        return false;
    }

    return true;
}

}