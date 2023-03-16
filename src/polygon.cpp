#include "polygon.h"
#include "spatialgrid.h"
#include <algorithm>

namespace vtrz
{

static char segSegParallelIsect(
    const Vector2I64 &a, const Vector2I64 &b,
    const Vector2I64 &c, const Vector2I64 &d);

static char segSegIsect(
    const Vector2I64 &a, const Vector2I64 &b,
    const Vector2I64 &c, const Vector2I64 &d,
    double &s, double &t)
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
        return segSegParallelIsect(a, b, c, d);

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

    s = (double)nums / (double)denom;
    t = (double)numt / (double)denom;

    if ((nums == 0) || (nums == denom)) return 'v';
    if ((numt == 0) || (numt == denom)) return 'v';
    return '1';
}

static char segSegParallelIsect(
    const Vector2I64 &a, const Vector2I64 &b,
    const Vector2I64 &c, const Vector2I64 &d)
{
    if (!collinear(a, b, c)) {
        return '0';
    }
    if (between(a, b, c) && between(a, b, d)) {
        return 'e';
    }
    if (between(c, d, a) && between(c, d, b)) {
        return 'e';
    }
    if (between(a, b, c) && between(c, d, b)) {
        return 'e';
    }
    if (between(a, b, c) && between(c, d, a)) {
        return 'e';
    }
    if (between(a, b, d) && between(c, d, b)) {
        return 'e';
    }
    if (between(a, b, d) && between(c, d, a)) {
        return 'e';
    }
    return '0';
}

ConvexIntersector::ConvexIntersector(const SpatialGrid<Vector2I64> *snapGrid) :
    snapGrid(snapGrid),
    isect(&allocator),
    diffChainsP(&allocator),
    diffChainsQ(&allocator),
    splitPolys(&allocator),
    convexHull(&allocator) {}

void ConvexIntersector::reset(int newSizeP, int newSizeQ, const Vector2I64 *newP, const Vector2I64 *newQ)
{
    sizeP = newSizeP; sizeQ = newSizeQ;
    P = newP; Q = newQ;

    isect = DynamicArray<Vector2I64>(&allocator);
    diffChainsP = DynamicArray<DynamicArray<IsectPoint>>(&allocator);
    diffChainsQ = DynamicArray<DynamicArray<IsectPoint>>(&allocator);
    splitPolys = DynamicArray<DynamicArray<Vector2I64>>(&allocator);
    convexHull.reset();
    // Reset memory.
    allocator.reset();
}

template <typename Container>
void insertUnique(Container &output, const IsectPoint &p);
template <typename Container>
void insertUnique(Container &output, const Vector2I64 &p);

static inline IsectPoint resolveIsectPoint(const Vector2I64 *P, const Vector2I64 *Q, int a1, int a, int b1, int b, double s, double t);
static inline InFlag inOut(const IsectPoint &p, InFlag inflag, bool initialRotate, int aHB, int bHA, ConvexIntersector &ctx);
static inline int advance(int a, int &aa, int n,
    bool inside, bool initialRotate, bool diffOpen, bool diffChain,
    const Vector2I64 &v, ConvexIntersector &ctx);
static inline int advanceColl(int a, int &aa, int n, bool diffOpen, bool diffChain,
    const Vector2I64 &v, ConvexIntersector &ctx);

bool ConvexIntersector::intersect()
{
    int     a, b;           /* indices on P and Q (resp.) */
    int     a1, b1;         /* a-1, b-1 (resp.) */
    Vector2I64 A, B;        /* directed edges on P and Q (resp.) */
    int     cross;          /* sign of z-component of A x B */
    int     bHA, aHB;       /* b in H(A); a in H(b). */
    double s, t;
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

        A = P[a] - P[a1];
        B = Q[b] - Q[b1];

        cross = sgn(vtrz::cross(A, B)); // areaSign(Vector2I64(0), A, B);
        aHB = areaSign(Q[b1], Q[b], P[a]);
        bHA = areaSign(P[a1], P[a], Q[b]);

        /* If A & B intersect, update inflag. */
        code = segSegIsect(P[a1], P[a], Q[b1], Q[b], s, t);
        if (code == '1' || code == 'v') {
            didIsect = true;
            IsectPoint p = resolveIsectPoint(P, Q, a1, a, b1, b, s, t);
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

template <typename Container>
void insertUnique(Container &output, const IsectPoint &p) {
    if (output.empty() || (p != output.back() && p != output.front())) {
        output.push_back(p);
    }
}

template <typename Container>
void insertUnique(Container &output, const Vector2I64 &p) {
    if (output.empty() || (p != output.back() && p != output.front())) {
        output.push_back(p);
    }
}

static inline IsectPoint resolveIsectPoint(const Vector2I64 *P, const Vector2I64 *Q, int a1, int a, int b1, int b, double s, double t)
{
    if (s == 0.0) return P[a1];
    if (s == 1.0) return P[a];
    if (t == 0.0) return Q[b1];
    if (t == 1.0) return Q[b];
    ImplicitPoint p;
    p.a = a1; p.b = a; p.s = s;
    p.c = b1; p.d = b; //p.t = t;
    return p;
}

static inline InFlag inOut(const IsectPoint &p, InFlag inflag, bool initialRotate, int aHB, int bHA, ConvexIntersector &ctx)
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
        ctx.diffChainsP.push_back(DynamicArray<IsectPoint>(&ctx.allocator));
        ctx.diffChainsQ.push_back(DynamicArray<IsectPoint>(&ctx.allocator));
        insertUnique(ctx.diffChainsP.back(), p);
        newflag = InFlag::Qin;
    }

    if (!initialRotate || newflag == InFlag::Qin) {
        if (!p.implicit) insertUnique(ctx.isect, p.exp);
        else insertUnique(ctx.isect, lerpRound(ctx.P[p.imp.a], ctx.P[p.imp.b], p.imp.s));
    }

    return newflag;
}

static inline int advance(int a, int &aa, int n,
    bool inside, bool initialRotate, bool diffOpen, bool diffChain,
    const Vector2I64 &v, ConvexIntersector &ctx)
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
    const Vector2I64 &v, ConvexIntersector &ctx)
{
    if (diffOpen) {
        if (diffChain) insertUnique(ctx.diffChainsP.back(), v);
        else insertUnique(ctx.diffChainsQ.back(), v);
    }
    ++aa;
    ++a; if (a == n) a = 0; // a = (a + 1) % n
    return a;
}

void ConvexIntersector::cleanDiffChains()
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

void ConvexIntersector::completeWithHole()
{
    diffChainsP.clear();
    diffChainsP.push_back(DynamicArray<IsectPoint>(&allocator));
    diffChainsP[0].clear();
    diffChainsP[0].reserve(sizeP);
    for (int i = 0; i < sizeP; ++i) {
        diffChainsP[0].push_back(P[i]);
    }
    isect.clear();
    isect.reserve(sizeQ);
    diffChainsQ.clear();
    diffChainsQ.push_back(DynamicArray<IsectPoint>(&allocator));
    diffChainsQ[0].clear();
    diffChainsQ[0].reserve(sizeQ);
    for (int i = 0; i < sizeQ; ++i) {
        isect.push_back(Q[i]);
        diffChainsQ[0].push_back(Q[sizeQ - 1 - i]);
    }
}

bool ConvexIntersector::intersectSpecial()
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

void ConvexIntersector::splitToConvex()
{
    splitPolys.clear();
    ASSERT(diffChainsP.size() == diffChainsQ.size());
    for (int i = 0; i < (int)diffChainsP.size(); ++i) {
        if (diffChainsQ[i].empty()) {
            DynamicArray<Vector2I64> poly(&allocator);
            poly.reserve((int)diffChainsP[i].size());
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

void ConvexIntersector::splitToConvex1(int i)
{
    ASSERT(!diffChainsQ[i][0].implicit);
    Vector2I64 la = diffChainsQ[i][0].exp;
    Vector2I64 lb;
    if (!diffChainsP[i][0].implicit) {
        lb = diffChainsP[i][0].exp;
    } else {
        lb = Q[diffChainsP[i][0].imp.c];
    }

    Vector2I64 vsplit;
    int offset = 0;
    for (int j = 1; j < (int)diffChainsP[i].size() - 1; ++j) {
        ASSERT(!diffChainsP[i][j].implicit);
        Vector2I64 c = diffChainsP[i][j].exp;
        Vector2I64 d;
        if (j < (int)diffChainsP[i].size() - 2 || !diffChainsP[i][j + 1].implicit) {
            d = diffChainsP[i][j + 1].exp;
        } else {
            d = P[diffChainsP[i][j + 1].imp.b];
        }
        double t = 0.0;
        char code = lineSegIsect(la, lb, c, d, t);
        if (code == '1' || code == 'v') {
            vsplit = lerpRound(c, d, t);
            // TODO: need comment to explain degeneracy.
            offset = (code == '1' || t == 0.0) ? j + 1 : j + 2;
            break;
        }
    }
    ASSERT_FATAL(offset > 0);
    DynamicArray<Vector2I64> poly(&allocator);
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

void ConvexIntersector::splitToConvex2(int i)
{
    ASSERT(!diffChainsQ[i][0].implicit && !diffChainsQ[i][1].implicit);
    Vector2I64 la = diffChainsQ[i][1].exp;
    Vector2I64 lb = diffChainsQ[i][0].exp;

    int count = 0;
    Vector2I64 vsplits[2];
    int offsets[2];
    for (int j = 0; j < (int)diffChainsP[i].size() - 1; ++j) {
        Vector2I64 c;
        if (!diffChainsP[i][j].implicit) {
            c = diffChainsP[i][j].exp;
        } else {
            c = P[diffChainsP[i][j].imp.a];
        }
        Vector2I64 d;
        if (!diffChainsP[i][j + 1].implicit) {
            d = diffChainsP[i][j + 1].exp;
        } else {
            d = P[diffChainsP[i][j + 1].imp.b];
        }
        double t = 0.0;
        char code = raySegIsect(la, lb, c, d, t);
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
    DynamicArray<Vector2I64> poly(&allocator);
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

void ConvexIntersector::splitToConvex3()
{
    const auto &hole = diffChainsQ[0];
    Vector2I64 p[3];
    int idx[3] = { -1, -1, -1 };
    char codes[3] = { '0', '0', '0' };
    for (int i = 0; i < sizeP; ++i) {
        Vector2I64 curr = P[i];
        Vector2I64 next = P[(i + 1) % sizeP];
        for (int j = 0; j < 3; ++j) {
            if (codes[j] != '1' && codes[j] != 'v') {
                double t;
                codes[j] = raySegIsect(hole[j].exp, hole[(j + 1) % 3].exp, curr, next, t);
                if (codes[j] == '1' || codes[j] == 'v') {
                    p[j] = lerpRound(curr, next, t);
                    if (codes[j] == '1') idx[j] = i;
                    else if (codes[j] == 'v') idx[j] = p[j] == curr ? i : (i + 1) % sizeP;
                }
            }
        }
    }
    ASSERT_FATAL(idx[0] >= 0 && idx[1] >= 0 && idx[2] >= 0);
    DynamicArray<Vector2I64> poly(&allocator);
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

bool ConvexIntersector::postProcess(DynamicArray<Vector2I64> &poly)
{
    if (poly.size() < 3) return false;

    if (snapGrid) {
        for (uint32_t i = 0; i < poly.size(); ++i) {
            NNSearchResult<Vector2I64> result = snapGrid->nnSearch(poly[i]);
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
        Vector2I64 e = poly[i] - poly[(i + 1) % poly.size()];
        perimeterSq += dot(e, e);
    }

    constexpr double kThinnessThreshold = 1e-3;
    if ((double)area2 / (double)perimeterSq < kThinnessThreshold) {
        return false;
    }

    return true;
}

}