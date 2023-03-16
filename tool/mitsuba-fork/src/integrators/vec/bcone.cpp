#include "bcone.h"
#include <mitsuba/core/random.h>

MTS_NAMESPACE_BEGIN

static SphericalCircle minBoundingConeWithTwoPoints(
    Vector *directions, uint32_t count, const Vector &q1, const Vector &q2, Random &rnd) {

    rnd.shuffle(directions, directions + count);

    Vector3 n = normalize(q1 + q2);
    SphericalCircle c(n, dot(n, q1));

    for (uint32_t i = 0; i < count; ++i) {
        if (!c.contain(directions[i])) {
            c = SphericalCircle(q1, q2, directions[i]);
        }
    }

    return c;
}

static SphericalCircle minBoundingConeWithOnePoint(
    Vector *directions, uint32_t count, const Vector &q, Random &rnd) {

    rnd.shuffle(directions, directions + count);

    Vector3 n = normalize(directions[0] + q);
    SphericalCircle c(n, dot(n, q));

    for (uint32_t i = 1; i < count; ++i) {
        if (!c.contain(directions[i])) {
            c = minBoundingConeWithTwoPoints(directions, i, directions[i], q, rnd);
        }
    }

    return c;
}

SphericalCircle minBoundingCone(Vector *directions, uint32_t count) {

    ref<Random> rnd = new Random(); // TODO: save this?
    rnd->shuffle(directions, directions + count);

    Vector3 n = normalize(directions[1] + directions[0]);
    SphericalCircle c(n, dot(n, directions[0]));

    for (uint32_t i = 2; i < count; ++i) {
        if (!c.contain(directions[i])) {
            c = minBoundingConeWithOnePoint(directions, i, directions[i], *rnd);
        }
    }

    return c;
}

MTS_NAMESPACE_END