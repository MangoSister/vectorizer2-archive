#include "polygonShared.h"
#include "util.h"

namespace vtrz
{

enum ClipFlag
{
    eLeft   = 1 << 0,
    eRight  = 1 << 1,
    eBottom = 1 << 2,
    eTop    = 1 << 3,
};

struct ClipData
{
    Vector2I64 *ptr;
    uint32_t n;
};

template <ClipFlag cf>
bool inside(const Vector2I64 &v, int64_t c) { static_assert(false, "Invalid specialization."); }
template <>
bool inside<ClipFlag::eLeft>(const Vector2I64 &v, int64_t c) { return v.x >= c; }
template <>
bool inside<ClipFlag::eRight>(const Vector2I64 &v, int64_t c) { return v.x <= c; }
template <>
bool inside<ClipFlag::eBottom>(const Vector2I64 &v, int64_t c) { return v.y >= c; }
template <>
bool inside<ClipFlag::eTop>(const Vector2I64 &v, int64_t c) { return v.y <= c; }

template <ClipFlag cf>
Vector2I64 clip(const Vector2I64 &v1, const Vector2I64 &v2, int64_t c) { static_assert(false, "Invalid specialization."); }
template <>
Vector2I64 clip<ClipFlag::eLeft>(const Vector2I64 &v1, const Vector2I64 &v2, int64_t c)
{
    return Vector2I64(c, lerpRound(v1.y, v2.y, double(c - v1.x) / double(v2.x - v1.x)));
}
template <>
Vector2I64 clip<ClipFlag::eRight>(const Vector2I64 &v1, const Vector2I64 &v2, int64_t c)
{
    return Vector2I64(c, lerpRound(v1.y, v2.y, double(c - v1.x) / double(v2.x - v1.x)));
}
template <>
Vector2I64 clip<ClipFlag::eBottom>(const Vector2I64 &v1, const Vector2I64 &v2, int64_t c)
{
    return Vector2I64(lerpRound(v1.x, v2.x, double(c - v1.y) / double(v2.y - v1.y)), c);
}
template <>
Vector2I64 clip<ClipFlag::eTop>(const Vector2I64 &v1, const Vector2I64 &v2, int64_t c)
{
    return Vector2I64(lerpRound(v1.x, v2.x, double(c - v1.y) / double(v2.y - v1.y)), c);
}

template <ClipFlag cf>
bool planeClip(ClipData &in, ClipData &out, int64_t c)
{
    out.n = 0;
    in.ptr[in.n] = in.ptr[0];
    bool insideStart = inside<cf>(in.ptr[0], c);
    for (uint32_t j = 0; j < in.n; ++j) {
        bool insideEnd = inside<cf>(in.ptr[j + 1], c);
        if (insideStart && insideEnd) {
            out.ptr[out.n++] = in.ptr[j + 1];
        } else if (insideStart && !insideEnd) {
            out.ptr[out.n++] = clip<cf>(in.ptr[j], in.ptr[j + 1], c);
        } else if (!insideStart && insideEnd) {
            out.ptr[out.n++] = clip<cf>(in.ptr[j + 1], in.ptr[j], c);
            out.ptr[out.n++] = in.ptr[j + 1];
        }
        // !insideStart && !insideEnd: do nothing.
        insideStart = insideEnd;
    }
    return out.n >= 3;
}

int64_t convexBoundIsectArea(
    const Vector2I64 &min, const Vector2I64 &max,
    const Vector2I64 *poly, uint32_t vertCount)
{
    uint8_t rejectFlags = 0x0F;
    uint8_t clipFlags = 0;
    for (uint32_t i = 0; i < vertCount; ++i) {
        uint8_t vertClipFlags = 0;
        vertClipFlags |= (poly[i].x < min.x) ? ClipFlag::eLeft : 0;
        vertClipFlags |= (poly[i].x > max.x) ? ClipFlag::eRight : 0;
        vertClipFlags |= (poly[i].y < min.y) ? ClipFlag::eBottom : 0;
        vertClipFlags |= (poly[i].y > max.y) ? ClipFlag::eTop : 0;

        rejectFlags &= vertClipFlags;
        clipFlags |= vertClipFlags;
    }
    if (rejectFlags) {
        // All out.
        return 0;
    }

    uint32_t maxClipVertCount = vertCount + 4;
    VLA(scratch1, Vector2I64, maxClipVertCount);
    VLA(scratch2, Vector2I64, maxClipVertCount);
    memcpy(scratch1, poly, sizeof(Vector2I64) *vertCount);

    ClipData in{ scratch1, vertCount };
    ClipData out{ scratch2, 0 };

    if (clipFlags & ClipFlag::eLeft) {
        if (!planeClip<ClipFlag::eLeft>(in, out, min.x)) return 0;
        std::swap(in, out);
    }
    if (clipFlags & ClipFlag::eRight) {
        if (!planeClip<ClipFlag::eRight>(in, out, max.x)) return 0;
        std::swap(in, out);
    }
    if (clipFlags & ClipFlag::eBottom) {
        if (!planeClip<ClipFlag::eBottom>(in, out, min.y)) return 0;
        std::swap(in, out);
    }
    if (clipFlags & ClipFlag::eTop) {
        if (!planeClip<ClipFlag::eTop>(in, out, max.y)) return 0;
        std::swap(in, out);
    }
    return convexArea2(in.ptr, in.n);
}

bool convexBoundIsectBool(
    const Vector2I64 &extent, const Vector2I64 &sum,
    const Vector2I64 *poly, uint32_t vertCount)
{
    for (uint32_t i = 0; i < vertCount; ++i) {
        const Vector2I64 &p = poly[i];
        const Vector2I64 &q = poly[(i + 1) % vertCount];
        Vector2I64 n(q.y - p.y, p.x - q.x);
        Vector2I64 absn(std::abs(n.x), std::abs(n.y));
        int64_t e = dot(absn, extent);
        int64_t s = dot(n, sum);
        int64_t k = 2 * dot(n, p);
        if (s - e >= k) {
            return false;
        }
    }
    return true;
}

}