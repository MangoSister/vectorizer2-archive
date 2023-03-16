#include "vhiz.h"
#include <array>
#include <fstream>

namespace vtrz
{

struct Rasterizer
{
    void init(VHiZ::TrianglePart part, const Tri2 &tri) {
        if (part == VHiZ::TrianglePart::eTop) {
            ASSERT(tri[1].x <= tri[2].x);
            invkl = (tri[0].x - tri[1].x) / (tri[0].y - tri[1].y);
            invkr = (tri[0].x - tri[2].x) / (tri[0].y - tri[2].y);
            yb = (int)std::ceil(tri[1].y);
            yt = (int)std::floor(tri[0].y);
            float t = (float)yb - tri[1].y;
            xl = tri[1].x + t * invkl;
            xr = tri[2].x + t * invkr;
        } else {
            ASSERT(tri[1].x >= tri[2].x);
            invkl = (tri[2].x - tri[0].x) / (tri[2].y - tri[0].y);
            invkr = (tri[1].x - tri[0].x) / (tri[1].y - tri[0].y);
            yb = (int)std::ceil(tri[0].y);
            yt = (int)std::floor(tri[1].y);
            float t = (float)yb - tri[0].y;
            xl = tri[0].x + t * invkl;
            xr = tri[0].x + t * invkr;
        }
        xli = (int)std::ceil(xl);
        xri = (int)std::floor(xr);
    }

    void initBC(VHiZ::TrianglePart part, const Tri3 &tri, int newWidth, int newHeight) {
        Tri2 tri2 = { (Vector2)tri[0], (Vector2)tri[1], (Vector2)tri[2] };
        init(part, tri2);

        // It's ok to ignore the boundary lattice points because they will be handled by
        // boundary function anyway.
        if (yb == 0) {
            // yb = std::max(yb, 1);
            yb = 1;
            nextRow();
        }
        yt = std::min(yt, newHeight - 1);
        xli = std::max(xli, 1);
        xri = std::min(xri, newWidth - 1);

        bc = barycentricCoordNoClamp(Vector2((float)xli, (float)yb), tri2);
        float invArea = 1.0f / cross(tri2[1] - tri2[0], tri2[2] - tri2[1]);
        bcdx = invArea * Vector3(tri[1].y - tri[2].y, tri[2].y - tri[0].y, tri[0].y - tri[1].y);
        bcdy = invArea * Vector3(tri[2].x - tri[1].x, tri[0].x - tri[2].x, tri[1].x - tri[0].x);
        width = newWidth;
    }

    void nextRow() {
        xl += invkl;
        xr += invkr;
        xli = (int)std::ceil(xl);
        xri = (int)std::floor(xr);
    }

    void nextPixelBC() {
        bc += bcdx;
    }

    void nextRowBC() {
        xl += invkl;
        xr += invkr;

        int newxli = std::max((int)std::ceil(xl), 1);
        bc += (float)(newxli - (xri + 1)) * bcdx;
        bc += bcdy;

        xli = newxli;
        xri = std::min((int)std::floor(xr), width - 1);
    }

    float invkl, invkr;
    int yb, yt;
    float xl, xr;
    int xli, xri;

    Vector3 bc;
    Vector3 bcdx, bcdy;
    int width;
};

template <typename T>
void sortTriVertically(std::array<T, 3> &tri)
{
    std::sort(tri.begin(), tri.end(), [](const T &p, const T &q) {
        if (p.y > q.y) return true;
        else if (p.y < q.y) return false;
        else return p.x <= q.x;
    });
}

template <typename T>
void splitToFlatTris(const std::array<T, 3> &tri, std::array<T, 3> &top, std::array<T, 3> &bottom)
{
    float t = (tri[1].y - tri[2].y) / (tri[0].y - tri[2].y);
    T tmp{};
    tmp.y = tri[1].y;
    for (uint32_t i = 0; i < T::kDim; ++i) {
        if (i != 1) {
            tmp[i] = lerp(tri[2][i], tri[0][i], t);
        }
    }

    top[0] = tri[0];
    if (tmp.x < tri[1].x) {
        top[1] = tmp; top[2] = tri[1];
    } else {
        top[1] = tri[1]; top[2] = tmp;
    }
    bottom[0] = tri[2];
    if (tmp.x < tri[1].x) {
        bottom[1] = tri[1]; bottom[2] = tmp;
    } else {
        bottom[1] = tmp; bottom[2] = tri[1];
    }
}

void VHiZ::reset(const AABB2 &newBound, uint32_t newWidth, uint32_t newHeight, DepthFunc newDepthFunc)
{
    bound = newBound;
    width = (int)newWidth;
    height = (int)newHeight;
    getCellSx = 2.0f / width;
    getCellSy = 2.0f / height;
    getCellTx = getCellSx - 1.0f;
    getCellTy = getCellSy - 1.0f;

    ndcScale = Vector2(1.0f) / bound.extents() * Vector2((float)width, (float)height);
    depthFunc = newDepthFunc;

    framebuffer.clear();
    framebuffer.resize(width * height, depthFunc == DepthFunc::eGreater ? 1.0f : 0.0f);
}

void VHiZ::recordOccludee(const Vector3 *poly, uint32_t vertCount)
{
    ASSERT(vertCount >= 3);
    for (uint32_t i = 0; i < vertCount - 2; ++i) {
        Tri3 tri{ poly[0], poly[i + 1], poly[i + 2] };
        recordOccludeeInterior(tri);
    }
    for (uint32_t i = 0; i < vertCount; ++i) {
        recordOccludeeBoundary(poly[i], poly[(i + 1) % vertCount]);
    }
}

void VHiZ::recordOccludeeInterior(Tri3 tri)
{
    if (cross((Vector2)tri[1] - (Vector2)tri[0], (Vector2)tri[2] - (Vector2)tri[1]) <= 0.0f) {
        return;
    }

    sortTriVertically(tri);

    if (tri[1].y == tri[2].y) {
        recordOccludeeInterior(TrianglePart::eTop, { tri[0], tri[1], tri[2] });
    } else if (tri[0].y == tri[1].y) {
        recordOccludeeInterior(TrianglePart::eBottom, { tri[2], tri[1], tri[0] });
    } else {
        Tri3 top;
        Tri3 bottom;
        splitToFlatTris(tri, top, bottom);
        recordOccludeeInterior(TrianglePart::eTop, top);
        recordOccludeeInterior(TrianglePart::eBottom, bottom);
    }
}

void VHiZ::recordOccludeeInterior(TrianglePart part, const Tri3 &tri)
{
    Rasterizer ras;
    ras.initBC(part, tri, width, height);

    for (int y = ras.yb; y <= ras.yt; ++y) {
        for (int x = ras.xli; x <= ras.xri; ++x) {
            int offset = y * width + x;
            ASSERT(ras.bc.x >= -1e-4f && ras.bc.x <= 1.0f + 1e-4f);
            ASSERT(ras.bc.y >= -1e-4f && ras.bc.y <= 1.0f + 1e-4f);
            ASSERT(ras.bc.z >= -1e-4f && ras.bc.z <= 1.0f + 1e-4f);
            ASSERT(ras.bc.x + ras.bc.y + ras.bc.z <= 1.0f + 1e-4f);
            float depth = tri[0].z * ras.bc[0] + tri[1].z * ras.bc[1] + tri[2].z * ras.bc[2];
            ASSERT(depth >= 0.0f && depth <= 1.0f);
            if (depthFunc == DepthFunc::eGreater) {
                framebuffer[offset] = std::min(framebuffer[offset], depth);
                framebuffer[offset - 1] = std::min(framebuffer[offset - 1], depth);
                framebuffer[offset - width] = std::min(framebuffer[offset - width], depth);
                framebuffer[offset - 1 - width] = std::min(framebuffer[offset - 1 - width], depth);
            } else {
                framebuffer[offset] = std::max(framebuffer[offset], depth);
                framebuffer[offset - 1] = std::max(framebuffer[offset - 1], depth);
                framebuffer[offset - width] = std::max(framebuffer[offset - width], depth);
                framebuffer[offset - 1 - width] = std::max(framebuffer[offset - 1 - width], depth);
            }
            ras.nextPixelBC();
        }
        ras.nextRowBC();
    }
}

struct Raymarcher
{
    void init(const Vector2 &start, const Vector2 &end) {
        origin = start;
        dir = (Vector2)end - start;
        // Also works with horizontal/vertical case.
        invDir = Vector2(1.0f) / dir;
        signDir = { !std::signbit(dir.x), !std::signbit(dir.y) };

        p = start;
        t = 0.0f;
    }

    void step() {
        float newx = signDir[0] ? std::floor(p.x + 1.0f) : std::ceil(p.x - 1.0f);
        float newy = signDir[1] ? std::floor(p.y + 1.0f) : std::ceil(p.y - 1.0f);

        float dtx = (newx - p.x) * invDir.x;
        float dty = (newy - p.y) * invDir.y;

        if (dtx < dty) {
            t += dtx;
            p.x = newx;
            p.y = origin.y + t * dir.y;
        } else {
            t += dty;
            p.y = newy;
            p.x = origin.x + t * dir.x;
        }
    }

    Vector2 origin;
    Vector2 dir;
    Vector2 invDir;
    std::array<bool, 2> signDir;
    Vector2 p;
    float t;

};

void VHiZ::recordOccludeeBoundary(const Vector3 &start, const Vector3 &end)
{
    // Basically 2d ray-marching...
    Raymarcher rm;
    rm.init((Vector2)start, (Vector2)end);

    while (rm.t < 1.0f) {
        float fx = std::floor(rm.p.x);
        float fy = std::floor(rm.p.y);
        bool flag[4];
        flag[0] = (rm.p.x < (float)width) && (rm.p.y < (float)height);
        flag[1] = (rm.p.x == fx) && (rm.p.x > 0.0f) && (rm.p.y < (float)height);
        flag[2] = (rm.p.y == fy) && (rm.p.y > 0.0f) && (rm.p.x < (float)width);
        flag[3] = (rm.p.x == fx) && (rm.p.x > 0.0f) && (rm.p.y == fy) && (rm.p.y > 0.0f);
        int offsets[4];
        offsets[0] = (int)fy * width + (int)fx;
        offsets[1] = offsets[0] - 1;
        offsets[2] = offsets[0] - width;
        offsets[3] = offsets[0] - 1 - width;

        float depth = lerp(start.z, end.z, rm.t);
        ASSERT(depth >= 0.0f && depth <= 1.0f);
        if (depthFunc == DepthFunc::eGreater) {
            for (int i = 0; i < 4; ++i) {
                if (flag[i]) framebuffer[offsets[i]] = std::min(framebuffer[offsets[i]], depth);
            }
        } else {
            for (int i = 0; i < 4; ++i) {
                if (flag[i]) framebuffer[offsets[i]] = std::max(framebuffer[offsets[i]], depth);
            }
        }

        rm.step();
    }
}

void VHiZ::recordOccluder(const Vector2 *poly, uint32_t vertCount, std::vector<Vector2I> &cellsToQuery)
{
    ASSERT(vertCount >= 3);
    occluderBuffer.clear();
    occluderBuffer.resize(height + 1, Vector2I(width + 1, -1));
    for (uint32_t i = 0; i < vertCount - 2; ++i) {
        Tri2 tri{ poly[0], poly[i + 1], poly[i + 2] };
        recordOccluderInterior(tri);
    }

    for (int y = 0; y < height; ++y) {
        int bl = occluderBuffer[y].x;
        int br = occluderBuffer[y].y;
        int tl = occluderBuffer[y + 1].x;
        int tr = occluderBuffer[y + 1].y;

        int ol = std::max(bl, tl);
        int or = std::min(br, tr);
        int l = std::min(bl, tl);
        int r = std::max(br, tr);
        if (ol >= or) {
            int start = std::max(l - 1, 0);
            int end = std::min(or , width - 1);
            for (int x = start; x <= end; ++x) {
                cellsToQuery.push_back(Vector2I(x, y));
            }
            start = std::max(ol - 1, 0) + std::max(2 - (ol - or ), 0);
            end = std::min(r, width - 1);
            for (int x = start; x <= end; ++x) {
                cellsToQuery.push_back(Vector2I(x, y));
            }
        } else {
            for (int x = std::max(l - 1, 0); x < ol; ++x) {
                cellsToQuery.push_back(Vector2I(x, y));
            }
            for (int x = or ; x <= std::min(r, width - 1); ++x) {
                cellsToQuery.push_back(Vector2I(x, y));
            }
            int rowOffset = y * width;
            std::fill(
                framebuffer.begin() + rowOffset + ol,
                framebuffer.begin() + rowOffset + or ,
                depthFunc == DepthFunc::eGreater ? 1.0f : 0.0f);
        }
    }
}

void VHiZ::recordOccluderInterior(Tri2 tri)
{
    sortTriVertically(tri);

    if (tri[1].y == tri[2].y) {
        recordOccluderInterior(TrianglePart::eTop, { tri[0], tri[1], tri[2] });
    } else if (tri[0].y == tri[1].y) {
        recordOccluderInterior(TrianglePart::eBottom, { tri[2], tri[1], tri[0] });
    } else {
        Tri2 top;
        Tri2 bottom;
        splitToFlatTris(tri, top, bottom);
        recordOccluderInterior(TrianglePart::eTop, top);
        recordOccluderInterior(TrianglePart::eBottom, bottom);
    }
}

void VHiZ::recordOccluderInterior(TrianglePart part, const Tri2 &tri)
{
    Rasterizer ras;
    ras.init(part, tri);

    for (int y = ras.yb; y <= ras.yt; ++y) {
        occluderBuffer[y].x = std::min(occluderBuffer[y].x, ras.xli);
        occluderBuffer[y].y = std::max(occluderBuffer[y].x, ras.xri);

        ras.nextRow();
    }
}

bool VHiZ::query(const AABB2 &box, float depth) const
{
    Vector2 min = convertToFrameCoord(box.min);
    Vector2 max = convertToFrameCoord(box.max);
    int xmin = (int)std::ceil(min.x);
    int ymin = (int)std::ceil(min.y);
    int xmax = (int)std::floor(max.x);
    int ymax = (int)std::floor(max.y);
    for (int y = ymin; y <= ymax; ++y) {
        int rowOffset = y * width;
        for (int x = xmin; x <= xmax; ++x) {
            if (depthFunc == DepthFunc::eGreater) {
                if (depth > framebuffer[rowOffset + x]) return false;
            } else {
                if (depth < framebuffer[rowOffset + x]) return false;
            }
        }
    }
    return true;
}

bool VHiZ::query(const Vector3 *vertexBuffer, uint32_t vertexCount) const
{
    for (uint32_t v = 0; v < vertexCount; v += 3) {
        Tri3 tri{ vertexBuffer[v], vertexBuffer[v + 1], vertexBuffer[v + 2] };
        for (uint32_t i = 0; i < 3; ++i) {
            tri[i] = convertToFrameCoord(tri[i]);
        }
        if (!query(tri)) {
            return false;
         }
    }
    return true;
}

bool VHiZ::query(Tri3 tri) const
{
    sortTriVertically(tri);

    if (tri[1].y == tri[2].y) {
        if (!queryInterior(TrianglePart::eTop, { tri[0], tri[1], tri[2] })) return false;
    } else if (tri[0].y == tri[1].y) {
        if (!queryInterior(TrianglePart::eBottom, { tri[2], tri[1], tri[0] })) return false;
    } else {
        Tri3 top;
        Tri3 bottom;
        splitToFlatTris(tri, top, bottom);
        if (!queryInterior(TrianglePart::eTop, top)) return false;
        if (!queryInterior(TrianglePart::eBottom, bottom)) return false;
    }

    for (uint32_t i = 0; i < 3; ++i) {
        if (!queryBoundary(tri[i], tri[(i + 1) % 3])) return false;
    }

    return true;
}

bool VHiZ::queryInterior(TrianglePart part, const Tri3 &tri) const
{
    Rasterizer ras;
    ras.initBC(part, tri, width, height);

    for (int y = ras.yb; y <= ras.yt; ++y) {
        for (int x = ras.xli; x <= ras.xri; ++x) {
            int offset = y * width + x;
            ASSERT(ras.bc.x >= -1e-4f && ras.bc.x <= 1.0f + 1e-4f);
            ASSERT(ras.bc.y >= -1e-4f && ras.bc.y <= 1.0f + 1e-4f);
            ASSERT(ras.bc.z >= -1e-4f && ras.bc.z <= 1.0f + 1e-4f);
            ASSERT(ras.bc.x + ras.bc.y + ras.bc.z <= 1.0f + 1e-4f);
            float depth = tri[0].z * ras.bc[0] + tri[1].z * ras.bc[1] + tri[2].z * ras.bc[2];
            if (depthFunc == DepthFunc::eGreater) {
                if (depth > framebuffer[offset]) return false;
                if (depth > framebuffer[offset - 1]) return false;
                if (depth > framebuffer[offset - width]) return false;
                if (depth > framebuffer[offset - 1 - width]) return false;
            } else {
                if (depth < framebuffer[offset]) return false;
                if (depth < framebuffer[offset - 1]) return false;
                if (depth < framebuffer[offset - width]) return false;
                if (depth < framebuffer[offset - 1 - width]) return false;
            }
            ras.nextPixelBC();
        }
        ras.nextRowBC();
    }

    return true;
}

bool VHiZ::queryBoundary(const Vector3 &start, const Vector3 &end) const
{
    // Basically 2d ray-marching...
    Raymarcher rm;
    rm.init((Vector2)start, (Vector2)end);

    while (rm.t < 1.0f) {
        float fx = std::floor(rm.p.x);
        float fy = std::floor(rm.p.y);
        bool flag[4];
        flag[0] = (rm.p.x < (float)width) && (rm.p.y < (float)height);
        flag[1] = (rm.p.x == fx) && (rm.p.x > 0.0f) && (rm.p.y < (float)height);
        flag[2] = (rm.p.y == fy) && (rm.p.y > 0.0f) && (rm.p.x < (float)width);
        flag[3] = (rm.p.x == fx) && (rm.p.x > 0.0f) && (rm.p.y == fy) && (rm.p.y > 0.0f);
        int offsets[4];
        offsets[0] = (int)fy * width + (int)fx;
        offsets[1] = offsets[0] - 1;
        offsets[2] = offsets[0] - width;
        offsets[3] = offsets[0] - 1 - width;

        float depth = lerp(start.z, end.z, rm.t);
        if (depthFunc == DepthFunc::eGreater) {
            for (int i = 0; i < 4; ++i) {
                if (flag[i] && depth > framebuffer[offsets[i]]) return false;
            }
        } else {
            for (int i = 0; i < 4; ++i) {
                if (flag[i] && depth < framebuffer[offsets[i]]) return false;
            }
        }

        rm.step();
    }

    return true;
}

void VHiZ::exportTo(const char *outputPath) const
{
    std::ofstream file(outputPath, std::ios::binary);
    file.write((char *)&width, sizeof(float));
    file.write((char *)&height, sizeof(float));
    size_t size = framebuffer.size() * sizeof(float);
    file.write((char *)framebuffer.data(), size);
    file.close();
}

}