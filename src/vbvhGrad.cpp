#include "vbvhGrad.h"
#include "polygonGrad.h"
#include "spatialgrid.h"
#include "util.h"
#include <unordered_set>

namespace vtrz
{

void VBVHGrad::reset(const AABB2 &bound, DepthFunc newDepthFunc)
{
    // Can I avoid this...
    std::vector<uint32_t> allocated = nodePool.allocatedList();
    for (uint32_t i = 0; i < (uint32_t)allocated.size(); ++i) {
        nodePool[allocated[i]].free();
    }
    nodePool = PoolArray<NodeType>();
    leafRefs.clear();
    vertexBuffer.clear();
    indexBuffer.clear();
    rootIndex = nodePool.append();
    NodeType &root = nodePool[rootIndex];
    const DVector2I64 viewport[4] = {
        cast(DVector2(bound.min.x, bound.min.y)),
        cast(DVector2(bound.max.x, bound.min.y)),
        cast(DVector2(bound.max.x, bound.max.y)),
        cast(DVector2(bound.min.x, bound.max.y))
    };
    // TODO: camera gradient?
    root.initLeaf(bound, kEmptyParentNodeIndex,
        viewport, countOf(viewport), kBackgroundTriIndex);

    depthFunc = newDepthFunc;
}

void VBVHGrad::reset(const DVector2 *rootShape, uint32_t vertCount, DepthFunc newDepthFunc)
{
    // Can I avoid this...
    std::vector<uint32_t> allocated = nodePool.allocatedList();
    for (uint32_t i = 0; i < (uint32_t)allocated.size(); ++i) {
        nodePool[allocated[i]].free();
    }
    nodePool = PoolArray<NodeType>();
    leafRefs.clear();
    vertexBuffer.clear();
    indexBuffer.clear();
    rootIndex = nodePool.append();
    NodeType &root = nodePool[rootIndex];
    AABB2 bound;

    BlockAllocator alloc;
    DynamicArray<TPoint2I> shape(&alloc);
    for (uint32_t i = 0; i < vertCount; ++i) {
        bound.expand(getVal(rootShape[i]));
        shape.push_back(cast(rootShape[i]));
    }
    ConvexHull<TPoint2I> ch(&alloc);
    ch.build(shape);

    ASSERT(convexArea2(shape.data(), (uint32_t)shape.size()) > 0);
    root.initLeaf(bound, kEmptyParentNodeIndex,
        shape.data(), (uint32_t)shape.size(), kBackgroundTriIndex);

    depthFunc = newDepthFunc;
}

void VBVHGrad::build(
    const DVector3 *vtxBuffer, uint32_t vertCount,
    const uint32_t *idxBuffer, uint32_t idxCount,
    bool enableMerge)
{
    ASSERT(idxCount % 3 == 0);
    vertexBuffer.insert(vertexBuffer.end(), vtxBuffer, vtxBuffer + vertCount);
    if (idxBuffer) {
        indexBuffer.insert(indexBuffer.end(), idxBuffer, idxBuffer + idxCount);
    }

    ConvexIntersectorGrad conv;
    TraverseContext ctx;

    uint32_t triCount = getTriCount();
    for (uint32_t i = 0; i < triCount; ++i) {
        uint32_t ip = permute(i, triCount);
        ctx.reset(ip);
        traverse(ctx, conv, TraversalType::eGeneral);
        if (enableMerge) {
            merge(ctx);
        }
    }

    buildLeafRefs();
}

void VBVHGrad::traverse(TraverseContext &ctx, ConvexIntersectorGrad &conv, TraversalType type)
{
    if (rootIndex == kEmptyParentNodeIndex) {
        // Don't do anything if the hierarachy is already empty (happens when buildOcclusion()).
        return;
    }

    Tri2Grad triGrad = getTri2d(ctx.triIndex);
    Tri2 tri = getVal(triGrad);
    TriBoundHelper help(tri);
    DVector2I64 triI64[3];
    for (uint32_t i = 0; i < 3; ++i) {
        triI64[i].x = vtrz::cast(tri[i].x);
        triI64[i].y = vtrz::cast(tri[i].y);

        triI64[i].d.x.value = tri[i].x;
        triI64[i].d.x.grad = triGrad[i].x.grad;

        triI64[i].d.y.value = tri[i].y;
        triI64[i].d.y.grad = triGrad[i].y.grad;
    }
    if (cross(getVal(triI64[1]) - getVal(triI64[0]),  getVal(triI64[2]) - getVal(triI64[1])) <= 0) {
        return;
    }

    NodeIndex curr = rootIndex;
    uint8_t height = 0;
    while (true) {
        NodeType &node = nodePool[curr];
        char test = triBoundIsect(tri, help, node.bound);
        if (test == '0') {
            if (!ctx.stack.pop(curr, &height)) break;
        } else if (test == 'c') {
            // TODO: still need simplified logic to color/split/merge tree.
            traverseNoTest(curr, height, ctx.triIndex, ctx);
            if (!ctx.stack.pop(curr, &height)) break;
        } else {
            ASSERT(test == '1');
            if (node.isLeaf()) {
                conv.reset(node.leaf.vertCount, 3, node.leaf.geom, triI64);
                if (conv.intersect()) {
                    Vector2 ref;
                    {
                        Vector2I64 center(0);
                        for (uint32_t i = 0; i < (uint32_t)conv.isect.size(); ++i) {
                            center += getVal(conv.isect[i]);
                        }
                        ref = cast(center) / (float)conv.isect.size();
                    }
                    if (depthOrder(ctx.triIndex, node.triIndex(), ref)) {
                        if (conv.diffChainsP.empty()) {
                            // Node completely occluded by tri. Simply change "color" and add to merge list.
                            node.triIndex() = ctx.triIndex;
                            ctx.mergeList.push_back({ curr , height });
                        } else {
                            uint32_t nodeTriIndex = node.triIndex();
                            // Split old node into pieces.
                            constexpr uint32_t maxLeafCount = 4;
                            NodeIndex newLeafs[maxLeafCount];
                            uint32_t leafCount = 0;

                            conv.splitToConvex();

                            for (uint32_t i = 0; i < (uint32_t)conv.splitPolys.size(); ++i) {
                                uint32_t splitNodeIndex = nodePool.append();
                                NodeType &splitNode = nodePool[splitNodeIndex];
                                AABB2 newBound = computeNewBound(conv.splitPolys[i].data(), (uint32_t)conv.splitPolys[i].size());
                                splitNode.initLeaf(newBound, curr, conv.splitPolys[i].data(), (uint32_t)conv.splitPolys[i].size(), nodeTriIndex);
                                newLeafs[leafCount++] = splitNodeIndex;
                            }

                            if (leafCount == 0) {
                                // Node completely occluded by tri. Simply change "color" and add to merge list.
                                node.triIndex() = ctx.triIndex;
                                ctx.mergeList.push_back({ curr , height });
                            } else {
                                if (type == TraversalType::eGeneral) {
                                    // Create a new node for this piece of tri and add to merge list.
                                    uint32_t newTriNodeIndex = nodePool.append();
                                    NodeType &newTriNode = nodePool[newTriNodeIndex];
                                    AABB2 newBound = computeNewBound(conv.isect.data(), (uint32_t)conv.isect.size());
                                    newTriNode.initLeaf(newBound, curr, conv.isect.data(), (uint32_t)conv.isect.size(), ctx.triIndex);
                                    newLeafs[leafCount++] = newTriNodeIndex;
                                }
                                if (leafCount == 1) {
                                    nodePool[curr].free();
                                    nodePool[curr].bound = nodePool[newLeafs[0]].bound;
                                    nodePool[curr].leaf.geom = nodePool[newLeafs[0]].leaf.geom;
                                    nodePool[curr].leaf.vertCount = nodePool[newLeafs[0]].leaf.vertCount;
                                    nodePool.remove(newLeafs[0]);
                                } else {
                                    // Grow the old leaf into a subtree.
                                    // The reference may become invalid due to appending!
                                    // node.free();
                                    // node.inner.flag = 0;
                                    nodePool[curr].free();
                                    nodePool[curr].inner.flag = 0;
                                    grow(curr, newLeafs, leafCount, height, ctx);
                                }
                            }
                        }
                    } else {
                        ctx.foreground = false;
                        // If node \cap tri == tri, then tri is completely covered by node, no need to continue traverse.
                        if (conv.isect.size() == 3) {
                            bool fullyOccluded = false;
                            for (uint32_t i = 0; i < 3; ++i) {
                                if (conv.isect[i] == triI64[0] &&
                                    conv.isect[(i + 1) % 3] == triI64[1] &&
                                    conv.isect[(i + 2) % 3] == triI64[2]) {
                                    fullyOccluded = true;
                                    break;
                                }
                            }
                            if (fullyOccluded) {
                                break;
                            }
                        }
                    }
                }
                if (!ctx.stack.pop(curr, &height)) break;
            } else {
                ++height;
                ctx.stack.push(node.inner.children[1], &height);
                curr = node.inner.children[0];
            }
        }
    }
}

void VBVHGrad::merge(TraverseContext &ctx)
{
    // TODO: Need a general merging algorithm.
    if (!ctx.foreground) {
        return;
    }
    if (ctx.mergeList.size() <= 1) {
        return;
    }
    // TODO: smarter heuristic?
    uint32_t dest = 0;
    for (uint32_t i = 1; i < (uint32_t)ctx.mergeList.size(); ++i) {
        if (ctx.mergeList[i].height < ctx.mergeList[dest].height) {
            dest = i;
        }
    }
    NodeType &destNode = nodePool[ctx.mergeList[dest].index];
    uint32_t triIndex = destNode.triIndex();
    destNode.free();

    Tri2Grad triGrad = getTri2d(triIndex);
    Tri2 tri = getVal(triGrad);
    DVector2I64 triI64[3];
    for (uint32_t i = 0; i < 3; ++i) {
        triI64[i].x = vtrz::cast(tri[i].x);
        triI64[i].y = vtrz::cast(tri[i].y);

        triI64[i].d.x.value = tri[i].x;
        triI64[i].d.x.grad = triGrad[i].x.grad;

        triI64[i].d.y.value = tri[i].y;
        triI64[i].d.y.grad = triGrad[i].y.grad;
    }
    AABB2 destBound;
    for (uint32_t i = 0; i < 3; ++i) {
        destBound.expand(tri[i]);
    }
    destNode.initLeaf(destBound, destNode.leaf.parent, triI64, 3, triIndex);
    {
        NodeIndex curr = ctx.mergeList[dest].index;
        NodeIndex parent = destNode.leaf.parent;
        while (parent != kEmptyParentNodeIndex) {
            NodeType &parentNode = nodePool[parent];
            NodeIndex sibling = curr == parentNode.inner.children[0] ?
                parentNode.inner.children[1] :
                parentNode.inner.children[0];
            parentNode.bound = join(nodePool[curr].bound, nodePool[sibling].bound);
            curr = parent;
            parent = nodePool[parent].inner.parent;
        }
    }

    std::swap(ctx.mergeList[dest], ctx.mergeList[0]);
    for (uint32_t i = 1; i < (uint32_t)ctx.mergeList.size(); ++i) {
        removeLeaf(ctx.mergeList[i].index);
    }
}

dfloat VBVHGrad::triangleCoverageQuery(const Tri3Grad &queryTri3,
    uint16_t refMeshId, uint32_t refPrimId,
    const uint16_t *meshIds, const uint32_t *primIds,
    const DVector3 *worldPositions, const dfloat *wInvs,
    const Matrix4x4 &queryTransform) const
{
    Tri2Grad queryTri = { (TPoint2F)queryTri3[0], (TPoint2F)queryTri3[1], (TPoint2F)queryTri3[2] };
    Tri2 queryTriVal = getVal(queryTri);
    TriBoundHelper help(queryTriVal);
    DVector2I64 queryTriI64[3] = { cast(queryTri[0]), cast(queryTri[1]), cast(queryTri[2]) };
    if (cross(getVal(queryTriI64[1]) - getVal(queryTriI64[0]), getVal(queryTriI64[2]) - getVal(queryTriI64[1])) <= 0) {
        return 0.0f;
    }

    ConvexIntersectorGrad conv;
    TraverseStack stack;

    dfloat coverage(0.0);
    NodeIndex curr = rootIndex;
    while (true) {
        const NodeType &node = nodePool[curr];
        char test = triBoundIsect(queryTriVal, help, node.bound);
        if (test == '0') {
            if (!stack.pop(curr)) break;
        } else {
            ASSERT(test == 'c' || test == '1');
            if (node.isLeaf()) {
                uint32_t nodeTriIndex = node.triIndex();
                if (nodeTriIndex != kBackgroundTriIndex &&
                    meshIds[nodeTriIndex] == refMeshId &&
                    primIds[nodeTriIndex] == refPrimId) {
                    conv.reset(node.leaf.vertCount, 3, node.leaf.geom, queryTriI64);
                    if (conv.intersect()) {
                        Tri2Grad nodeTri2 = getTri2d(nodeTriIndex);
                        DVector2 v0, vlast;
                        for (uint32_t i = 0; i < (uint32_t)conv.isect.size(); ++i) {
                            DVector2 xy = cast(conv.isect[i]);
                            DVector3 coord = barycentricCoordSafe(xy, nodeTri2);
                            coord.x *= wInvs[3 * nodeTriIndex];
                            coord.y *= wInvs[3 * nodeTriIndex + 1];
                            coord.z *= wInvs[3 * nodeTriIndex + 2];
                            coord /= (coord.x + coord.y + coord.z);

                            DVector3 wp =
                                coord.x * worldPositions[3 * nodeTriIndex] +
                                coord.y * worldPositions[3 * nodeTriIndex + 1] +
                                coord.z * worldPositions[3 * nodeTriIndex + 2];

                            DVector4 h = queryTransform * DVector4(wp, 1.0f);
                            h /= h.w;
                            DVector2 v(h.x, h.y);
                            if (i == 0) {
                                v0 = vlast = v;
                            } else {
                                coverage += cross(vlast, v);
                                vlast = v;
                            }
                        }
                        coverage += cross(vlast, v0);
                    }
                }
                if (!stack.pop(curr)) break;
            } else {
                stack.push(node.inner.children[1]);
                curr = node.inner.children[0];
            }
        }
    }

    // TODO: invBoundArea here should be given as a parameter?
    float invBoundArea = 1.0f / nodePool[rootIndex].bound.area();
    return saturateCoverage(coverage * 0.5f * invBoundArea);
}

void VBVHGrad::buildOcclusion(
    const DVector3 *vtxBuffer, uint32_t vertCount,
    const uint32_t *idxBuffer, uint32_t idxCount,
    const uint32_t *occludeeList, uint32_t occludeeCount)
{
    ASSERT(idxCount % 3 == 0);
    vertexBuffer.insert(vertexBuffer.end(), vtxBuffer, vtxBuffer + vertCount);
    if (idxBuffer) {
        indexBuffer.insert(indexBuffer.end(), idxBuffer, idxBuffer + idxCount);
    }

    ConvexIntersectorGrad conv;
    TraverseContext ctx;

    uint32_t triCount = getTriCount();
    std::unordered_set<uint32_t> occludeeSet;
    for (uint32_t i = 0; i < occludeeCount; ++i) {
        occludeeSet.insert(occludeeList[i]);
    }

    struct Occluder
    {
        uint32_t triIdx;
        float area;
    };
    std::vector<Occluder> occluderList;
    for (uint32_t i = 0; i < triCount; ++i) {
        if (!occludeeSet.count(i)) {
            auto tri = getTri2d(i);
            float area = cross(tri[1].value() - tri[0].value(), tri[2].value() - tri[1].value());
            occluderList.push_back({ i, area });
        }
    }
    std::sort(occluderList.begin(), occluderList.end(), [](const Occluder &occ1, const Occluder &occ2) {
        return occ1.area > occ2.area;
    });

    for (uint32_t i = 0; i < occludeeCount; ++i) {
        uint32_t ip = permute(i, occludeeCount);
        ctx.reset(occludeeList[ip]);
        traverse(ctx, conv, TraversalType::eGeneral);
        merge(ctx);
    }

    removeBackground();

    for (uint32_t i = 0; i < (uint32_t)occluderList.size(); ++i) {
        if (rootIndex == kEmptyParentNodeIndex) break;
        //uint32_t ip = permute(i, (uint32_t)occluderList.size());
        //ctx.reset(occluderList[ip].triIdx);
        ctx.reset(occluderList[i].triIdx);
        traverse(ctx, conv, TraversalType::eOcclusionOnly);
        deleteOccluded(ctx);
    }

    buildLeafRefs();
}

void VBVHGrad::simplify(const uint16_t *meshIds, const uint32_t *primIds)
{
    uint32_t leafCount = (uint32_t)leafRefs.size();
    if (leafCount == 0) {
        return;
    }
    uint16_t meshId = 0;
    uint32_t primId = 0;
    uint32_t repTriId = 0;
    for (uint32_t i = 0; i < leafCount; ++i) {
        NodeType &node = nodePool[leafRefs[i]];
        uint32_t triId = node.triIndex();
        if (i > 0 && (meshId != meshIds[triId] || primId != primIds[triId])) {
            return;
        }
        meshId = meshIds[triId];
        primId = primIds[triId];
        repTriId = triId;
    }

    BlockAllocator alloc;
    DynamicArray<TPoint2I> points(&alloc);
    for (uint32_t i = 0; i < leafCount; ++i) {
        NodeType &node = nodePool[leafRefs[i]];
        for (uint32_t v = 0; v < node.leaf.vertCount; ++v) {
            points.push_back(node.leaf.geom[v]);
        }
    }
    ConvexHull<TPoint2I> ch(&alloc);
    ch.build(points);
    AABB2 bound;
    for (const auto &p : points) {
        bound.expand(getVal(cast(p)));
    }

    // Can I avoid this...
    std::vector<uint32_t> allocated = nodePool.allocatedList();
    for (uint32_t i = 0; i < (uint32_t)allocated.size(); ++i) {
        nodePool[allocated[i]].free();
    }
    nodePool = PoolArray<NodeType>();
    leafRefs.clear();

    rootIndex = nodePool.append();
    NodeType &root = nodePool[rootIndex];
    root.initLeaf(bound, kEmptyParentNodeIndex, points.data(), (uint32_t)points.size(), repTriId);
    leafRefs.push_back(rootIndex);
}

}