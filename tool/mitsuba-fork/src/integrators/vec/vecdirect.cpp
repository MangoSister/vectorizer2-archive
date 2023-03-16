// #include <mitsuba/render/scene.h>
// #include <mitsuba/render/derivative.h>
// #include <Vectorizer/Vectorizer.h>
// #include <cstdio>
// #include <fstream>
// #include <unordered_map>
// #include <unordered_set>
// #include <boost/filesystem/path.hpp>

// MTS_NAMESPACE_BEGIN

// using vectorizer::Vectorizer;
// using vectorizer::vec2;
// using vectorizer::vec3;
// using vectorizer::vec3f;
// using vectorizer::vec4f;
// using vectorizer::mat4f;

// // Function to combine two hashes (from boost::hash_combine)
// // The magic number is the reciprocal of the Golden Ratio represented as
// // a 64-bit fixed number, i.e. with G = (1+sqrt(5))/2, magic = (1/G * 2^64) (mod 2^64)
// constexpr std::size_t hashCombine(std::size_t lhs, std::size_t rhs)
// {
//     lhs ^= rhs + UINT64_C(0x9e3779b97f4a7c17) + (lhs << 6) + (lhs >> 2); // For 32-bit size_t, use 0x9e3779b9
//     return lhs;
// }

// // Function to bit-mix 64-bit numbers, for instance as a last step in combining
// // the result of other hashes.
// constexpr uint64_t bitMix64(uint64_t x)
// {
//     // Constants taken from (mix 13) http://zimbry.blogspot.com/2011/09/better-bit-mixing-improving-on.html
//     x = (x ^ (x >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
//     x = (x ^ (x >> 27)) * UINT64_C(0x94d049bb133111eb);
//     x = x ^ (x >> 31);
//     return x;
// }

// struct PointHash
// {
//     size_t operator()(Point point) const {
//         size_t hx = std::hash<float>()(point.x);
//         size_t hy = std::hash<float>()(point.y);
//         size_t hz = std::hash<float>()(point.z);
//         return bitMix64(hashCombine(hashCombine(hx, hy), hz));
//     }
// };
// struct PointEqual
// {
//     bool operator()(Point pt1, Point pt2) const {
//         return pt1.x == pt2.x && pt1.y == pt2.y && pt1.z == pt2.z;
//     }
// };

// using VertexCache = std::unordered_map<Point, uint32_t, PointHash, PointEqual>;

// struct ShadingInfo
// {
//     std::vector<Normal> vertexNormals;
//     std::vector<Point2> vertexTexcoords;
//     std::vector<Float> vertexInvWs; // For perspective-correct interpolation.
//     std::vector<uint32_t> objectIds;

//     void clear() {
//         vertexNormals.clear();
//         vertexTexcoords.clear();
//         vertexInvWs.clear();
//         objectIds.clear();
//     }
// };

// static inline RayDifferential createRayDiff(const PerspectiveCamera &persp, Point2i pixel, uint32_t sampleCount)
// {
//     RayDifferential rayDiff;
//     persp.sampleRayDifferential(rayDiff, Point2(pixel) + Point2(0.5, 0.5), Point2(0.0), Float(0.0));
//     Float diffScaleFactor = 1.0f / std::sqrt((Float)sampleCount);
//     rayDiff.scaleDifferential(diffScaleFactor);
//     return rayDiff;
// }

// //////////////////////////////////////////////////////////////////////////
// // rainbow-like gradient:
// // red: 0 -> yellow: 0.5 -> blue: 1.0
// inline static Vector colorGradient(Float t)
// {
//     t = math::clamp(t, Float(0.0), Float(1.0));

//     constexpr int s = 7;
//     Vector3 p[s];
//     p[1] = Vector(238, 64, 53) / 255.0;
//     p[2] = Vector(243, 119, 54) / 255.0;
//     p[3] = Vector(253, 244, 152) / 255.0;
//     p[4] = Vector(123, 192, 67) / 255.0;
//     p[5] = Vector(3, 146, 207) / 255.0;

//     p[0] = p[s - 2];
//     p[s - 1] = p[1];

//     // map t (0.0 to 1.0) to distance down the spline
//     Float m = Float(s) - Float(2.0);
//     Float d = Float(1.0) + (m - Float(1.0)) * t;
//     // get the base index and t value
//     int b = int(d);
//     Float dt = d - floor(d);

//     return (
//         (p[b] * 2.0) +
//         (-p[b - 1] + p[b + 1]) * dt +
//         (p[b - 1] * 2.0 - p[b]* 5.0 + p[b + 1] * 4.0 - p[b + 2]) * dt * dt +
//         (-p[b - 1] + p[b] * 3.0 - p[b + 1] * 3.0 + p[b + 2]) * dt * dt * dt) * 0.5;
// }

// struct DiffRenderMode
// {
//     using float_type = DFloat;
//     using vector2_type = DVector2;
//     using vector_type = DVector;
//     using vector4_type = DVector4;
//     using spectrum_type = DSpectrum;
//     using matrix3x3_type = DMatrix3x3;
//     using matrix4x4_type = DMatrix4x4;
//     using transform_type = DTransform;

//     DiffRenderMode(const Transform &primaryInvVpTrans,
//         uint32_t diffObjectId,
//         const Vector gradWorldPos) :
//         primaryInvVpTrans(primaryInvVpTrans),
//         diffObjectId(diffObjectId),
//         gradWorldPos(gradWorldPos) {}

//     inline vectorizer::SampleArray samplePrimaryVect(const Vectorizer &primaryVect, const std::vector<uint32_t> &objectIds) const {
//         mat4f vpMatrix;
//         mat4f invVpMatrix;
//         for (uint32_t row = 0; row < 4; ++row) {
//             for (uint32_t col = 0; col < 4; ++col) {
//                 vpMatrix[col][row] = primaryInvVpTrans.getInverseMatrix().m[row][col];
//                 invVpMatrix[col][row] = primaryInvVpTrans.getMatrix().m[row][col];
//             }
//         }
//         return primaryVect.sampleMultiGrad(objectIds.data(), (uint32_t)objectIds.size(),
//             vpMatrix, invVpMatrix, diffObjectId, vec3f(gradWorldPos.x, gradWorldPos.y, gradWorldPos.z));
//     }

//     inline float_type getSampleCoverage(const vectorizer::SampleArray &sampleArray, uint32_t index) const {
//         return float_type(sampleArray.entries[index].coverage, sampleArray.entries[index].gradCoverage);
//     }

//     inline void vectQueryTriangle(const vectorizer::Vectorizer &vect,
//         const vec3 projTri[3], vectorizer::TriangleQueryInterface &query) {
//         vect.queryTriangle(projTri, query, diffObjectId);
//     }

//     const Transform &primaryInvVpTrans;
//     uint32_t diffObjectId;
//     Vector gradWorldPos;
// };

// struct RenderMode
// {
//     using float_type = Float;
//     using vector2_type = Vector2;
//     using vector_type = Vector;
//     using vector4_type = Vector4;
//     using spectrum_type = Spectrum;
//     using matrix3x3_type = Matrix3x3;
//     using matrix4x4_type = Matrix4x4;
//     using transform_type = Transform;

//     inline vectorizer::SampleArray samplePrimaryVect(const Vectorizer &primaryVect, const std::vector<uint32_t> &objectIds) const {
//         return primaryVect.sampleMulti(objectIds.data(), (uint32_t)objectIds.size());
//     }

//     inline float_type getSampleCoverage(const vectorizer::SampleArray &sampleArray, uint32_t index) const {
//         return float_type(sampleArray.entries[index].coverage);
//     }

//     inline void vectQueryTriangle(const vectorizer::Vectorizer &vect,
//         const vec3 projTri[3], vectorizer::TriangleQueryInterface &query) {
//         vect.queryTriangle(projTri, query);
//     }
// };

// class VectDirectIntegrator final : public SamplingIntegrator {
// public:
//     MTS_DECLARE_CLASS()

// public:
//     /// Initialize the integrator with the specified properties
//     VectDirectIntegrator(const Properties &props);

//     /// Unserialize from a binary data stream
//     VectDirectIntegrator(Stream *stream, InstanceManager *manager);

//     /// Serialize to a binary data stream
//     void serialize(Stream *stream, InstanceManager *manager) const final;

//     bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
//         int sceneResID, int sensorResID, int samplerResID) final;

//     void renderBlock(const Scene *scene, const Sensor *sensor,
//         Sampler *sampler, ImageBlock *block, const bool &stop,
//         const std::vector< TPoint2<uint8_t> > &points) const final;

//     void prepareVectorizer(
//         const Scene *scene, const Transform &viewTrans, const Transform &projTrans, Float nearClip,
//         const vectorizer::AABB2 vectBound, const std::vector<FrustumIsectEntry> &isects, const TriMesh *meshMask,
//         VertexCache &vertexCache, std::vector<vec3f> &vertexBuffer, std::vector<uint32_t> &indexBuffer,
//         ShadingInfo *shadingInfo = nullptr) const;

//     template<typename TMode,
//         typename = std::enable_if_t<
//         std::is_same<TMode, RenderMode>::value ||
//         std::is_same<TMode, DiffRenderMode>::value>>
//     typename TMode::spectrum_type evalDirectLighting(const Vectorizer &primaryVect,
//         const std::vector<vec3f> &primaryVectVb, const std::vector<uint32_t> &primaryVectIb,
//         const ShadingInfo &primaryShadingInfo, const Transform &invVpTrans, const Point &sensorPos,
//         const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel, TMode &mode) const;

//     template<typename TMode,
//         typename = std::enable_if_t<
//         std::is_same<TMode, RenderMode>::value ||
//         std::is_same<TMode, DiffRenderMode>::value>>
//     typename TMode::spectrum_type evalDirectLightingSample(const vectorizer::SampleEntry &entry,
//         const Intersection &its, const Point &sensorPos, const Scene &scene, const Emitter &emitter,
//         TMode &mode) const;

//     Intersection buildItsFromSample(const vectorizer::SampleEntry &entry,
//         const std::vector<vec3f> &primaryVectVb, const std::vector<uint32_t> &primaryVectIb,
//         const ShadingInfo &shadingInfo, const Transform &invVpTrans, const Point &sensorPos,
//         const Scene &scene) const;

//     struct VectDirectLightingSampleRecord
//     {
//         vectorizer::Vectorizer vect;
//         std::vector<vec3f> vectVertexBuffer;
//         std::vector<uint32_t> vectIndexBuffer;
//         Transform shadingProjTrans;
//         Transform shadingViewTrans;
//     };

//     bool vectorizeDirectLightingSample(const vectorizer::SampleEntry &entry,
//         const Intersection &its, const Point &sensorPos, const Scene &scene, const Emitter &emitter,
//         VectDirectLightingSampleRecord &record) const;

//     Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const final;

//     Spectrum E(const Scene *scene, const Intersection &its,
//         const Medium *medium, Sampler *sampler, int nSamples,
//         bool includeIndirect) const final;

//     void bindUsedResources(ParallelProcess *proc) const final;

//     void wakeup(ConfigurableObject *parent, std::map<std::string, SerializableObject *> &params) final;

// private:
//     // This is to filter out vertices with duplicate positions.
//     // Duplicate vertices are sometimes necessary for certain UV wrapping.
//     using IndexBufferFilter = std::vector<uint32_t>;
//     std::vector<IndexBufferFilter> m_indexBufferFilters;

//     bool m_diffRender = false;
//     std::string m_diffRenderObject;
//     std::string m_diffRenderParameter;
//     uint32_t m_diffObjectId = (uint32_t)~0;
//     Float m_diffRenderParamDelta = 0.0;

//     enum class EVectDirectDiffRenderParam
//     {
//         EConstant,
//         //
//         ETranslateX,
//         ETranslateY,
//         ETranslateZ,
//     };
//     EVectDirectDiffRenderParam m_diffRenderParam;
// };

// //////////////////////////////////////////////////////////////////////////
// VectDirectIntegrator::VectDirectIntegrator(const Properties &props) :
//     SamplingIntegrator(props)
// {
//     m_diffRender = props.getBoolean("diffRender", false);
//     m_diffRenderObject = props.getString("diffRenderObject", std::string());
//     m_diffRenderParameter = props.getString("diffRenderParameter", std::string());
//     m_diffRenderParamDelta = props.getFloat("diffRenderParamDelta", Float(0.0));
// }

// VectDirectIntegrator::VectDirectIntegrator(Stream *stream, InstanceManager *manager)
//     : SamplingIntegrator(stream, manager)
// {
//     m_diffRender = stream->readBool();
//     m_diffRenderObject = stream->readString();
//     m_diffRenderParameter = stream->readString();
//     m_diffRenderParamDelta = stream->readFloat();
// }

// void VectDirectIntegrator::serialize(Stream *stream, InstanceManager *manager) const
// {
//     SamplingIntegrator::serialize(stream, manager);

//     stream->writeBool(m_diffRender);
//     stream->writeString(m_diffRenderObject);
//     stream->writeString(m_diffRenderParameter);
//     stream->writeFloat(m_diffRenderParamDelta);
// }

// bool VectDirectIntegrator::preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
//     int sceneResID, int sensorResID, int samplerResID)
// {
//     SamplingIntegrator::preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);

//     uint32_t meshCount = (uint32_t)scene->getMeshes().size();
//     m_indexBufferFilters.resize(meshCount);
//     for (uint32_t m = 0; m < meshCount; ++m) {
//         VertexCache cache;
//         const TriMesh *mesh = scene->getMeshes()[m];
//         IndexBufferFilter &cib = m_indexBufferFilters[m];
//         uint32_t vertexCount = (uint32_t)mesh->getVertexCount();
//         cib.resize(vertexCount);
//         for (uint32_t v = 0; v < vertexCount; ++v) {
//             Point pos = mesh->getVertexPositions()[v];
//             auto it = cache.find(pos);
//             if (it != cache.end()) {
//                 Assert(it->second < v);
//                 cib[v] = it->second;
//             } else {
//                 cache.insert({ pos, v });
//                 cib[v] = v;
//             }
//         }
//     }

//     // Hack.
//     if (m_diffRender) {
//         m_diffObjectId = (uint32_t)~0;
//         if (m_diffRenderParameter == "translateX") {
//             m_diffRenderParam = EVectDirectDiffRenderParam::ETranslateX;
//         } else if (m_diffRenderParameter == "translateY") {
//             m_diffRenderParam = EVectDirectDiffRenderParam::ETranslateY;
//         } else if (m_diffRenderParameter == "translateZ") {
//             m_diffRenderParam = EVectDirectDiffRenderParam::ETranslateZ;
//         } else {
//             m_diffRenderParam = EVectDirectDiffRenderParam::EConstant;
//         }
//         if (m_diffRenderParam != EVectDirectDiffRenderParam::EConstant) {
//             const std::vector<TriMesh *> &meshes = scene->getMeshes();
//             for (uint32_t objectId = 0; objectId < (uint32_t)meshes.size(); ++objectId) {
//                 // TODO: getID() not working for shapes/meshes?
//                 if (m_diffRenderObject == meshes[objectId]->getName()) {
//                     m_diffObjectId = objectId;
//                     break;
//                 }
//             }
//         } else {
//             ref_vector<ConfigurableObject> &objects = const_cast<Scene *>(scene)->getReferencedObjects();
//             for (auto &object : objects) {
//                 if (m_diffRenderObject == object->getID()) {
//                     const Class *objectClass = object->getClass();
//                     if (objectClass->derivesFrom(MTS_CLASS(BSDF))) {
//                         (static_cast<BSDF &>(*object)).configureDiffRender(m_diffRenderParameter, m_diffRenderParamDelta);
//                     } else if (objectClass->derivesFrom(MTS_CLASS(Emitter))) {
//                         (static_cast<Emitter &>(*object)).configureDiffRender(m_diffRenderParameter, m_diffRenderParamDelta);
//                     }
//                     break;
//                 }
//             }
//         }
//     }

//     // Technically only need to call this once.
//     vectorizer::initGlobalConstants();

//     return true;
// }

// /// Linearly interpolate between two values
// template <typename T, typename U>
// constexpr U lerp(T t, U v1, U v2) {
//     return ((T)1 - t) * v1 + t * v2;
// }

// static uint8_t clipTriangle(Float near, Point v[4], Normal vn[4], Point2 tc[4]) {
//     // near > 0, but -z should be forward (gl convention).

//     // Distances to the plane ( this is an array parallel
//     // to v [] , stored as a vec3 )
//     constexpr Float nearEps(1e-4);
//     Vector dist = Vector(v[0].z, v[1].z, v[2].z) + Vector(near + nearEps);
//     bool above[3] = { dist[0] <= 0.0, dist[1] <= 0.0, dist[2] <= 0.0 };
//     if (!above[0] && !above[1] && !above[2]) {
//         // Case 1 ( all clipped )
//         return 0;
//     }
//     if (above[0] && above[1] && above[2]) {
//         // Case 2 ( none clipped )
//         // v3 = v0;
//         return 3;
//     }
//     // There are either 1 or 2 vertices above the clipping plane .
//     bool nextIsAbove;
//     // Find the CCW - most vertex above the plane .
//     if (above[1] && !above[0]) {
//         // Cycle once CCW . Use v3 as a temp
//         nextIsAbove = above[2];
//         // xyz -> yzx
//         v[3] = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3];
//         if (vn) { vn[3] = vn[0]; vn[0] = vn[1]; vn[1] = vn[2]; vn[2] = vn[3]; }
//         if (tc) { tc[3] = tc[0]; tc[0] = tc[1]; tc[1] = tc[2]; tc[2] = tc[3]; }
//         Float temp;
//         temp = dist[0]; dist[0] = dist[1]; dist[1] = dist[2]; dist[2] = temp;
//     } else if (above[2] && !above[1]) {
//         // Cycle once CW . Use v3 as a temp.
//         nextIsAbove = above[0];
//         // xyz -> zxy
//         v[3] = v[2]; v[2] = v[1]; v[1] = v[0]; v[0] = v[3];
//         if (vn) { vn[3] = vn[2]; vn[2] = vn[1]; vn[1] = vn[0]; vn[0] = vn[3]; }
//         if (tc) { tc[3] = tc[2]; tc[2] = tc[1]; tc[1] = tc[0]; tc[0] = tc[3]; }
//         Float temp;
//         temp = dist[2]; dist[2] = dist[1]; dist[1] = dist[0]; dist[0] = temp;
//     } else {
//         nextIsAbove = above[1];
//     }

//     // We always need to clip v2 - v0.
//     Float t = dist[0] / (dist[0] - dist[2]);
//     v[3] = lerp(t, v[0], v[2]);
//     if (vn) vn[3] = normalize(lerp(t, vn[0], vn[2]));
//     if (tc) tc[3] = lerp(t, tc[0], tc[2]);
//     if (nextIsAbove) {
//         // Case 3
//         t = dist[1] / (dist[1] - dist[2]);
//         v[2] = lerp(t, v[1], v[2]);
//         if (vn) vn[2] = normalize(lerp(t, vn[1], vn[2]));
//         if (tc) tc[2] = lerp(t, tc[1], tc[2]);
//         return 4;
//     } else {
//         // Case 4
//         t = dist[0] / (dist[0] - dist[1]);
//         v[1] = lerp(t, v[0], v[1]);
//         v[2] = v[3];
//         if (vn) {
//             vn[1] = normalize(lerp(t, vn[0], vn[1]));
//             vn[2] = vn[3];
//         }
//         if (tc) {
//             tc[1] = lerp(t, tc[0], tc[1]);
//             tc[2] = tc[3];
//         }
//         // v3 = v0;
//         return 3;
//     }
// }

// static inline vec3f toVec3f(const Point &p)
// {
//     return vec3f(p.x, p.y, p.z);
// }

// static inline Point toPoint(const vec3f &v)
// {
//     return Point(v.x, v.y, v.z);
// }

// static void dumpVectorizerInput(
//     const Scene *scene, const Point2i &pixel,
//     const vectorizer::AABB2 &bound,
//     const std::vector<vec3f> &vertexBuffer,
//     const std::vector<uint32_t> &indexBuffer)
// {
//     fs::path sourceStem = scene->getSourceFile().stem();

//     char *name = new char[256];
//     snprintf(name, 256, "%s-%d-%d.data", sourceStem.string().c_str(), pixel.x, pixel.y);
//     std::ofstream file(name, std::ios::binary);
//     SAssert(file);

//     size_t bufSize = sizeof(vectorizer::AABB2) + sizeof(uint32_t) * 2 + sizeof(vec3f) * vertexBuffer.size() + sizeof(uint32_t) * indexBuffer.size();
//     char *buf = new char[bufSize];

//     size_t offset = 0;
//     memcpy(buf + offset, &bound, sizeof(vectorizer::AABB2));
//     offset += sizeof(vectorizer::AABB2);

//     uint32_t vertexCount = (uint32_t)vertexBuffer.size();
//     memcpy(buf + offset, &vertexCount, sizeof(uint32_t));
//     offset += sizeof(uint32_t);

//     memcpy(buf + offset, vertexBuffer.data(), sizeof(vec3f) * vertexCount);
//     offset += sizeof(vec3f) * vertexCount;

//     uint32_t indexCount = (uint32_t)indexBuffer.size();
//     memcpy(buf + offset, &indexCount, sizeof(uint32_t));
//     offset += sizeof(uint32_t);

//     memcpy(buf + offset, indexBuffer.data(), sizeof(uint32_t) * indexCount);
//     offset += sizeof(uint32_t) * indexCount;

//     file.write(buf, bufSize);

//     file.close();

//     delete[] name;
//     delete[] buf;
// }

// void VectDirectIntegrator::renderBlock(const Scene *scene, const Sensor *sensor,
//     Sampler *sampler, ImageBlock *block, const bool &stop,
//     const std::vector< TPoint2<uint8_t> > &points) const
// {
//     block->clear();
//     if (m_diffRender) {
//         // Gradients can have negative values.
//         block->setWarn(false);
//     }
//     Assert(sensor->getClass()->derivesFrom(MTS_CLASS(PerspectiveCamera)));
//     const PerspectiveCamera *persp = static_cast<const PerspectiveCamera *>(sensor);
//     Vector2 resolution = Vector2(sensor->getFilm()->getCropSize());
//     Transform viewTrans = persp->getViewTransform(0.0); // Don't care about motion blur for now.
//     // Flip x and z so that +x points right, and +z points outward.
//     Matrix4x4 flipViewMtx;
//     flipViewMtx.setIdentity();
//     flipViewMtx.m[0][0] = -1.0;
//     flipViewMtx.m[2][2] = -1.0;
//     viewTrans = Transform(flipViewMtx) * viewTrans;
//     Point sensorPos(viewTrans.getInverseMatrix().m[0][3],
//         viewTrans.getInverseMatrix().m[1][3],
//         viewTrans.getInverseMatrix().m[2][3]);
//     Transform projTrans = persp->getProjectionTransform(Point2(0.0), Point2(0.0)); // Don't care about DOF for now.
//     Transform invVpTrans = (projTrans * viewTrans).inverse();
//     Float nearClip = persp->getNearClip();

//     std::vector<FrustumIsectEntry> isects;
//     VertexCache vertexCache;
//     std::vector<vec3f> vertexBuffer;
//     std::vector<uint32_t> indexBuffer;
//     ShadingInfo shadingInfo;
//     Vectorizer vect;

//     for (auto pt : points) {
//         if (stop) {
//             break;
//         }

//         Point2i pixel = Point2i(pt) + Vector2i(block->getOffset());
//         // sampler->generate(pixel); // Need to call this at the start of every pixel, if using the sampler.
//         //////////////////////////////////////////////////////////////////////////
//         // Debug
//         //////////////////////////////////////////////////////////////////////////
//         Frustum frustum = persp->buildFrustum(pixel);
//         vectorizer::AABB2 vectBound;
//         // Why...?
//         vectBound.min.x = (Float(2.0) * (pixel.x + 0.5)) / resolution.x - Float(1.0);
//         vectBound.max.x = (Float(2.0) * ((pixel.x + 0.5) + Float(1.0))) / resolution.x - Float(1.0);
//         vectBound.min.y = -(Float(2.0) * ((pixel.y - 0.5) + Float(1.0))) / resolution.y + Float(1.0);
//         vectBound.max.y = -(Float(2.0) * (pixel.y - 0.5)) / resolution.y + Float(1.0);

//         isects.clear();
//         vertexCache.clear();
//         vertexBuffer.clear();
//         indexBuffer.clear();
//         shadingInfo.clear();
//         vect.reset(vectBound);

//         scene->frustumIntersect(frustum, isects);

//         prepareVectorizer(
//             scene, viewTrans, projTrans, nearClip,
//             vectBound, isects, nullptr,
//             vertexCache, vertexBuffer, indexBuffer,
//             &shadingInfo);

//         //////////////////////////////////////////////////////////////////////////
//         // Debug
//         // dumpVectorizerInput(scene, pixel, vectBound, vertexBuffer, indexBuffer);
//         //////////////////////////////////////////////////////////////////////////
//         vect.process(vertexBuffer.data(), (uint32_t)vertexBuffer.size(),
//             indexBuffer.data(), (uint32_t)indexBuffer.size());
//         // Spectrum L = Spectrum(Float(vect.queryCoverage()));

//         Spectrum pixelResult;

//         if (!m_diffRender) {
//             pixelResult = evalDirectLighting<RenderMode>(
//                 vect, vertexBuffer, indexBuffer,
//                 shadingInfo, invVpTrans, sensorPos,
//                 *scene, *persp, pixel, RenderMode());
//         } else {
//             Vector gradWorldPos;
//             switch (m_diffRenderParam) {
//             case EVectDirectDiffRenderParam::ETranslateX: gradWorldPos = Vector(1.0, 0.0, 0.0); break;
//             case EVectDirectDiffRenderParam::ETranslateY: gradWorldPos = Vector(0.0, 1.0, 0.0); break;
//             case EVectDirectDiffRenderParam::ETranslateZ: gradWorldPos = Vector(0.0, 0.0, 1.0); break;
//             case EVectDirectDiffRenderParam::EConstant: gradWorldPos = Vector(0.0, 0.0, 0.0); break;
//             }
//             DiffRenderMode mode(invVpTrans, m_diffObjectId, gradWorldPos);
//             DSpectrum dL = evalDirectLighting<DiffRenderMode>(
//                 vect, vertexBuffer, indexBuffer,
//                 shadingInfo, invVpTrans, sensorPos,
//                 *scene, *persp, pixel, mode);
//             for (uint32_t dim = 0; dim < SPECTRUM_SAMPLES; ++dim) {
//                 pixelResult[dim] = dL[dim].grad;
//             }
//             // Better visualization?
//             //Float lum = pixelResult.getLuminance();
//             //lum = math::clamp(lum, Float(-5.0), Float(5.0));
//             //lum += Float(5.0);
//             //lum *= Float(0.1);
//             //Vector color = colorGradient(lum);
//             //pixelResult.fromLinearRGB(color.x, color.y, color.z);
//         }

//         // Do I actually need pixel filtering?
//         Point2 pixelCenter(Point2(pixel) + Vector2(0.5, 0.5));
//         // No alpha.
//         block->put(pixelCenter, pixelResult, 1.0);

//         // sampler->advance(); // Need to call this at the end of every pixel, if using the sampler.
//     }
// }

// void VectDirectIntegrator::prepareVectorizer(
//     const Scene *scene, const Transform &viewTrans, const Transform &projTrans, Float nearClip,
//     const vectorizer::AABB2 vectBound, const std::vector<FrustumIsectEntry> &isects, const TriMesh *meshMask,
//     VertexCache &vertexCache, std::vector<vec3f> &vertexBuffer, std::vector<uint32_t> &indexBuffer,
//     ShadingInfo *shadingInfo) const
// {
//     const BVH *bvh = scene->getBVH();
//     const std::vector<Primitive> &primitives = bvh->primitives();

//     for (FrustumIsectEntry entry : isects) {
//         for (uint32_t i = 0; i < entry.primCount; ++i) {
//             const Primitive &prim = primitives[entry.primOffset + i];
//             const TriMesh *mesh = scene->getMeshes()[prim.meshIndex];
//             if (mesh == meshMask) {
//                 continue;
//             }
//             Triangle tri = mesh->getTriangles()[prim.primIndex];
//             Triangle triFiltered;
//             for (uint32_t v = 0; v < 3; ++v) {
//                 triFiltered.idx[v] = m_indexBufferFilters[prim.meshIndex][tri.idx[v]];
//             }
//             Normal vn[4];
//             Point2 tc[4];
//             if (shadingInfo) {
//                 if (mesh->hasVertexNormals()) {
//                     for (uint32_t v = 0; v < 3; ++v) {
//                         vn[v] = mesh->getVertexNormals()[tri.idx[v]];
//                     }
//                 } else {
//                     vn[0] = vn[1] = vn[2] = Normal(normalize(cross(
//                         mesh->getVertexPositions()[tri.idx[1]] - mesh->getVertexPositions()[tri.idx[0]],
//                         mesh->getVertexPositions()[tri.idx[2]] - mesh->getVertexPositions()[tri.idx[1]])));
//                 }
//                 if (mesh->hasVertexTexcoords()) {
//                     for (uint32_t v = 0; v < 3; ++v) {
//                         tc[v] = mesh->getVertexTexcoords()[tri.idx[v]];
//                     }
//                 } else {
//                     tc[0] = tc[1] = tc[2] = Point2(0.0);
//                 }
//             }
//             Point transPositions[4];
//             Float invW[4];
//             for (uint32_t v = 0; v < 3; ++v) {
//                 Point worldPos = mesh->getVertexPositions()[triFiltered.idx[v]];
//                 transPositions[v] = viewTrans(worldPos);
//             }
//             // Only need to clip against near plane. (Maybe do it in clip space?)
//             uint8_t clip = clipTriangle(nearClip, transPositions, shadingInfo ? vn : nullptr, shadingInfo ? tc : nullptr);
//             if (clip == 0) {
//                 continue;
//             } else {
//                 for (uint32_t v = 0; v < 3; ++v) {
//                     Assert(transPositions[v].z <= -nearClip);
//                     invW[v] = Float(1.0) / dot(projTrans.getMatrix().row(3),
//                         Vector4(transPositions[v].x, transPositions[v].y, transPositions[v].z, 1.0));
//                     transPositions[v] = projTrans.transformAffine(transPositions[v]);
//                     transPositions[v] *= invW[v]; // Perspective divide.
//                 }
//                 // Backface culling (only need 3 verts to determine normal anyway.)
//                 if (cross(Vector(transPositions[1]) - Vector(transPositions[0]),
//                     Vector(transPositions[2]) - Vector(transPositions[1])).z <= 0.0) {
//                     continue;
//                 }

//                 //////////////////////////////////////////////////////////////////////////
//                 // Actually also do simple 2d AABB test.
//                 if (clip == 3) {
//                     vectorizer::AABB2 b;
//                     b.expand(vec2(transPositions[0].x, transPositions[0].y));
//                     b.expand(vec2(transPositions[1].x, transPositions[1].y));
//                     b.expand(vec2(transPositions[2].x, transPositions[2].y));
//                     if (vectorizer::intersectExclusive(vectBound, b)) {
//                         for (uint32_t v = 0; v < 3; ++v) {
//                             auto it = vertexCache.find(transPositions[v]);
//                             if (it != vertexCache.end()) {
//                                 indexBuffer.push_back(it->second);
//                             } else {
//                                 indexBuffer.push_back((uint32_t)vertexBuffer.size());
//                                 vertexCache.insert({ transPositions[v], (uint32_t)vertexBuffer.size() });
//                                 vertexBuffer.push_back(toVec3f(transPositions[v]));
//                                 if (shadingInfo) shadingInfo->vertexInvWs.push_back(invW[v]);
//                             }
//                         }
//                         if (shadingInfo) {
//                             shadingInfo->vertexNormals.insert(shadingInfo->vertexNormals.end(), { vn[0], vn[1], vn[2] });
//                             shadingInfo->vertexTexcoords.insert(shadingInfo->vertexTexcoords.end(), { tc[0], tc[1], tc[2] });
//                             shadingInfo->objectIds.push_back(prim.meshIndex);
//                         }
//                     }
//                 } else {
//                     Assert(clip == 4);
//                     Assert(transPositions[3].z <= -nearClip);
//                     invW[3] = Float(1.0) / dot(projTrans.getMatrix().row(3),
//                         Vector4(transPositions[3].x, transPositions[3].y, transPositions[3].z, 1.0));
//                     transPositions[3] = projTrans.transformAffine(transPositions[3]);
//                     transPositions[3] *= invW[3]; // Perspective divide.

//                     vectorizer::AABB2 b0;
//                     b0.expand(vec2(transPositions[0].x, transPositions[0].y));
//                     b0.expand(vec2(transPositions[1].x, transPositions[1].y));
//                     b0.expand(vec2(transPositions[2].x, transPositions[2].y));
//                     vectorizer::AABB2 b1;
//                     b1.expand(vec2(transPositions[0].x, transPositions[0].y));
//                     b1.expand(vec2(transPositions[2].x, transPositions[2].y));
//                     b1.expand(vec2(transPositions[3].x, transPositions[3].y));
//                     bool accepted[2];
//                     accepted[0] = vectorizer::intersectExclusive(vectBound, b0);
//                     accepted[1] = vectorizer::intersectExclusive(vectBound, b1);

//                     uint32_t indices[4];
//                     if (accepted[0] || accepted[1]) {
//                         for (uint32_t v = 0; v < 4; ++v) {
//                             if (v == 1 && !accepted[0]) continue;
//                             if (v == 3 && !accepted[1]) continue;
//                             auto it = vertexCache.find(transPositions[v]);
//                             if (it != vertexCache.end()) {
//                                 indices[v] = it->second;
//                             } else {
//                                 indices[v] = (uint32_t)vertexBuffer.size();
//                                 vertexCache.insert({ transPositions[v], (uint32_t)vertexBuffer.size() });
//                                 vertexBuffer.push_back(toVec3f(transPositions[v]));
//                                 if (shadingInfo) shadingInfo->vertexInvWs.push_back(invW[v]);
//                             }
//                         }
//                     }
//                     if (accepted[0]) {
//                         indexBuffer.insert(indexBuffer.end(), { indices[0], indices[1], indices[2] });
//                         if (shadingInfo) {
//                             shadingInfo->vertexNormals.insert(shadingInfo->vertexNormals.end(), { vn[0], vn[1], vn[2] });
//                             shadingInfo->vertexTexcoords.insert(shadingInfo->vertexTexcoords.end(), { tc[0], tc[1], tc[2] });
//                             shadingInfo->objectIds.push_back(prim.meshIndex);
//                         }
//                     }
//                     if (accepted[1]) {
//                         indexBuffer.insert(indexBuffer.end(), { indices[0], indices[2], indices[3] });
//                         if (shadingInfo) {
//                             shadingInfo->vertexNormals.insert(shadingInfo->vertexNormals.end(), { vn[0], vn[2], vn[3] });
//                             shadingInfo->vertexTexcoords.insert(shadingInfo->vertexTexcoords.end(), { tc[0], tc[2], tc[3] });
//                             shadingInfo->objectIds.push_back(prim.meshIndex);
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }

// template<typename TMode, typename>
// typename TMode::spectrum_type VectDirectIntegrator::evalDirectLighting(
//     const Vectorizer &primaryVect,
//     const std::vector<vec3f> &primaryVectVb,
//     const std::vector<uint32_t> &primaryVectIb,
//     const ShadingInfo &shadingInfo,
//     const Transform &invVpTrans, const Point &sensorPos,
//     const Scene &scene, const PerspectiveCamera &persp, const Point2i &pixel,
//     TMode &mode) const
// {
//     vectorizer::SampleArray sampleArray = mode.samplePrimaryVect(primaryVect, shadingInfo.objectIds);
//     if (sampleArray.count == 0) {
//         return TMode::spectrum_type(0.0);
//     }

//     RayDifferential rayDiff = createRayDiff(persp, pixel, sampleArray.count);

//     TMode::spectrum_type L(0.0);

//     uint32_t objId = shadingInfo.objectIds[sampleArray.entries[0].triId];
//     TMode::float_type objCoverage = mode.getSampleCoverage(sampleArray, 0);
//     uint32_t objSampleCount = 0;
//     TMode::spectrum_type Lobj(0.0);

//     for (uint32_t i = 0; i < sampleArray.count; ++i) {
//         //Intersection its = buildItsFromSample(sampleArray.entries[i],
//         //    primaryVectVb, primaryVectIb, shadingInfo, invVpTrans,
//         //    sensorPos, scene);
//         //its.getBSDF(rayDiff); // Calculate its partials here.
//         //uint32_t emitterCount = (uint32_t)scene.getEmitters().size();
//         //for (uint32_t emitterIndex = 0; emitterIndex < emitterCount; ++emitterIndex) {
//         //    const Emitter &emitter = *(scene.getEmitters()[emitterIndex]);
//         //    Lobj += evalDirectLightingSample<TMode>(sampleArray.entries[i],
//         //        its, sensorPos, scene, emitter, mode);
//         //}

//         ++objSampleCount;
//         if (i + 1 == sampleArray.count || shadingInfo.objectIds[sampleArray.entries[i + 1].triId] != objId) {
//             //L += Lobj * (objCoverage / TMode::float_type((Float)objSampleCount));
//             L += TMode::spectrum_type(objCoverage / TMode::float_type((Float)objSampleCount));
//             if (i + 1 < sampleArray.count) {
//                 objId = shadingInfo.objectIds[sampleArray.entries[i + 1].triId];
//                 objCoverage = mode.getSampleCoverage(sampleArray, i + 1);
//                 objSampleCount = 0;
//                 Lobj = TMode::spectrum_type(0.0);
//             }
//         }
//     }

//     return L;
// }

// template<typename SpectrumType>
// inline SpectrumType evalBSDFPolyIntegral(const BSDF *bsdf, const BSDFPolyIntegralContext &context, std::false_type doDiffRender) {
//     static_assert(std::is_same_v<SpectrumType, Spectrum>, "evalBSDFPolyIntegral() only accepts Spectrum or DSpectrum type.");
//     return bsdf->evalPolyIntegral(context);
// }

// template<typename SpectrumType>
// inline SpectrumType evalBSDFPolyIntegral(const BSDF *bsdf, const BSDFPolyIntegralContext &context, std::true_type doDiffRender) {
//     return bsdf->evalGradPolyIntegral(context);
// }

// template<typename SpectrumType>
// inline SpectrumType evalBSDFPolyIntegral(const BSDF *bsdf, const BSDFPolyIntegralContext &context) {
//     return evalBSDFPolyIntegral<SpectrumType>(bsdf, context, std::is_same<SpectrumType, DSpectrum>{});
// }

// template<typename SpectrumType>
// inline SpectrumType evalConstantEmitter(const Emitter &emitter, std::false_type doDiffRender) {
//     static_assert(std::is_same_v<SpectrumType, Spectrum>, "evalConstantEmitter() only accepts Spectrum or DSpectrum type.");
//     return emitter.evalPosition(PositionSamplingRecord()) * INV_PI;
// }

// template<typename SpectrumType>
// inline SpectrumType evalConstantEmitter(const Emitter &emitter, std::true_type doDiffRender) {
//     return DSpectrum(emitter.evalPosition(PositionSamplingRecord()) * INV_PI);
// }

// template<typename SpectrumType>
// inline SpectrumType evalConstantEmitter(const Emitter &emitter) {
//     return evalConstantEmitter<SpectrumType>(emitter, std::is_same<SpectrumType, DSpectrum>{});
// }

// template<typename TMode, typename>
// typename TMode::spectrum_type VectDirectIntegrator::evalDirectLightingSample(
//     const vectorizer::SampleEntry &entry, const Intersection &its,
//     const Point &sensorPos, const Scene &scene, const Emitter &emitter,
//     TMode &mode) const
// {
//     const Shape *emitterShape = emitter.getShape();
//     // Only area lights so far.
//     Assert(emitterShape->getClass()->derivesFrom(MTS_CLASS(TriMesh)));
//     const TriMesh *emitterMesh = static_cast<const TriMesh *>(emitterShape);
//     // Hack. Assume constant radiance.
//     TMode::spectrum_type emitterRadiance = evalConstantEmitter<TMode::spectrum_type>(emitter);
//     if (its.shape == emitterMesh) {
//         return emitterRadiance;
//     }

//     VectDirectLightingSampleRecord record;
//     if (!vectorizeDirectLightingSample(entry, its, sensorPos, scene, emitter, record)) {
//         return TMode::spectrum_type(0.0);
//     }

//     // TODO: gradients of triangle query.
//     Transform shadingVpTrans = record.shadingProjTrans * record.shadingViewTrans;

//     vectorizer::TriangleQueryInterface query;
//     for (uint32_t i = 0; i < emitterMesh->getTriangleCount(); ++i) {
//         Triangle tri = emitterMesh->getTriangles()[i];
//         vec3 projTri[3];
//         for (uint32_t j = 0; j < 3; ++j) {
//             Point ndc = shadingVpTrans(emitterMesh->getVertexPositions()[tri.idx[j]]);
//             projTri[j] = vec3(ndc.x, ndc.y, ndc.z);
//         }
//         vec2 e01 = vec2(projTri[1] - projTri[0]);
//         vec2 e12 = vec2(projTri[2] - projTri[1]);
//         if (e01.x * e12.y - e01.y * e12.x <= 0.0) {
//             continue;
//         }
//         mode.vectQueryTriangle(record.vect, projTri, query);
//     }

//     const BSDF *bsdf = its.getBSDF();
//     TMode::spectrum_type bsdfInt(0.0);
//     if (query.polyCount() > 0) {
//         Vector N = (Vector)its.shFrame.n;
//         Vector V = normalize(sensorPos - its.p);
//         Float NdotV = dot(N, V);
//         BSDFPolyIntegralContext context(query,
//             record.shadingProjTrans.inverse(), record.shadingViewTrans.inverse(), its, NdotV);
//         bsdfInt = evalBSDFPolyIntegral<TMode::spectrum_type>(bsdf, context);
//     }

//     return emitterRadiance * bsdfInt;
// }


// bool VectDirectIntegrator::vectorizeDirectLightingSample(
//     const vectorizer::SampleEntry &entry, const Intersection &its,
//     const Point &sensorPos, const Scene &scene, const Emitter &emitter,
//     VectDirectLightingSampleRecord &record) const
// {
//     const Shape *emitterShape = emitter.getShape();
//     BSphere emitterBSphere = emitterShape->getAABB().getBSphere();
//     const TriMesh *emitterMesh = static_cast<const TriMesh *>(emitterShape);

//     Vector shadingToEmitter = emitterBSphere.center - its.p;
//     Vector shadingToEmitterDir = shadingToEmitter;
//     Float shadingToEmitterDist = shadingToEmitterDir.length();
//     shadingToEmitterDir *= ((Float)1.0 / shadingToEmitterDist);

//     Frame viewFrame;
//     Transform shadingViewTrans;
//     {
//         Vector viewXAxis = cross(its.shFrame.n, -shadingToEmitterDir);
//         if (viewXAxis.lengthSquared() == 0.0) {
//             viewFrame = Frame(-shadingToEmitterDir);
//         } else {
//             viewXAxis = normalize(viewXAxis);
//             Vector viewYAxis = cross(-shadingToEmitterDir, viewXAxis);
//             Assert(dot(cross(viewXAxis, viewYAxis), -shadingToEmitterDir) > 0.0);
//             viewFrame = Frame(viewXAxis, viewYAxis, -shadingToEmitterDir);
//         }
//         shadingViewTrans = Transform::fromFrame(viewFrame).inverse() * Transform::translate(-(Vector)its.p);
//     }

//     // Actually want to further clip the frustum, as well as the light, to horizon/upper hemisphere.
//     Vector viewS = viewFrame.s * emitterBSphere.radius;
//     Vector viewT = viewFrame.t * emitterBSphere.radius;
//     Float d0 = dot(its.shFrame.n, (shadingToEmitter - viewT));
//     Float d1 = dot(its.shFrame.n, (shadingToEmitter + viewT));

//     Vector frustumCornerDirs[4];
//     vectorizer::AABB2 vectBound({ {-1.0, -1.0}, {1.0, 1.0} }); // or offset the projection matrix?
//     if (d0 <= 0.0 && d1 <= 0.0) {
//         // entire emitter frustum under horizon.
//         return false;
//     } else if (d0 >= 0.0 && d1 >= 0.0) {
//         // entire emitter frustum above horizon.
//         frustumCornerDirs[0] = shadingToEmitter - viewS - viewT;
//         frustumCornerDirs[1] = shadingToEmitter + viewS - viewT;
//         frustumCornerDirs[2] = shadingToEmitter + viewS + viewT;
//         frustumCornerDirs[3] = shadingToEmitter - viewS + viewT;
//     } else {
//         Float tClip = math::lerp(d0 / (d0 - d1), Float(-1.0), Float(1.0));
//         if (d0 < 0) {
//             frustumCornerDirs[0] = shadingToEmitter - viewS + tClip * viewT;
//             frustumCornerDirs[1] = shadingToEmitter + viewS + tClip * viewT;
//             frustumCornerDirs[2] = shadingToEmitter + viewS + viewT;
//             frustumCornerDirs[3] = shadingToEmitter - viewS + viewT;
//             vectBound.min.y = tClip;
//         } else {
//             frustumCornerDirs[0] = shadingToEmitter - viewS - viewT;
//             frustumCornerDirs[1] = shadingToEmitter + viewS - viewT;
//             frustumCornerDirs[2] = shadingToEmitter + viewS + tClip * viewT;
//             frustumCornerDirs[3] = shadingToEmitter - viewS + tClip * viewT;
//             vectBound.max.y = tClip;
//         }
//         Assert(dot(shadingToEmitterDir, cross(frustumCornerDirs[1] - frustumCornerDirs[0], frustumCornerDirs[2] - frustumCornerDirs[1])) < 0);
//         Assert(dot(shadingToEmitterDir, cross(frustumCornerDirs[2] - frustumCornerDirs[0], frustumCornerDirs[3] - frustumCornerDirs[2])) < 0);
//     }

//     // TODO: far clip is actually useful here.
//     Frustum shadingFrustum(its.p, frustumCornerDirs);

//     // TODO: This is wrong if the shading point is inside the bounding sphere.
//     Float halfFovSin = math::clamp(emitterBSphere.radius / shadingToEmitterDist, Float(0.0), Float(0.9));
//     // Float halfFovSin = emitterBSphere.radius / shadingToEmitterDist;
//     // Assert(halfFovSin < Float(1.0));
//     Float fov = radToDeg(Float(2.0) * std::asin(halfFovSin));
//     constexpr Float near(0.01);
//     Float far = shadingToEmitterDist + emitterBSphere.radius;
//     Transform shadingProjTrans = Transform::glPerspective(fov, near, far);

//     std::vector<FrustumIsectEntry> isects;
//     scene.frustumIntersect(shadingFrustum, isects);

//     VertexCache vertexCache;
//     std::vector<vec3f> vertexBuffer;
//     std::vector<uint32_t> indexBuffer;
//     prepareVectorizer(&scene, shadingViewTrans, shadingProjTrans,
//         near, vectBound, isects, emitterMesh,
//         vertexCache, vertexBuffer, indexBuffer);


//     //////////////////////////////////////////////////////////////////////////
//     // Debug
//     // dumpVectorizerInput(&scene, Point2i(217, 714), vectBound, vertexBuffer, indexBuffer);
//     //////////////////////////////////////////////////////////////////////////

//     record.vect.reset(vectBound);
//     record.vect.process(vertexBuffer.data(), (uint32_t)vertexBuffer.size(),
//         indexBuffer.data(), (uint32_t)indexBuffer.size());
//     record.vectVertexBuffer = std::move(vertexBuffer);
//     record.vectIndexBuffer = std::move(indexBuffer);
//     record.shadingViewTrans = shadingViewTrans;
//     record.shadingProjTrans = shadingProjTrans;
//     return true;
// }

// Intersection VectDirectIntegrator::buildItsFromSample(const vectorizer::SampleEntry &entry,
//     const std::vector<vec3f> &primaryVectVb, const std::vector<uint32_t> &primaryVectIb,
//     const ShadingInfo &shadingInfo, const Transform &invVpTrans, const Point &sensorPos,
//     const Scene &scene) const
// {
//     Intersection its;
//     // Irrelevant fields.
//     its.time = Float(0.0);
//     its.color = Spectrum(0.0);
//     its.instance = nullptr;
//     its.wi = Vector(0.0);

//     // Useful fields.
//     its.shape = scene.getMeshes()[shadingInfo.objectIds[entry.triId]];
//     // TODO: its.primIndex

//     // Transform back to world pos.
//     its.p = Point(entry.pos.x, entry.pos.y, entry.pos.z);
//     its.p = invVpTrans(its.p);

//     Vector dp02, dp12;
//     {
//         Point p0 = invVpTrans(toPoint(primaryVectVb[primaryVectIb[entry.triId * 3 + 0]]));
//         Point p1 = invVpTrans(toPoint(primaryVectVb[primaryVectIb[entry.triId * 3 + 1]]));
//         Point p2 = invVpTrans(toPoint(primaryVectVb[primaryVectIb[entry.triId * 3 + 2]]));
//         its.geoFrame = Frame(normalize(cross(p1 - p0, p2 - p1)));
//         dp02 = p0 - p2;
//         dp12 = p1 - p2;
//     }

//     constexpr Float offsetEps(1e-3);
//     its.p += offsetEps * its.geoFrame.n;

//     Vector3 perspCoord(entry.coord.x, entry.coord.y, Float(1.0) - entry.coord.x - entry.coord.y);
//     {
//         Float denom(0.0);
//         for (uint32_t i = 0; i < 3; ++i) {
//             perspCoord[i] *= shadingInfo.vertexInvWs[primaryVectIb[entry.triId * 3 + i]];
//             denom += perspCoord[i];
//         }
//         perspCoord *= Float(1.0) / denom;
//     }

//     {
//         Normal n0 = shadingInfo.vertexNormals[entry.triId * 3 + 0];
//         Normal n1 = shadingInfo.vertexNormals[entry.triId * 3 + 1];
//         Normal n2 = shadingInfo.vertexNormals[entry.triId * 3 + 2];

//         Vector N = normalize(n0 * perspCoord[0] + n1 * perspCoord[1] + n2 * perspCoord[2]);
//         // Important: do not construct an arbitrary shading frame!
//         // LTC requires the frame to be constructed this way.
//         Vector V = normalize(sensorPos - its.p);
//         Vector T1 = normalize(V - N * dot(V, N));
//         Vector T2 = cross(N, T1);
//         its.shFrame = Frame(T1, T2, N);
//     }

//     Vector2 duv02, duv12;
//     {
//         Point2 tc0 = shadingInfo.vertexTexcoords[entry.triId * 3 + 0];
//         Point2 tc1 = shadingInfo.vertexTexcoords[entry.triId * 3 + 1];
//         Point2 tc2 = shadingInfo.vertexTexcoords[entry.triId * 3 + 2];
//         its.uv = tc0 * perspCoord[0] + tc1 * perspCoord[1] + tc2 * perspCoord[2];
//         duv02 = tc0 - tc2, duv12 = tc1 - tc2;
//     }

//     Float duvDet = duv02[0] * duv12[1] - duv02[1] * duv12[0];
//     if (duvDet == 0.0) {
//         its.dpdu = its.geoFrame.s;
//         its.dpdv = its.geoFrame.t;
//     } else {
//         Float invDuvDet = Float(1.0) / duvDet;
//         its.dpdu = (duv12[1] * dp02 - duv02[1] * dp12) * invDuvDet;
//         its.dpdv = (-duv12[0] * dp02 + duv02[0] * dp12) * invDuvDet;
//     }
//     its.hasUVPartials = false;
//     return its;
// }


// Spectrum VectDirectIntegrator::Li(const RayDifferential &ray, RadianceQueryRecord &rRec) const
// {
//     Log(EError, "VectDirectIntegrator doesn't really have a meaningful Li() method. This is likely a bug.");
//     return Spectrum(0.0);
// }

// Spectrum VectDirectIntegrator::E(const Scene *scene, const Intersection &its,
//     const Medium *medium, Sampler *sampler, int nSamples,
//     bool includeIndirect) const
// {
//     Log(EError, "Not implemented yet, but should?");
//     return Spectrum(0.0);
// }

// void VectDirectIntegrator::bindUsedResources(ParallelProcess *proc) const
// {

// }

// void VectDirectIntegrator::wakeup(ConfigurableObject *parent, std::map<std::string, SerializableObject *> &params)
// {

// }

// //////////////////////////////////////////////////////////////////////////
// MTS_IMPLEMENT_CLASS_S(VectDirectIntegrator, false, SamplingIntegrator)
// MTS_EXPORT_PLUGIN(VectDirectIntegrator, "Vectorizer direct integrator");
// MTS_NAMESPACE_END