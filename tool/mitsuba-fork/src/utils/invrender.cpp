// #include <mitsuba/core/plugin.h>
// #include <mitsuba/core/bitmap.h>
// #include <mitsuba/core/fresolver.h>
// #include <mitsuba/render/util.h>
// #include <mitsuba/render/renderqueue.h>
// #include <mitsuba/render/renderjob.h>
// #include <iostream>

// MTS_NAMESPACE_BEGIN

// template <typename T>
// struct AdamOptimizer
// {
//     AdamOptimizer(Float alpha = Float(0.001), Float beta1 = Float(0.9), Float beta2 = Float(0.999), Float eps = Float(1e-8)) :
//         alpha(alpha), beta1(beta1), beta2(beta2), eps(eps) {}

//     void reset(const T &initValue) {
//         value = initValue;
//         mt = T(0);
//         vt = T(0);
//         t = 0;
//     }

//     void update(const T &grad) {
//         ++t;
//         mt = beta1 * mt + (Float(1) - beta1) * grad;	        // Update biased first moment estimate.
//         vt = beta2 * vt + (Float(1) - beta2) * (grad * grad);   // Update biased second raw moment estimate.
//         T mcap = mt / (Float(1) - std::pow(beta1, Float(t)));   // Compute bias-corrected first moment estimate.
//         T vcap = vt / (Float(1) - std::pow(beta2, Float(t)));   // Compute bias-corrected second raw moment estimate.
//         value = value - alpha * mcap / (std::sqrt(vcap) + eps);
//     }

//     T value = T(0);
//     T mt = T(0); // First moment
//     T vt = T(0); // Second moment.
//     uint32_t t = 0;

//     // Hyper-parameters.
//     Float alpha;
//     Float beta1;
//     Float beta2;
//     Float eps;
// };

// class InvRender : public Utility {
// public:
//     int run(int argc, char **argv) {
//         if (argc < 7) {
//             cout << "Minimal inverse renderer that optimizes one scalar parameter using the vectorizer." << endl;
//             cout << R"(Syntax: mtsutil invrender
//                     <scene path> <target exr image>
//                     <object name> <parameter name> <init parameter value>
//                     <loss threshold> [maxStep=10])" << endl;
//             return -1;
//         }

//         fs::path scenePath(argv[1]);
//         fs::path targetImagePath(argv[2]);
//         std::string objectName(argv[3]);
//         std::string parameterName(argv[4]);
//         Float initValue = std::stof(argv[5]); // how to send the parameter to the scene
//         Float threshold = std::stof(argv[6]);
//         uint32_t maxSteps = 10;
//         if (argc > 7) {
//             maxSteps = std::stoi(argv[7]);
//         }

//         fs::path outputDir = targetImagePath;
//         outputDir.remove_filename();

//         ref<Scene> scene = loadScene(scenePath);

//         ref<PluginManager> pluginMgr = PluginManager::getInstance();

//         Properties integratorPropsShared;
//         integratorPropsShared.setPluginName("vecdirect");
//         integratorPropsShared.setString("diffRenderObject", objectName);
//         integratorPropsShared.setString("diffRenderParameter", parameterName);

//         ref<Bitmap> targetImage = new Bitmap(targetImagePath); // Assuming Bitmap::EOpenEXR.
//         Vector2i imageSize = targetImage->getSize();
//         ref<Bitmap> valueImage = new Bitmap(Bitmap::EPixelFormat::ERGB, Bitmap::EComponentFormat::EFloat32,
//             scene->getFilm()->getSize());
//         ref<Bitmap> gradImage = new Bitmap(Bitmap::EPixelFormat::ERGB, Bitmap::EComponentFormat::EFloat32,
//             scene->getFilm()->getSize());

//         ref<RenderQueue> renderQueue = new RenderQueue();


//         AdamOptimizer<Float> adam(0.1);
//         adam.reset(initValue);
//         NeumaierSum lossAcc;
//         NeumaierSum gradLossAcc;
//         Log(EInfo, "Training started: ");
//         Log(EInfo, "Init parameter value: %f", adam.value);
//         Log(EInfo, "Adam alpha: %.4f", adam.alpha);
//         Log(EInfo, "Adam beta1: %.4f", adam.beta1);
//         Log(EInfo, "Adam beta2: %.4f", adam.beta2);
//         Log(EInfo, "Adam eps: %f", adam.eps);

//         while (true) {
//             // render value image
//             // TODO: how to switch mode?
//             integratorPropsShared.setFloat("diffRenderParamDelta", adam.value - initValue, false);
//             integratorPropsShared.setBoolean("diffRender", false, false);
//             ref<Integrator> valueIntegrator = static_cast<Integrator *> (pluginMgr->
//                 createObject(Integrator::m_theClass, integratorPropsShared));
//             ref<RenderJob> renderJob = new RenderJob("rend", scene, renderQueue,
//                 -1, -1, -1, false, false);
//             scene->setIntegrator(valueIntegrator);
//             renderJob->start();
//             renderQueue->waitLeft(0);
//             scene->getFilm()->develop(
//                 Point2i(0, 0), scene->getFilm()->getSize(),
//                 Point2i(0, 0), valueImage);

//             // render grad image
//             // TODO: how to switch mode?
//             integratorPropsShared.setBoolean("diffRender", true, false);
//             ref<Integrator> gradIntegrator = static_cast<Integrator *> (pluginMgr->
//                 createObject(Integrator::m_theClass, integratorPropsShared));
//             renderJob = new RenderJob("rend", scene, renderQueue,
//                 -1, -1, -1, false, false);
//             scene->setIntegrator(gradIntegrator);
//             renderJob->start();
//             renderQueue->waitLeft(0);
//             scene->getFilm()->develop(
//                 Point2i(0, 0), scene->getFilm()->getSize(),
//                 Point2i(0, 0), gradImage);

//             lossAcc.reset();
//             gradLossAcc.reset();
//             // compute loss and propagate gradients.
//             float *refBuf = targetImage->getFloat32Data();
//             float *valueBuf = valueImage->getFloat32Data();
//             float *dPixeldParamBuf = gradImage->getFloat32Data();

//             for (int i = 0; i < imageSize.x * imageSize.y; ++i) {
//                 Color3 refPixel = *reinterpret_cast<Color3 *>(&refBuf[3 * i]);
//                 Color3 valuePixel = *reinterpret_cast<Color3 *>(&valueBuf[3 * i]);
//                 Color3 dPixeldParam = *reinterpret_cast<Color3 *>(&dPixeldParamBuf[3 * i]);
//                 for (int dim = 0; dim < 3; ++dim) {
//                     // L1 loss
//                     float diff = refPixel[dim] - valuePixel[dim];
//                     float loss = std::abs(diff);
//                     float dLossdPixel = diff >= 0.0f ? -1.0f : 1.0f;
//                     // L2 loss?

//                     lossAcc += loss;
//                     gradLossAcc += (dLossdPixel * dPixeldParam[dim]);
//                 }
//             }
//             Float loss = (Float)lossAcc / Float(imageSize.x * imageSize.y);
//             Float gradLoss = (Float)gradLossAcc / Float(imageSize.x * imageSize.y);
//             Log(EInfo, "[Step %u/%u] parameter: %.4f | loss: %.4f | grad: %.4f", adam.t, maxSteps, adam.value, loss, gradLoss);
//             char filename[64];
//             sprintf(filename, "step_%u.exr", adam.t);
//             fs::path outputPath = outputDir / fs::path(filename);
//             Log(EInfo, "[Step %u/%u] save to %s", adam.t, maxSteps, outputPath.c_str());
//             valueImage->write(outputPath);

//             if (std::abs(loss) < threshold || adam.t >= maxSteps) {
//                 break;
//             }
//             // update parameter
//             adam.update(gradLoss);
//             // TODO: how to send the updated parameter to the scene?
//         }

//         Log(EInfo, "Training finished:");
//         Log(EInfo, "Steps taken: %u/%u", adam.t, maxSteps);
//         Log(EInfo, "Final parameter value: %.4f", adam.value);
//         Log(EInfo, "Final loss", adam.value);

//         return 0;
//     }

//     MTS_DECLARE_UTILITY()
// };

// MTS_EXPORT_UTILITY(InvRender, "Minimal inverse renderer")
// MTS_NAMESPACE_END
