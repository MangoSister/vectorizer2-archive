import os, sys, glob
import numpy as np
import cv2
import csv

def load_exr(filename):
    bgr = np.array(cv2.imread(filename, cv2.IMREAD_UNCHANGED), dtype=np.float)
    if bgr.ndim >= 3:
        img = bgr[...,::-1]
    else:
        img = bgr
    return img

class Adam:
    def reset(self, init_params, alpha, beta1, beta2, eps):
        if (init_params.shape != alpha.shape):
            raise Exception('Mismatched alpha dimension')
        if (init_params.shape != beta1.shape):
            raise Exception('Mismatched beta1 dimension')
        if (init_params.shape != beta2.shape):
            raise Exception('Mismatched beta2 dimension')
        if (init_params.shape != eps.shape):
            raise Exception('Mismatched eps dimension')
        self.alpha = alpha
        self.beta1 = beta1
        self.beta2 = beta2
        self.eps = eps

        self.mt = np.zeros_like(init_params)
        self.vt = np.zeros_like(init_params)
        self.t = 0
        self.params = init_params

    def update(self, grad):
        if (self.params.shape != grad.shape):
            raise Exception('Mismatched grad dimension')
        self.t = self.t + 1
        self.mt = self.beta1 * self.mt + (1.0 - self.beta1) * grad
        self.vt = self.beta2 * self.vt + (1.0 - self.beta2) * (grad * grad)
        mcap = self.mt / (1.0 - np.power(self.beta1, float(self.t)))
        vcap = self.vt / (1.0 - np.power(self.beta2, float(self.t)))
        self.params = self.params - self.alpha * mcap / (np.sqrt(vcap) + self.eps)

    def __str__(self):
        s = "alpha: " + str(self.alpha) + "\n"
        s += "beta1: " + str(self.beta1) + "\n"
        s += "beta2: " + str(self.beta2) + "\n"
        s += "eps: " + str(self.eps) + "\n"
        s += "mt: " + str(self.mt) + "\n"
        s += "vt: " + str(self.vt) + "\n"
        s += "t: " + str(self.t) + "\n"
        s += "params: " + str(self.params)
        return s

###################################################
sys.path.append(os.path.join(sys.path[0], 'mitsuba-fork', 'dist', 'python', '3.6'))
os.environ['PATH'] = os.path.join(sys.path[0], 'mitsuba-fork', 'dist') + os.pathsep + os.environ['PATH']

import mitsuba
from mitsuba.core import *
from mitsuba.render import SceneHandler, RenderQueue, RenderJob
import multiprocessing

logger = Thread.getThread().getLogger()
logger.setLogLevel(EError)

fileResolver = Thread.getThread().getFileResolver()
fileResolver.appendPath('../test/assets/mitsuba-scenes/tree/')

scheduler = Scheduler.getInstance()
# Start up the scheduling system with one worker per local core
for i in range(0, multiprocessing.cpu_count()):
    scheduler.registerWorker(LocalWorker(i, 'wrk%i' % i))
scheduler.start()

work_dir = "tree-invrender"
os.makedirs(work_dir, exist_ok=True)
files = glob.glob(os.path.join(work_dir, "*"))
for f in files:
    os.remove(f)

img_target = load_exr("tree-invrender-target.exr")
width = img_target.shape[1]
height = img_target.shape[0]

num_param = 1
params_target = np.array([0.6])
params = np.zeros(num_param)
param_ranges_min = np.zeros(num_param)
param_ranges_max = np.zeros(num_param)

params[0] = 0  # light position x
param_ranges_min[0] = 0
param_ranges_max[0] = 6.28

params_init = np.empty_like(params)
params_init[:] = params

# common: alpha = 0.001, beta1 = 0.9, beta2 = 0.999, eps = 1e-8
alpha = np.array([0.02])
beta1 = np.full((num_param), 0.9)
beta2 = np.full((num_param), 0.999)
eps = np.full((num_param), 1e-8)

adam = Adam()
adam.reset(params, alpha, beta1, beta2, eps)

max_step = 100
threshold_rmse = 1e-4
print("**************************")
print("Max step: %d" % max_step)
print("Image RMSE Threshold: %f" % threshold_rmse)
print("Initial params: ", adam.params)
print("Optimizer info:")
print(adam)
print("**************************")

img_loss_rmse = []
param_loss_rmse = []
succ = False
for step in range(0, max_step):
    print("Step [%d/%d]..." % (step+1, max_step))

    params = StringMap()
    for i in range(0, adam.params.size):
        params["invrender-param-%d" % i] = str(adam.params[i] / np.pi * 180) # Note here
    scene = SceneHandler.loadScene(fileResolver.resolve("tree-invrender.xml"), params)
    scene.setBlockSize(16)

    # Create a queue for tracking render jobs
    queue = RenderQueue()
    name = "step-%d" % step
    scene.setDestinationFile(os.path.join(work_dir, name))
    job = RenderJob(name, scene, queue)
    job.start()

    queue.waitLeft(0)
    queue.join()

    img_val = load_exr(os.path.join(work_dir, "step-%d.exr") % step)
    # L2 loss. Other loss functions?
    diff = img_val - img_target
    loss = diff ** 2
    rmse = np.sqrt(np.mean(loss))
    img_loss_rmse.append(rmse)

    param_rmse = np.sqrt(np.mean(np.square(adam.params - params_target)))
    param_loss_rmse.append(param_rmse)

    grad = np.zeros(adam.params.size)
    for i in range(0, adam.params.size):
        img_grad = load_exr(os.path.join(work_dir, "step-%d-grad%d.exr") % (step, i))

        d_loss = 2 * diff * img_grad
        grad[i] = np.mean(d_loss)

    if rmse <= threshold_rmse:
        succ = True
        print("\n")
        print("Successfully reached target image RMSE theshold.")
        break

    old_params = np.empty_like(adam.params)
    old_params[:] = adam.params

    adam.update(grad)
    adam.params = np.clip(adam.params, param_ranges_min, param_ranges_max)

    print("\n")
    print("Image RMSE: %f" % rmse)
    print("Param RMSE: %f" % param_rmse)
    print("Updated params: ", old_params, " -> ", adam.params)

if succ == False:
    print("Did not reach target image RMSE threshold (%f) in %d steps." % (threshold_rmse, max_step))
print("Final params: ", adam.params)

with open(os.path.join(sys.path[0], work_dir, "record.csv"), 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(["Target Params"])
    writer.writerow(params_target.tolist())
    writer.writerow(["Init Params"])
    writer.writerow(params_init.tolist())
    writer.writerow(["Final Params"])
    writer.writerow(adam.params.tolist())

    writer.writerow(["Step #", "Image Loss RMSE", "Param Loss RMSE"])
    for i in range(0, len(img_loss_rmse)):
        writer.writerow([i, img_loss_rmse[i], param_loss_rmse[i]])