import os, sys, glob
import numpy as np
import scipy.optimize
import cv2
import csv

def load_exr(filename):
    bgr = np.array(cv2.imread(filename, cv2.IMREAD_UNCHANGED), dtype=np.float)
    if bgr.ndim >= 3:
        img = bgr[...,::-1]
    else:
        img = bgr
    return img

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

params[0] = 0.0  # light position x
param_bounds = [(0, 6.28)]

step_counter = 0

def diff_render(vals, *args):
    grad_scale = args[0]
    global step_counter

    params = StringMap()
    for i in range(0, num_param):
        params["invrender-param-%d" % i] = str(vals[i] / np.pi * 180) # Note here
    scene = SceneHandler.loadScene(fileResolver.resolve("tree-invrender.xml"), params)
    scene.setBlockSize(16)

    # Create a queue for tracking render jobs
    queue = RenderQueue()

    name = "step-%d" % step_counter
    scene.setDestinationFile(os.path.join(work_dir, name))
    job = RenderJob(name, scene, queue)
    job.start()

    queue.waitLeft(0)
    queue.join()

    img_val = load_exr(os.path.join(work_dir, "step-%d.exr") % step_counter)
    # L2 loss. Other loss functions?
    diff = img_val - img_target
    loss = np.mean(diff ** 2)

    grad = np.zeros(num_param)
    for i in range(0, num_param):
        img_grad = load_exr(os.path.join(work_dir, "step-%d-grad%d.exr") % (step_counter, i))

        d_loss = 2 * diff * img_grad
        grad[i] = np.mean(d_loss)

    step_counter += 1

    loss *= grad_scale
    grad *= grad_scale

    print("----")
    print("step: {}".format(step_counter))
    print("val: {}".format(vals))
    print("grad: {}".format(grad))
    print("loss: {}".format(loss))

    return loss, grad

options = { "maxiter" : 100 }
grad_scale = 1e3
args = (grad_scale)
res = scipy.optimize.minimize(diff_render, params, args=args, jac=True, method='L-BFGS-B', bounds=param_bounds, options=options)
# res = scipy.optimize.minimize(diff_render, params, jac=True, method='BFGS', options=options)

print(res)
