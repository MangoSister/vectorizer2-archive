import os, sys, glob, shutil
import numpy as np
import cv2
import csv

def clear_dir(path):
    if os.path.exists(path) and os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path, exist_ok=True)

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
        self.params = np.copy(init_params)

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

def record_progress(loss_path, img_loss_rmse, param_loss_rmse, params_path, params):
    with open(loss_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        # writer.writerow(["Step #", "Image Loss RMSE", "Param Loss RMSE"])
        for i in range(0, len(img_loss_rmse)):
            writer.writerow([i, img_loss_rmse[i], param_loss_rmse[i]])

    with open(params_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        # writer.writerow(["Step #", "Params"])
        for i in range(len(params)):
            p = params[i].tolist()
            p.insert(0, i)
            writer.writerow(p)

import mitsuba
from mitsuba.core import *
from mitsuba.render import SceneHandler, RenderQueue, RenderJob
import multiprocessing

def mitsuba_init(scene_dir):
    logger = Thread.getThread().getLogger()
    logger.setLogLevel(EError)
    fileResolver = Thread.getThread().getFileResolver()
    fileResolver.appendPath(scene_dir)

    scheduler = Scheduler.getInstance()
    # Start up the scheduling system with one worker per local core
    for i in range(0, multiprocessing.cpu_count()):
        scheduler.registerWorker(LocalWorker(i, 'wrk%i' % i))
    scheduler.start()

def mitsuba_render(scene_name, dest_path, params):
    fileResolver = Thread.getThread().getFileResolver()
    scene = SceneHandler.loadScene(fileResolver.resolve(scene_name), params)
    scene.setBlockSize(16)
    scene.setDestinationFile(dest_path)
    queue = RenderQueue()
    job = RenderJob("job", scene, queue)
    job.start()
    queue.waitLeft(0)
    queue.join()