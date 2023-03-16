import math
import numpy as np
import os, sys, glob, shutil, struct
import cv2
import csv
sys.path.append(os.path.join(sys.path[0], 'mitsuba-fork', 'dist', 'python', '3.6'))
os.environ['PATH'] = os.path.join(sys.path[0], 'mitsuba-fork', 'dist') + os.pathsep + os.environ['PATH']
from invutil import *

work_dir = "work-dir"
scene_dir = "../test/assets/mitsuba-scenes/dragon/"
scene_name = "dragon-invrender.xml"

clear_dir(work_dir)

mitsuba_init(scene_dir)

num_param = 4
params_target = np.array([0.2, 0.2, 0.3, 0.6776])
params_init = np.full(num_param, 0.0)
#
param_ranges_min = np.array([0, -0.5, 0, 0])
param_ranges_max = np.array([1, 0.3, 1, 0.98])
# common: alpha = 0.001, beta1 = 0.9, beta2 = 0.999, eps = 1e-8
alpha = np.full(num_param, 0.05)
beta1 = np.full(num_param, 0.9)
beta2 = np.full(num_param, 0.999)

eps = np.full(num_param, 1e-8)
adam = Adam()
adam.reset(params_init, alpha, beta1, beta2, eps)

def write_sc_params(params):
    sc_params = StringMap()
    sc_params["invrender-param-0"] = str(0 + params[0] * 100)
    sc_params["invrender-param-1"] = str(50 - params[1] * 100)
    sc_params["invrender-param-2"] = str(0 + params[2] * 100)
    sc_params["invrender-param-3"] = str(np.sqrt(0.9801 - params[3]))
    return sc_params

##########################################
print("Render target image...")
img_target_path = os.path.join(work_dir, "target")
sc_params = write_sc_params(params_target)
sc_params["invrender-param-output-mode"] = "none"
mitsuba_render(scene_name, img_target_path, sc_params)
img_target = load_exr(img_target_path + ".exr")

##########################################

max_step = 100
print("\n")
print("**************************")
print("Max step: %d" % max_step)
# print("Initial params: ", adam.params)
# print("Optimizer info:")
# print(adam)
print("**************************")

img_loss_rmse = []
param_loss_rmse = []
all_params = []
for step in range(0, max_step):
    print("Step [%d/%d]..." % (step+1, max_step))
    step_name = "step-%d" % step
    step_dir = os.path.join(work_dir, step_name)
    os.makedirs(step_dir, exist_ok=True)
    dest_path = os.path.join(step_dir, step_name)
    sc_params = write_sc_params(adam.params)
    sc_params["invrender-param-output-mode"] = "component"
    mitsuba_render(scene_name, dest_path, sc_params)

    mse = 0.0
    grad = np.zeros(adam.params.size)

    img_val = load_exr(dest_path + ".exr")
    diff = img_val - img_target
    mse += np.mean(diff ** 2)
    # sgn_diff = np.sign(diff)
    for i in range(0, adam.params.size):
        img_grad_path = dest_path + "-grad{}.exr".format(i)
        img_grad = load_exr(img_grad_path)
        # l2 loss
        grad[i] += np.mean(2 * diff * img_grad)
        # l1 loss
        # grad[i] += np.mean(sgn_diff * img_grad)

    # connect grad to params
    grad[0] *= 100
    grad[1] *= -100
    grad[2] *= 100
    grad[3] *= -0.5*np.power(0.9801-adam.params[3], -0.5)

    rmse = np.sqrt(mse)
    img_loss_rmse.append(rmse)

    param_rmse = np.sqrt(np.mean(np.square(adam.params - params_target)))
    param_loss_rmse.append(param_rmse)

    old_params = np.copy(adam.params)
    all_params.append(old_params)

    adam.update(grad)
    adam.params = np.clip(adam.params, param_ranges_min, param_ranges_max)

    print("\n")
    print("Image RMSE: %f" % rmse)
    print("Param RMSE: %f" % param_rmse)
    print("Grad: ", grad)
    print("Updated params: ", old_params, " -> ", adam.params)

    record_progress(os.path.join(step_dir, "loss.csv"), img_loss_rmse, param_loss_rmse,
        os.path.join(step_dir, "params.csv"), all_params)

print("Final params: ", adam.params)