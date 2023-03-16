import math
import numpy as np
import os, sys, glob, shutil, struct
import cv2
import csv
sys.path.append(os.path.join(sys.path[0], '..',))
sys.path.append(os.path.join(sys.path[0], '..', 'mitsuba-fork', 'dist', 'python', '3.6'))
os.environ['PATH'] = os.path.join(sys.path[0], '..', 'mitsuba-fork', 'dist') + os.pathsep + os.environ['PATH']
from invutil import *

work_dir = "work-dir"
scene_dir = "../../test/assets/mitsuba-scenes/mesh-opt/"
scene_name = "mesh-opt.xml"

if os.path.exists(work_dir) and os.path.isdir(work_dir):
    shutil.rmtree(work_dir, ignore_errors=True)
os.makedirs(work_dir, exist_ok=True)

sphere_path = "C:/vectorizer2/tool/mesh-opt/icosphere.obj"
cube_path = "C:/vectorizer2/tool/mesh-opt/cube.obj"
origin_mesh_path = sphere_path
grad_scale = 1.0
face_normal = "false"

view_origin = ["20 10 -20", "-10 3 10", "0 5 -10", "0 15 0", "0 5 10", "0 -5 0", "10 5 0", "-10 5 0"]
view_target = ["0 4 0", "0 6 0", "0 5 0", "0 5 0", "0 5 0", "0 5 0", "0 5 0", "0 5 0"]
view_up = ["0 1 0", "0 1 0", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 1 0", "0 1 0"]
num_view = len(view_origin)

mitsuba_init(scene_dir)

def write_coefficients(path, coeffs):
    with open(path, "wb") as out:
        out.write(np.uint32(coeffs.shape[0]).tobytes())
        out.write(np.float32(coeffs).tobytes())

def read_coefficients(path):
    buf = open(path, "rb").read()
    count = struct.unpack("I", buf[:4])[0]
    coeffs = struct.unpack("f"*count, buf[4:])
    if len(coeffs) != count:
        raise RuntimeError("Wrong coeffs count.")
    return np.array(coeffs, dtype=np.float32)

##########################################
print("Render target image...")
img_target_path = []
img_target = []

for view in range(num_view):
    print("\n")
    print("Target View [{}/{}]...".format(view, num_view))
    img_target_path.append(os.path.join(work_dir, "target-{}".format(view)))
    sc_params = StringMap()
    sc_params["invrender-param-view-origin"] = view_origin[view]
    sc_params["invrender-param-view-target"] = view_target[view]
    sc_params["invrender-param-view-up"] = view_up[view]
    sc_params["invrender-param-mesh"] = origin_mesh_path
    sc_params["invrender-param-target"] = "targetMesh"
    sc_params["invrender-param-grad-scale"] = str(grad_scale)
    sc_params["invrender-param-face-normal"] = face_normal
    sc_params["invrender-param-output-mode"] = "none"
    sc_params["invrender-param-disp-type"] = str(1)
    sc_params["invrender-param-disp-params"] = "0.5 5.0 5.0"
    sc_params["invrender-param-disp-coeffs"] = ""
    if view == 0:
        sc_params["invrender-param-output-vertex-coeffs"] = "true"
    else:
        sc_params["invrender-param-output-vertex-coeffs"] = "false"
    mitsuba_render(scene_name, img_target_path[view], sc_params)
    img_target.append(load_exr(img_target_path[view] + ".exr"))

params_target = read_coefficients(os.path.join(work_dir, "coeffs.bin"))
num_param = params_target.shape[0]
params = np.zeros(num_param)
params_init = np.copy(params)
param_ranges_min = np.full(num_param, -1.0)
param_ranges_max = np.full(num_param, 1.0)
# common: alpha = 0.001, beta1 = 0.9, beta2 = 0.999, eps = 1e-8
alpha = np.full(num_param, 0.05)
beta1 = np.full(num_param, 0.9)
beta2 = np.full(num_param, 0.999)
eps = np.full(num_param, 1e-8)
adam = Adam()
adam.reset(params, alpha, beta1, beta2, eps)
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
for step in range(0, max_step):
    print("Step [%d/%d]..." % (step+1, max_step))
    step_name = "step-%d" % step
    step_dir = os.path.join(work_dir, step_name)
    os.makedirs(step_dir, exist_ok=True)

    sc_params = StringMap()
    sc_params["invrender-param-mesh"] = origin_mesh_path
    sc_params["invrender-param-target"] = "targetMesh"
    sc_params["invrender-param-grad-scale"] = str(grad_scale)
    sc_params["invrender-param-face-normal"] = face_normal
    sc_params["invrender-param-output-mode"] = "both"
    if step == 0:
        sc_params["invrender-param-disp-type"] = str(0)
        sc_params["invrender-param-disp-params"] = "0.0"
        sc_params["invrender-param-disp-coeffs"] = ""
        sc_params["invrender-param-output-vertex-coeffs"] = "true"
    else:
        sc_params["invrender-param-disp-type"] = str(2)
        coeffs_path = os.path.join(step_dir, "coeffs.bin")
        write_coefficients(coeffs_path, adam.params)
        sc_params["invrender-param-disp-coeffs"] = os.path.abspath(coeffs_path)
        sc_params["invrender-param-disp-params"] = "0.0"
        sc_params["invrender-param-output-vertex-coeffs"] = "false"

    for view in range(num_view):
        print("\n")
        print("Step [{}/{}] View [{}/{}]...".format(step+1, max_step, view, num_view))
        view_name = "view-{}".format(view)
        view_dir = os.path.join(step_dir, view_name)
        os.makedirs(view_dir, exist_ok=True)
        dest_path = os.path.join(view_dir, "step-{}-view-{}".format(step, view))

        sc_params["invrender-param-view-origin"] = view_origin[view]
        sc_params["invrender-param-view-target"] = view_target[view]
        sc_params["invrender-param-view-up"] = view_up[view]
        if view != 0:
            sc_params["invrender-param-output-vertex-coeffs"] = "false"

        mitsuba_render(scene_name, dest_path, sc_params)

    mse = 0.0
    grad = np.zeros(adam.params.size)
    for view in range(num_view):
        view_name = "view-%d" %view
        view_dir = os.path.join(step_dir, view_name)
        dest_path = os.path.join(view_dir, "step-{}-view-{}".format(step, view))
        img_val = load_exr(dest_path + ".exr")
        diff = img_val - img_target[view]
        mse += np.mean(diff ** 2)
        for i in range(0, adam.params.size):
            img_grad_path = dest_path + "-grad{}.exr".format(i)
            img_grad = load_exr(img_grad_path)
            grad[i] += np.mean(2 * diff * img_grad)

    rmse = np.sqrt(mse)
    img_loss_rmse.append(rmse)

    param_rmse = np.sqrt(np.mean(np.square(adam.params - params_target)))
    param_loss_rmse.append(param_rmse)

    old_params = np.copy(adam.params)
    adam.update(grad)
    adam.params = np.clip(adam.params, param_ranges_min, param_ranges_max)

    print("\n")
    print("Image RMSE: %f" % rmse)
    print("Param RMSE: %f" % param_rmse)
    # print("Updated params: ", old_params, " -> ", adam.params)

    record_progress(os.path.join(step_dir, "record.csv"), params_target, params_init, adam.params, img_loss_rmse, param_loss_rmse)

print("Final params: ", adam.params)