import os, sys, shutil, glob
import numpy as np
import cv2

sys.path.append(os.path.join(sys.path[0], 'gradvis'))
import vis

sys.path.append(os.path.join(sys.path[0], 'mitsuba-fork', 'dist', 'python', '3.6'))
os.environ['PATH'] = os.path.join(sys.path[0], 'mitsuba-fork', 'dist') + os.pathsep + os.environ['PATH']

import mitsuba
from mitsuba.core import *
from mitsuba.render import SceneHandler, RenderQueue, RenderJob
import multiprocessing

fileResolver = Thread.getThread().getFileResolver()
fileResolver.appendPath('../test/assets/mitsuba-scenes/spot-studio/')

scheduler = Scheduler.getInstance()
# Start up the scheduling system with one worker per local core
for i in range(0, multiprocessing.cpu_count()):
    scheduler.registerWorker(LocalWorker(i, 'wrk%i' % i))
scheduler.start()

output_dir = "cow-fd"
os.makedirs(output_dir, exist_ok=True)
files = glob.glob(os.path.join(output_dir, "*"))
for f in files:
    os.remove(f)

raw_images = []

width = 512
height = 512
eps = 0.01
inv_eps = 1 / eps
nder = 10

for i in range(0, nder + 1):
    params = StringMap()
    params["width"] = str(width)
    params["height"] = str(height)
    params["key-x-offset"] = "0"
    params["key-y-offset"] = "0"
    params["key-z-offset"] = "0"
    params["fill-x-offset"] = "0"
    params["fill-y-offset"] = "0"
    params["fill-z-offset"] = "0"
    params["back-x-offset"] = "0"
    params["back-y-offset"] = "0"
    params["back-z-offset"] = "0"
    params["roughness"] = "0.6"

    if i == 1: params["key-x-offset"] = str(eps)
    elif i == 2: params["key-y-offset"] = str(eps)
    elif i == 3: params["key-z-offset"] = str(eps)
    elif i == 4: params["fill-x-offset"] = str(eps)
    elif i == 5: params["fill-y-offset"] = str(eps)
    elif i == 6: params["fill-z-offset"] = str(eps)
    elif i == 7: params["back-x-offset"] = str(eps)
    elif i == 8: params["back-y-offset"] = str(eps)
    elif i == 9: params["back-z-offset"] = str(eps)
    elif i == 10: params["roughness"] = str(0.6 + eps)
    
    scene = SceneHandler.loadScene(fileResolver.resolve("spot-studio-metal-fd.xml"), params)
    scene.setBlockSize(16)

    # Create a queue for tracking render jobs
    queue = RenderQueue()

    name = "fd-%i" % (i - 1) if i > 0 else "origin"
    scene.setDestinationFile(os.path.join(output_dir, name))
    job = RenderJob(name, scene, queue)
    job.start()

    queue.waitLeft(0)
    queue.join()

    raw_images.append(np.fromfile(os.path.join(output_dir, name) + "-raw.bin", dtype='float32'))

for i in range(0, nder):
    diff = inv_eps * raw_images[i + 1] - inv_eps * raw_images[0]
    img = np.reshape(diff, (width, height, 3))
    img = np.flip(img, 2) # correct rgb channel order
    cv2.imwrite(os.path.join(output_dir, "fd-%i-diff.exr") % i, img)