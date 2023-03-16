import os, sys, shutil, glob

sys.path.append(os.path.join(sys.path[0], 'gradvis'))
import vis

sys.path.append(os.path.join(sys.path[0], 'mitsuba-fork', 'dist', 'python', '3.6'))
os.environ['PATH'] = os.path.join(sys.path[0], 'mitsuba-fork', 'dist') + os.pathsep + os.environ['PATH']

import mitsuba
from mitsuba.core import *
from mitsuba.render import SceneHandler, RenderQueue, RenderJob
import multiprocessing

fileResolver = Thread.getThread().getFileResolver()
fileResolver.appendPath('../test/assets/mitsuba-scenes/tree/')

scheduler = Scheduler.getInstance()
# Start up the scheduling system with one worker per local core
for i in range(0, multiprocessing.cpu_count()):
    scheduler.registerWorker(LocalWorker(i, 'wrk%i' % i))
scheduler.start()

output_dir = "tree-anim"
os.makedirs(output_dir, exist_ok=True)
files = glob.glob(os.path.join(output_dir, "*"))
for f in files:
    os.remove(f)

frame_rate = 30
video_length = 10
frameCount = frame_rate * video_length

paramMap = StringMap()
paramMap["width"] = "1280"
paramMap["height"] = "720"

for i in range(0, frameCount):
    yaw = float(i) / float(frameCount) * 360.0
    paramMap["yaw"] = str(yaw)
    scene = SceneHandler.loadScene(fileResolver.resolve("tree-rotate-z-anim.xml"), paramMap)
    scene.setBlockSize(8)

    # Create a queue for tracking render jobs
    queue = RenderQueue()

    output_prefix = "tree-anim"
    scene.setDestinationFile(os.path.join(output_dir, output_prefix+"-%03i" % i))
    job = RenderJob(os.path.join(output_prefix+"-%03i" % i), scene, queue)
    job.start()

    queue.waitLeft(0)
    queue.join()

grads = [img for img in os.listdir(output_dir) if img.endswith("-grad0.exr")]

vmin = '-3.0'
vmax = '3.0'
for name in grads:
    vis.main(["%s/%s" % (output_dir, name), '-legend', '0', '-vmin', vmin, '-vmax', vmax])