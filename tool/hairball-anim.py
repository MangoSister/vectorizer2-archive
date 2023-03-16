import os, sys, shutil, glob

sys.path.append(os.path.join(sys.path[0], 'gradvis'))
import vis

sys.path.append(os.path.join(sys.path[0], 'mitsuba-fork', 'dist', 'python', '3.6'))
os.environ['PATH'] = os.path.join(sys.path[0], 'mitsuba-fork', 'dist') + os.pathsep + os.environ['PATH']

import mitsuba
from mitsuba.core import *
from mitsuba.render import SceneHandler, RenderQueue, RenderJob
import multiprocessing

logger = Thread.getThread().getLogger()
logger.setLogLevel(EError)
fileResolver = Thread.getThread().getFileResolver()
fileResolver.appendPath('../test/assets/mitsuba-scenes/hairball/')

# Optional: supply parameters that can be accessed
# by the scene (e.g. as $myParameter)
paramMap = StringMap()
paramMap["width"] = "720"
paramMap["height"] = "720"

scheduler = Scheduler.getInstance()
# Start up the scheduling system with one worker per local core
for i in range(0, multiprocessing.cpu_count()):
    scheduler.registerWorker(LocalWorker(i, 'wrk%i' % i))
scheduler.start()

output_dir = "hairball-anim"
prefix = "move-far"

os.makedirs(output_dir, exist_ok=True)
files = glob.glob(os.path.join(output_dir, "*"))
for f in files:
    os.remove(f)

# Create a queue for tracking render jobs
queue = RenderQueue()

# Move far
frame_rate = 30
video_length = 10
frameCount = frame_rate * video_length

z_start = -1.0
z_end = 3.0
for i in range(0, frameCount):
    scene = SceneHandler.loadScene(fileResolver.resolve("hairball-color-anim.xml"), paramMap)
    scene.setBlockSize(8)

    sensor = scene.getSensor()
    trans = Transform.lookAt(Point(0, 0, 0), Point(0, 0, -1), Vector(0, 1, 0))
    t = i / (frameCount - 1.0)
    offset = Transform.translate(Vector(0, 0, z_start * (1.0 - t) + z_end * t))
    trans = offset * trans
    sensor.setWorldTransform(trans)

    scene.setDestinationFile(os.path.join(output_dir, prefix+"-%03i" % i))
    job = RenderJob(os.path.join(prefix+"-%03i" % i), scene, queue)
    job.start()

    queue.waitLeft(0)
    queue.join()

grads = [img for img in os.listdir(output_dir) if img.endswith("-grad0.exr")]

vmin = '-25.0'
vmax = '25.0'
for name in grads:
    vis.main(["%s/%s" % (output_dir, name), '-legend', '0', '-vmin', vmin, '-vmax', vmax])