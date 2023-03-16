import sys, os
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection, LineCollection

if len(sys.argv) < 2:
    print("Need an input file.")
    sys.exit()
filename = sys.argv[1]
data = open(filename, "rb").read()
start = 0
end = 8
(width, height) = struct.unpack("II", data[start:end])

start = end
end += (width*height)*4
pixels = struct.unpack("f"*(width*height), data[start:end])

start = end
end += 4
(occludee_vert_count,) = struct.unpack("I", data[start:end])

start = end
end += occludee_vert_count*3*4
occludee = struct.unpack("f"*occludee_vert_count*3, data[start:end])

start = end
end += 4
(occluder_vert_count,) = struct.unpack("I", data[start:end])

start = end
end += occluder_vert_count*3*4
occluder = struct.unpack("f"*occluder_vert_count*3, data[start:end])

#
fb = np.reshape(np.asarray(pixels), (height, width))
plt.imshow(fb, origin='lower', cmap='gray', vmin=0.0, vmax=1.0)

occludee = np.reshape(np.asarray(occludee), (occludee_vert_count, 3))
occludee[:,0:2] = (occludee[:, 0:2] + 1.0) * 0.5 * width - 0.5 # -0.5 for plt convention

occluder = np.reshape(np.asarray(occluder), (occluder_vert_count, 3))
occluder[:,0:2] = (occluder[:, 0:2] + 1.0) * 0.5 * height - 0.5 # -0.5 for plt convention

edges = []
for i in range(occludee_vert_count//3):
    for j in range(3):
        p = occludee[3*i+j]
        q = occludee[3*i+(j+1)%3]
        edges.append([[p[0], p[1]], [q[0], q[1]]])
plt.gca().add_collection(LineCollection(edges, linewidths=1.5, colors='g', linestyle='solid'))

edges = []
for i in range(occluder_vert_count//3):
    for j in range(3):
        p = occluder[3*i+j]
        q = occluder[3*i+(j+1)%3]
        edges.append([[p[0], p[1]], [q[0], q[1]]])
plt.gca().add_collection(LineCollection(edges, linewidths=1.5, colors='r', linestyle='solid'))

plt.show()

