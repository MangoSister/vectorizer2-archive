import sys, os
import csv
from enum import Enum
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection, LineCollection
import struct
from dataclasses import dataclass

canvasBgColor = (0.0, 0.0, 0.1)
emptyTriangleId = 4294967295
emptyColor = (0.1, 0.0, 0.0)
def randomColorFromId(n):
    if (n == emptyTriangleId):
        return emptyColor # Make black color unique to emptyTriangleId for easier debugging
    n = ((n ^ n >> 15) * 2246822519) & 0xffffffff
    n = ((n ^ n >> 13) * 3266489917) & 0xffffffff
    n = (n ^ n >> 16) >> 8
    color = [u / 255. for u in n.to_bytes(3, 'big')] + [1.0]
    color[0] = color[0] * 0.8 + 0.2
    color[1] = color[1] * 0.8 + 0.2
    color[2] = color[2] * 0.8 + 0.2
    return color


if len(sys.argv) < 2:
    print("Need an input file.")
    sys.exit()
filename = sys.argv[1]
overlay = False
if '--overlay' in sys.argv:
    overlay = True
showEmpty = False
if '--showEmpty' in sys.argv:
    showEmpty = True
faceOnly = False
if '--faceOnly' in sys.argv:
    faceOnly = True
@dataclass(eq=True, frozen=True)
class Vec2:
    x: float
    y: float
@dataclass(eq=True, frozen=True)
class Edge:
    v0: Vec2
    v1: Vec2

faceIndices = []
faceVerts = []
vertSet = set()
edgeSet = set()
with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        # Skip empty regions
        if (not showEmpty) and (int(row[0]) == emptyTriangleId):
            continue
        faceIndices.append(int(row[0]))
        verts = []
        count = int((len(row) - 1) / 2)
        for i in range(count):
            verts.append([float(row[1 + 2 * i]), float(row[1 + 2 * i + 1])])
            vertSet.add(Vec2(float(row[1 + 2 * i]), float(row[1 + 2 * i + 1])))
        for i in range(count):
            v0 = Vec2(verts[i][0], verts[i][1])
            v1 = Vec2(verts[(i + 1) % count][0], verts[(i + 1) % count][1])
            e0 = Edge(v0, v1)
            e1 = Edge(v1, v0)
            if (e0 not in edgeSet) and (e1 not in edgeSet):
                edgeSet.add(e0)
        faceVerts.append(verts)

# Vertices
vertList = []
for p in vertSet:
    vertList.append([p.x, p.y])
# Edges
edgeList = []
for e in edgeSet:
    edgeList.append([[e.v0.x, e.v0.y], [e.v1.x, e.v1.y]])

# Faces
patches = []
for i in range(len(faceIndices)):
    color = randomColorFromId(faceIndices[i])
    poly = Polygon(faceVerts[i], True, color=color)
    patches.append(poly)
pc = PatchCollection(patches, match_original=True, picker=2)

def onPick(event):
    if type(event.artist) == PatchCollection:
        print("Patch picking:")
        for i in event.ind:
            color = event.artist.get_facecolor()[i]
            print("index: {}, tri: {}".format(i,faceIndices[i]))

if not overlay:
    if faceOnly:
        fig, ax1 = plt.subplots(1, 1)
        ax1.add_collection(pc)
        ax1.set_facecolor(canvasBgColor)
        ax1.axis('square')
        ax1.set_xlim([-1, 1])
        ax1.set_ylim([-1, 1])

        fig.suptitle(filename, fontsize=16)
        pid = fig.canvas.mpl_connect('pick_event', onPick)

        plt.show()
    else:
        fig, (ax1, ax2) = plt.subplots(1, 2)
        ax1.scatter(*zip(*vertList))
        coll = LineCollection(edgeList, colors='k', linestyle='solid')
        ax1.add_collection(coll)
        ax1.set_title('Subd View', fontsize=16)
        ax1.axis('equal')

        ax2.add_collection(pc)
        ax2.set_facecolor(canvasBgColor)
        ax2.set_xlim([-1, 1])
        ax2.set_ylim([-1, 1])
        ax2.set_title('Mesh View', fontsize=16)
        ax2.axis('equal')
        fig.suptitle(filename, fontsize=16)

        pid = fig.canvas.mpl_connect('pick_event', onPick)
        plt.show()
else:
    fig, (ax1) = plt.subplots(1, 1)
    ax1.set_facecolor(canvasBgColor)
    ax1.add_collection(pc)
    coll = LineCollection(edgeList, colors='k', linestyle='solid')
    ax1.add_collection(coll)
    ax1.scatter(*zip(*vertList), c=(1.0,1.0,0,1), s=10, zorder=2)

    ax1.axis('square')
    ax1.set_xlim([-1, 1])
    ax1.set_ylim([-1, 1])
    #fig.suptitle(filename, fontsize=16)
    #ax1.set_title('Overlay View', fontsize=16)

    pid = fig.canvas.mpl_connect('pick_event', onPick)

    ax1.set_axis_off()
    plt.margins(0,0)
    plt.savefig(("%s.png") % os.path.splitext(filename)[0], dpi=500, bbox_inches = 'tight', pad_inches = 0)

    plt.show()
