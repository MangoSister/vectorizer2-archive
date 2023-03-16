import os, sys, shutil, glob
import cv2
import numpy as np

sys.path.append(os.path.join(sys.path[0], 'gradvis'))
import exr_utils

def main(argv):
    os.makedirs(os.path.join(argv[0], "pngs"), exist_ok=True)
    files = glob.glob(os.path.join(os.path.join(argv[0], "pngs"), "*"))
    for f in files:
        os.remove(f)

    files = glob.glob(os.path.join(argv[0], "*.exr"))
    for f in files:
       ldr = exr_utils.tone_mapping(exr_utils.load_exr(f))
       ldr = np.flip(ldr, 2) # Correct rgb channel order

       name = os.path.splitext(os.path.basename(f))[0]
       cv2.imwrite(os.path.join(argv[0], "pngs", name + ".png"), ldr)

if __name__ == "__main__":
	main(sys.argv[1:])