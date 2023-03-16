import sys, os
import cv2
import numpy as np
import shutil
import argparse

def imgdiff(img1, img2, output, scale, abs, gray_scale):
    res = scale * img1 - scale * img2
    if args.abs > 0:
        res = np.abs(res)
    if args.gray_scale:
        res = np.sum(res, axis=2)
    if output.endswith(".exr"):
        cv2.imwrite(output, res.astype("float32"))
    elif output.endswith(".png"):
        png = np.clip(np.power(res, 1.0 / 2.2) * 255, 0, 255)
        cv2.imwrite(output, png.astype('uint8'))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Simple image diff')
    parser.add_argument('-i', '--images', type=str, nargs=2, required=True)
    parser.add_argument('-o', '--output', type=str, required=True)
    parser.add_argument('-a', '--abs', type=int, default=0)
    parser.add_argument('-s', '--scale', type=float, default=1.0)
    parser.add_argument('-g', '--gray_scale', type=int, default=0)

    args = parser.parse_args(sys.argv[1:])
    if os.path.isfile(args.images[0]):
        img1 = cv2.imread(args.images[0], cv2.IMREAD_UNCHANGED)
        img2 = cv2.imread(args.images[1], cv2.IMREAD_UNCHANGED)
        imgdiff(img1, img2, args.output, args.scale, args.abs, args.gray_scale)
    elif os.path.isdir(args.images[0]):
        if os.path.exists(args.output) and os.path.isdir(args.output):
            shutil.rmtree(args.output, ignore_errors=True)
        os.makedirs(args.output, exist_ok=True)
        img2 = cv2.imread(args.images[1], cv2.IMREAD_UNCHANGED)
        for f in os.listdir(args.images[0]):
            img = cv2.imread(os.path.join(args.images[0], f), cv2.IMREAD_UNCHANGED)
            output = os.path.join(args.output, os.path.splitext(f)[0]+".png")
            imgdiff(img, img2, output, args.scale, args.abs, args.gray_scale)