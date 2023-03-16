import os
import sys
import numpy as np
import cv2
import shutil
import argparse
import re

def exr2png(input_path, output_path, expo=0.0, gamma=2.2):
    exr = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)
    png = np.clip(np.power(exr * (2.0 ** expo), 1.0 / gamma) * 255, 0, 255)
    cv2.imwrite(output_path, png.astype('uint8'))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='PNG to MP4.')
    parser.add_argument('-i', '--input_dir', type=str, required=True)
    parser.add_argument('-o', '--output', type=str, required=True)
    parser.add_argument('-p', '--prefix', type=str, required=True)
    parser.add_argument('-x', '--width', type=int, required=True)
    parser.add_argument('-y', '--height', type=int, required=True)
    parser.add_argument('-r', '--framerate', type=int, required=True)
    parser.add_argument('--nested_dir', type=int, default=0)

    args = parser.parse_args(sys.argv[1:])
    input_dir = args.input_dir
    prefix = args.prefix

    tmp = "png2mp4_tmp"
    if os.path.exists(tmp) and os.path.isdir(tmp):
        shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp, exist_ok=True)

    img_pattern = re.compile(r"{}[0-9]+.(exr|png)".format(prefix))

    if args.nested_dir:
        subdir_pattern = re.compile("{}[0-9]+".format(prefix))
        for f in os.listdir(input_dir):
            subdir = os.path.join(input_dir, f)
            if os.path.isdir(subdir) and subdir_pattern.match(f):
                for ff in os.listdir(subdir):
                    if img_pattern.match(ff):
                        src = os.path.join(subdir, ff)
                        dst = os.path.join(tmp, os.path.splitext(ff)[0]+".png")
                        if ff.endswith(".exr"):
                            exr2png(src, dst)
                        elif ff.endswith(".png"):
                            shutil.copyfile(src, dst)

    else:
        for f in os.listdir(input_dir):
            if img_pattern.match(f):
                src = os.path.join(input_dir, f)
                dst = os.path.join(tmp, os.path.splitext(f)[0]+".png")
                if f.endswith(".exr"):
                    exr2png(src, dst)
                elif f.endswith(".png"):
                    shutil.copyfile(src, dst)

    cmd = "ffmpeg -f image2 "
    cmd += "-r {} ".format(args.framerate)
    cmd += "-s {}x{} ".format(args.width, args.height)
    cmd += "-i {} ".format(os.path.join(tmp, prefix + "%d.png"))
    cmd += "-vcodec libx264 -crf 23 -pix_fmt yuv420p {}.mp4".format(args.output)
    os.system(cmd)

    shutil.rmtree(tmp, ignore_errors=True)