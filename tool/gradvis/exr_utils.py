import numpy as np
import sys
import os
import platform
from scipy import interpolate
import scipy.ndimage.filters as sfilters

#import pyexr


def load_exr(filename):
    #if platform.system() == "Windows":
    if True:
        import cv2
        bgr = np.array(cv2.imread(filename, cv2.IMREAD_UNCHANGED), dtype=np.float)
        if bgr.ndim >= 3:
            img = bgr[...,::-1]
        else:
            img = bgr
    else:
        from mitsuba.core import Vector2i
        from mitsuba.core import Bitmap
        from mitsuba.core import FileStream
        img = Bitmap(filename)
        img = np.array(img.buffer(), dtype=np.float)
    return img


def save_exr(filename, img):
    #if platform.system() == "Windows":
    if True:
        import cv2
        rgb = img.astype(np.float32)
        if rgb.ndim >= 3:
            img = rgb[...,::-1]
        else:
            img = rgb
        cv2.imwrite(filename, img)
    else:
        from mitsuba.core import Vector2i
        from mitsuba.core import Bitmap
        from mitsuba.core import FileStream

        img = np.array(img, dtype=np.float32)
        size = Vector2i(img.shape[1], img.shape[0])
        if img.ndim == 2:
            bitmap = Bitmap(Bitmap.ELuminance, Bitmap.EFloat32, size)
        else:
            bitmap = Bitmap(Bitmap.ERGB, Bitmap.EFloat32, size)
        bitmap.fromByteArray(bytearray(img.tostring()))
        fp = FileStream(filename, FileStream.ETruncReadWrite)
        bitmap.write(Bitmap.EOpenEXR, fp)
        fp.close()


def tone_mapping(hdr, expo=0.0, gamma=2.2):
    #tonemap = cv2.createTonemap(gamma)
    #res = tonemap.process(hdr)
    hdr = hdr * (2.0 ** expo)
    res = np.power(hdr, 1.0 / gamma)
    ldr = np.clip(res * 255, 0, 255).astype("uint8")
    return ldr


def upsample(img, reso, interp_kind='linear'):
    cur_reso = img.shape
    y = np.linspace(0, cur_reso[0] - 1, cur_reso[0])
    x = np.linspace(0, cur_reso[1] - 1, cur_reso[1])

    new_y = np.linspace(0, cur_reso[0] - 1, reso[0])
    new_x = np.linspace(0, cur_reso[1] - 1, reso[1])

    new_img = np.zeros((reso[0], reso[1], cur_reso[2]))
    for k in range(cur_reso[2]):
        f = interpolate.interp2d(x, y, img[:, :, k], kind=interp_kind)
        new_img[:, :, k] = f(new_x, new_y)
    return new_img


def bilinear_upsample(img, reso):
    cur_reso = img.shape
    new_img = np.zeros((reso[0], reso[1], cur_reso[2]))
    for r in range(reso[0]):
        for c in range(reso[1]):
            y = float(r) / (reso[0] - 1.0) * (cur_reso[0] - 1.0)
            x = float(c) / (reso[1] - 1.0) * (cur_reso[1] - 1.0)
            ri = np.clip(int(np.floor(y)), 0, cur_reso[0] - 2)
            ci = np.clip(int(np.floor(x)), 0, cur_reso[1] - 2)

            for dr in range(2):
                v = y - ri
                wv = abs(1.0 - dr - v)
                for dc in range(2):
                    u = x - ci
                    wu = abs(1.0 - dc - u)
                    new_img[r, c, :] += img[ri + dr, ci + dc, :] * (wu * wv)

    return new_img


def downsample_box(img, n):
    cur_reso = img.shape
    assert (cur_reso[0] % n == 0)
    assert (cur_reso[1] % n == 0)
    img_out = np.reshape(img, (cur_reso[0] // n, n, cur_reso[1] // n, n, cur_reso[2]))
    img_out = np.mean(img_out, axis=(1, 3))
    return img_out


def smooth_gaussian(img, sigma, mode='wrap'):
    img_out = sfilters.gaussian_filter(img, sigma, mode=mode)
    return img_out


if __name__ == "__main__":
    img = load_exr("../data/analyze_beckmann/base_heightmap_4.exr")
    save_exr("test.exr", img)
