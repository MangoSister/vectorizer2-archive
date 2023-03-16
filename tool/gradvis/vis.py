import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.axes_grid1 import make_axes_locatable
import argparse, os
from skimage.transform import resize
from exr_utils import load_exr
import os, sys

table_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "CubicL.txt")
cubicL = LinearSegmentedColormap.from_list("cubicL", np.loadtxt(table_path), N=256)

def remap(img):
    return np.multiply(np.sign(img), np.log1p(np.abs(100*img)))

def rgb2gray(img):
	return 0.2989*img[:, :, 0] + 0.5870*img[:, :, 1] + 0.1140*img[:, :, 2]

def main(argv):
	parser = argparse.ArgumentParser(description='Script for simple EXR utilities', epilog='Cheng Zhang (chengz20@uci.edu)')
	parser.add_argument('input', metavar='input', type=str, help='input file to convert to color map')
	parser.add_argument('-vmin', metavar='vmin', type=str, help='minimum value for color map')
	parser.add_argument('-vmax', metavar='vmax', type=str, help='maximum value for color map')
	parser.add_argument('-output', metavar='output', type=str, default='', help='output file path')
	parser.add_argument('-scalar', metavar='scalar', type=int, default=1, help='scalar for error image')
	parser.add_argument('-resize', metavar='resize', type=float, default=1.0, help='rescale scalar for error image')
	parser.add_argument('-legend', metavar='legend', type=int, default=1, help='add color bar')
	parser.add_argument('-dpi', metavar='dpi', type=float, default=100, help='output image dpi (when adding legend)')
	args = parser.parse_args(argv)

	img_input = rgb2gray(load_exr(args.input)) * args.scalar
	img_input = resize(img_input, (img_input.shape[0]*args.resize, img_input.shape[1]*args.resize), anti_aliasing=True)

	if args.legend != 0:
		ratio = img_input.shape[0]/img_input.shape[1]
		fig = plt.figure(figsize=(5, 5*ratio/1.2))
		#im = plt.imshow(remap(img_input), interpolation='bilinear', vmin=args.vmin, vmax=args.vmax, cmap=cubicL)
		im = plt.imshow(img_input, interpolation='bilinear', vmin=args.vmin, vmax=args.vmax, cmap=cubicL)
		plt.axis('off')
		plt.subplots_adjust(bottom=0.0, left=-0.05, top=1.0)
		cax = fig.add_axes([0.85, 0.03, 0.02, 0.94])
		plt.colorbar(im, cax=cax)
		if args.scalar == 1.0:
			plt.savefig(os.path.splitext(args.input)[0] + '.png', dpi=args.dpi)
		else:
			plt.savefig(os.path.splitext(args.input)[0] + 'x%d.png' % args.scalar, dpi=args.dpi)
	else:
		fig = plt.figure(figsize=(1,float(img_input.shape[0]) / float(img_input.shape[1])))
		ax = plt.Axes(fig, [0., 0., 1., 1.])
		ax.set_axis_off()
		fig.add_axes(ax)
		ax.imshow(img_input, interpolation='bilinear', vmin=args.vmin, vmax=args.vmax, cmap=cubicL)
		plt.savefig(os.path.splitext(args.input)[0] + '.png', dpi=img_input.shape[1])
	plt.close()

if __name__ == "__main__":
	main(sys.argv[1:])