import sys, os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import argparse
import re

curve_colors = ["#4169e1", "#FF6060", "#D500FF", "#85DC44", "#9B44DC", "#FEAF00"]

def plot_curves(all_curves, axis_name, labels, N, log_scale, tick, legend, outfile):
    # N = 0
    # for curve in all_curves:
    #     N = np.maximum(N, len(curve))

    num_iter = list(range(N))
    plt.rcParams["font.family"] = "serif"

    fig = plt.figure()

    handles = []
    legends = []

    ax1 = fig.add_subplot(111)
    ax1.set_xlabel('# Iterations', fontsize=24)
    ax1.set_xticks(np.arange(0, N+tick, tick))

    ax1.set_ylabel(axis_name, fontsize=24)
    ax1.tick_params(axis='both', which='major', labelsize=20)
    ax1.grid()
    if log_scale:
        ax1.set_yscale("log")
    for i, curve in enumerate(all_curves):
        d = N - len(curve)
        if d > 0:
            for _ in range(d):
                curve.append(curve[-1])

        curve_handle, = ax1.plot(num_iter, curve[:N], '-', c=curve_colors[i], linewidth=2)
        handles.append(curve_handle)
        legends.append(labels[i])

    if legend == 2:
        plt.legend(handles, legends, fontsize=20, bbox_to_anchor=(1.4, 1.0))
    elif legend == 1:
        plt.legend(handles, legends, fontsize=20, loc="upper right")

    plt.savefig(outfile, bbox_inches='tight', pad_inches=0)
    plt.close()

def read_progress(filename):
    img_rmse = []
    param_rmse = []
    with open(filename, newline='') as f:
        # reader = csv.reader(f, delimiter=' ')
        for row in f:
            row = row.replace(",", "")
            fields = re.split(r' ', row)
            img_rmse.append(float(fields[1]))
            param_rmse.append(float(fields[2]))
    return img_rmse, param_rmse


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Invutil plot.')
    parser.add_argument('-f', '--files', type=str, nargs='+', required=True)
    parser.add_argument('-o', '--output', type=str, required=True)
    parser.add_argument('-l', '--labels', type=str, nargs='+', required=True)
    parser.add_argument('-n', '--numsteps', type=int, required=True)
    parser.add_argument('--log_scale', type=int, default=0)
    parser.add_argument('--tick', type=int, default=20)
    parser.add_argument('--legend', type=int, default=0)
    parser.add_argument('--axis_name', type=int, default=1)


    args = parser.parse_args(sys.argv[1:])
    if len(args.files) != len(args.labels):
        raise RuntimeError("Mismatch file number / label number.")

    all_img_rmse = []
    all_param_rmse = []
    for f in args.files:
        img_rmse, param_rmse = read_progress(f)
        all_img_rmse.append(img_rmse)
        all_param_rmse.append(param_rmse)

    output = args.output
    N = args.numsteps
    log_scale = args.log_scale
    tick = args.tick
    legend = args.legend
    axis_name = args.axis_name

    plot_curves(all_img_rmse, "Image RMSE" if axis_name else None, args.labels, N, log_scale, tick, legend, output+"_image_rmse.pdf")
    plot_curves(all_param_rmse, "Param. RMSE" if axis_name else None, args.labels, N, log_scale, tick, legend, output+"_param_rmse.pdf")

    # plot gears scene param rmse.
    # target = [-0.2, -0.2, 0.2, -0.2, 0.2, 0.2, 0.2, 0.2, 0.2, -0.2, 0.2, -0.2, -0.2, -0.2, -0.2]
    # all_param_rmse = []
    # for filename in args.files:
    #     with open(filename, newline='') as f:
    #         param_rmse = []
    #         for row in f:
    #             row = row.replace(",", "")
    #             fields = re.split(r' ', row)[1:]
    #             rmse = 0.0
    #             for j in range(1, len(fields)):
    #                 rmse += (float(fields[j]) - target[j]) ** 2
    #             rmse /= (len(target) - 1)
    #             rmse = np.sqrt(rmse)
    #             param_rmse.append(rmse)
    #         all_param_rmse.append(param_rmse)
    # plot_curves(all_param_rmse, "Param. RMSE" if axis_name else None, args.labels, N, log_scale, tick, legend, output+"_param_rmse.pdf")
