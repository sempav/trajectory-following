#!/usr/bin/env python2

import matplotlib, matplotlib.pyplot as plt
import argparse
from sys import argv
from math import pi

def main():
    if len(argv) < 2 or len(argv) > 3:
        print "Usage:", argv[0], "<dataX.txt> [<output_prefix>]"
        exit(0)

    if len(argv) == 3:
        output_prefix = argv[2]
    else:
        output_prefix = 'plot'

    f = open(argv[1], "r")
    num_bots = eval(f.readline())["num_bots"]
    for i in xrange(num_bots):
        d = eval(f.readline())
        print "Plotting", str(d["id"]) + "..."
        plot(argv[1], d["id"], output_prefix + '_' + d["id"] + ".png")


def plot(filename, id, output_filename):

    f = open(filename, "r")

    #plt.rcParams['legend.framealpha'] = 0.5

    num_bots = eval(f.readline())["num_bots"]
    title_data = None
    for i in xrange(num_bots):
        d = eval(f.readline())
        if d["id"] == id:
            title_data = d
    if title_data == None:
        raise RuntimeError, "id " + id + ": data not found in file header"

    time_data = []
    ex_data = []
    ey_data = []
    et_data = []
    real_ex_data = []
    real_ey_data = []
    real_et_data = []
    v_data = []
    omega_data = []
    for line in f:
        d = eval(line)
        if d["id"] != id:
            continue
        time_data.append(d["time"])
        ex_data.append(d["e_x"])
        ey_data.append(d["e_y"])
        et_data.append(d["e_theta"])
        real_ex_data.append(d["real_e_x"])
        real_ey_data.append(d["real_e_y"])
        real_et_data.append(d["real_e_theta"])
        v_data.append(d["v"])
        omega_data.append(d["omega"])

    legends = []

    fig = plt.figure()
    axes = plt.subplot(321)
    ex, = axes.plot(time_data, ex_data, '-', label=r'$e_x$')
    ey, = axes.plot(time_data, ey_data, '-', label=r'$e_y$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    legends.append(lgd)
    axes.set_ylim([-1.0, 1.0])

    axes = plt.subplot(323)
    real_ex, = axes.plot(time_data, real_ex_data, '-', label=r'real $e_x$')
    real_ey, = axes.plot(time_data, real_ey_data, '-', label=r'real $e_y$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    legends.append(lgd)
    axes.set_ylim([-1.0, 1.0])

    axes = plt.subplot(322)
    et, = axes.plot(time_data, et_data, 'r-', label=r'$e_\theta$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    legends.append(lgd)
    axes.set_ylim([-pi, pi])

    axes = plt.subplot(324)
    real_et, = axes.plot(time_data, real_et_data, 'r-', label=r'real $e_\theta$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    legends.append(lgd)
    axes.set_ylim([-pi, pi])

    axes = plt.subplot(325)
    ev, = axes.plot(time_data, v_data, 'g-', label=r'$v$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    legends.append(lgd)

    axes = plt.subplot(326)
    eomega, = axes.plot(time_data, omega_data, '-', label=r'$\omega$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    legends.append(lgd)

    #fig.tight_layout()

    #plt.rcParams['lines.linewidth'] = 1.0

    fig.subplots_adjust(hspace=0.3, wspace=1.0)

    #fig.suptitle("id " + str(id) + "\r" + \
    #             r"Noise: $\sigma=" + str(title_data['noise_sigma']) + '$ ' + \
    #             "\rRef. points = " + str(title_data['reference_points_cnt']) + \
    #             "\rdelay = " + str(title_data['trajectory_delay']))


    plt.savefig(output_filename, dpi=150, bbox_extra_artists=legends, bbox_inches='tight')
    #plt.show()


def parse_arguments():
    parser = argparse.ArgumentParser(description='')
    return parser.parse_args()


if __name__ == "__main__":
    main()
