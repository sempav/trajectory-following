#!/usr/bin/env python2

import matplotlib, matplotlib.pyplot as plt
import argparse
from collections import namedtuple
from sys import argv
from math import pi, sqrt


SimulationInfo = namedtuple('SimulationInfo', 'header, bots')
BotInfo = namedtuple('BotInfo', 'header, data')

def main():
    if len(argv) < 2 or len(argv) > 3:
        print "Usage:", argv[0], "<dataX.txt> [<output_prefix>]"
        exit(0)

    if len(argv) == 3:
        output_prefix = argv[2]
    else:
        output_prefix = 'plot'

    info = read_info(argv[1])
    num_bots = info.header['num_bots']
    for bot in sorted(info.bots.values()):
        print "Plotting", str(bot.header['id']) + "..."
        plot(bot.header, bot.data, output_prefix + '_' + bot.header["id"] + ".png")


def read_info(filename):
    f = open(filename, "r")
    header = eval(f.readline())
    bots = {}
    num_bots = header['num_bots']
    for i in xrange(num_bots):
        bot_header = eval(f.readline())
        bots[bot_header['id']] = BotInfo(header=bot_header, data = [])

    for line in f:
        d = eval(line)
        bots[d['id']].data.append(d)

    return SimulationInfo(header=header, bots=bots)


def plot(header, data, output_filename):

    #plt.rcParams['legend.framealpha'] = 0.5

    time_data = []
    ex_data = []
    ey_data = []
    et_data = []
    real_e_data = []
    real_et_data = []
    v_data = []
    omega_data = []
    approx_e_data = []
    for d in data:
        time_data.append(d["time"])
        ex_data.append(d["e_x"])
        ey_data.append(d["e_y"])
        et_data.append(d["e_theta"])
        real_ex = d["real_e_x"]
        real_ey = d["real_e_y"]
        real_e_data.append(sqrt(real_ex**2 + real_ey**2))
        real_et_data.append(d["real_e_theta"])
        v_data.append(d["v"])
        omega_data.append(d["omega"])
        approx_e_data.append(sqrt(d["approx_e_x"]**2 +
                                  d["approx_e_y"]**2))

    artists = []

    fig = plt.figure()
    axes = plt.subplot(421)
    ex, = axes.plot(time_data, ex_data, '-', label=r'$e_x$')
    ey, = axes.plot(time_data, ey_data, '-', label=r'$e_y$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    axes = plt.subplot(423)
    real_e, = axes.plot(time_data, real_e_data, '-', label=r'real $\Vert e\Vert$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    axes = plt.subplot(422)
    et, = axes.plot(time_data, et_data, 'r-', label=r'$e_\theta$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)
    #axes.set_ylim([-pi, pi])

    axes = plt.subplot(424)
    real_et, = axes.plot(time_data, real_et_data, 'r-', label=r'real $e_\theta$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)
    #axes.set_ylim([-pi, pi])

    axes = plt.subplot(425)
    ev, = axes.plot(time_data, v_data, 'g-', label=r'$v$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    axes = plt.subplot(426)
    eomega, = axes.plot(time_data, omega_data, '-', label=r'$\omega$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    axes = plt.subplot(427)
    approx_e, = axes.plot(time_data, approx_e_data, '-', label=r'$e_{approx}$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    #fig.tight_layout()

    #plt.rcParams['lines.linewidth'] = 1.0

    fig.subplots_adjust(hspace=0.3, wspace=1.0)

    suptitle = plt.suptitle("id " + str(header['id']) + ", " + \
         r"noise: $\sigma = " + str(header['noise_sigma']) + '$ ' + \
         ", ref. points = " + str(header['reference_points_cnt']) + \
         ", delay = " + str(header['trajectory_delay']) + "s", y=0.05)
    artists.append(suptitle)


    plt.savefig(output_filename, dpi=150, bbox_extra_artists=artists,
                bbox_inches='tight')
    #plt.show()


def parse_arguments():
    parser = argparse.ArgumentParser(description='')
    return parser.parse_args()


if __name__ == "__main__":
    main()
