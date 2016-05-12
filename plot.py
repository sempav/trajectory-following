#!/usr/bin/env python2

import matplotlib, matplotlib.pyplot as plt
import argparse
from collections import namedtuple
from sys import argv
from math import pi, sqrt


SimulationInfo = namedtuple('SimulationInfo', 'header, bots')
BotInfo = namedtuple('BotInfo', 'header, data')

def main(args):
    info = read_info(args.input)
    num_bots = info.header['num_bots']
    for bot in sorted(info.bots.values()):
        if bot.header["id"] == 'leader':
            continue
        print "Plotting", str(bot.header['id']) + "..."
        plot(args.start, args.end, bot.header, bot.data, args.output_prefix + '_' + bot.header["id"] + ".png")


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


def plot(start_time, end_time, header, data, output_filename):

    #plt.rcParams['legend.framealpha'] = 0.5

    time_data = []
    e_data = []
    et_data = []
    real_e_data = []
    real_et_data = []
    v_data = []
    omega_data = []
    approx_e_data = []

    invisible_regions = []
    last_invis_start = 0.0
    now_invisible = False
    for d in data:
        if not(start_time <= d["time"] <= end_time):
            continue
        time_data.append(d["time"] - start_time)
        ex = d["e_x"]
        ey = d["e_y"]
        e_data.append(sqrt(ex**2 + ey**2))
        et_data.append(d["e_theta"])
        real_ex = d["real_e_x"]
        real_ey = d["real_e_y"]
        real_e_data.append(sqrt(real_ex**2 + real_ey**2))
        real_et_data.append(d["real_e_theta"])
        v_data.append(d["v"])
        omega_data.append(d["omega"])
        approx_e_data.append(sqrt(d["approx_e_x"]**2 +
                                  d["approx_e_y"]**2))
        cur_visible = d["leader_is_visible"]
        if cur_visible:
            if now_invisible:
                invisible_regions.append((last_invis_start, time_data[-1]))
                now_invisible = False
        else: # not cur_visible
            if not now_invisible:
                now_invisible = True
                last_invis_start = time_data[-1]

    artists = []

    fig = plt.figure()
    axes = plt.subplot(321)
    e, = axes.plot(time_data, e_data, '-', label=r'$\Vert e\Vert$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    axes = plt.subplot(323)
    real_e, = axes.plot(time_data, real_e_data, '-', label=r'total $\Vert e\Vert$')
    axes.grid()
    #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    axes = plt.subplot(322)
    et, = axes.plot(time_data, et_data, 'r-', label=r'$e_\theta$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)
    #axes.set_ylim([-pi, pi])

    axes = plt.subplot(324)
    real_et, = axes.plot(time_data, real_et_data, 'r-', label=r'total $e_\theta$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)
    #axes.set_ylim([-pi, pi])

    #axes = plt.subplot(325)
    #approx_e, = axes.plot(time_data, approx_e_data, '-', label=r'$e_{approx}$')
    #axes.grid()
    #for st, fn in invisible_regions:
    #    axes.axvspan(st, fn, color='red', alpha=0.3)
    #lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    #artists.append(lgd)

    axes = plt.subplot(325)
    omega, = axes.plot(time_data, v_data, 'r-', label=r'$v$')
    axes.grid()
    lgd = axes.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    artists.append(lgd)

    axes = plt.subplot(326)
    omega, = axes.plot(time_data, omega_data, 'r-', label=r'$\omega$')
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
    parser.add_argument('input', metavar='input', type=str,
                        help='File containing simulation data')
    parser.add_argument('output_prefix', type=str, nargs='?',
                        help='Output file prefix', default='plot')
    parser.add_argument('--start', '-s', type=float,
                        default=0.0,
                        help='Start of the time interval')
    parser.add_argument('--end', '-e', type=float,
                        default=1e+5,
                        help='End of the time interval')
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_arguments())
