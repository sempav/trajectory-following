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

    file_type = '.pdf' if args.pdf else '.png'
    lx,ly = extract_trajectory(args.start, args.end, info.bots['leader'].header, info.bots['leader'].data)
    if args.id == 'all':
        for bot in sorted(info.bots.values()):
            if (bot.header['id'] == 'leader'):
                continue
            print "Plotting", str(bot.header['id']) + "..."
            x,y = extract_trajectory(args.start, args.end, bot.header, bot.data)
            plot_trajectory(x, y, lx, ly, args, args.output_prefix + '_' + bot.header['id'] + file_type)
            plt.clf()
    else:
        x,y = extract_trajectory(args.start, args.end, info.bots[args.id].header,
                                                       info.bots[args.id].data)
        plot_trajectory(x, y, lx, ly, args, args.output_prefix + file_type)


def plot_trajectory(x, y, lx, ly, args, output_filename):
        artists = []
        axes = plt.subplot(111)
        traj = axes.plot(x, y, '-', label='follower')
        lead_traj = axes.plot(lx, ly, '--', label='leader')
        #real_e, = axes.plot(time_data, real_e_data, '-', label=r'$\Vert e\Vert$')
        axes.grid()
        #lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
        lgd = axes.legend(loc=args.loc, prop={'size':20})
        artists.append(lgd)
        plt.axis('equal')

        #fig.tight_layout()

        plt.rcParams['lines.linewidth'] = 1.0

        #fig.subplots_adjust(hspace=0.3, wspace=1.0)

        plt.savefig(output_filename,
                    dpi=150, bbox_extra_artists=artists,
                    bbox_inches='tight')
        #plt.show()


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


def extract_trajectory(start_time, end_time, header, data):

    #plt.rcParams['legend.framealpha'] = 0.5

    x_data = []
    y_data = []

    for d in data:
        if not(start_time <= d["time"] <= end_time):
            continue
        x_data.append(d["x"])
        y_data.append(d["y"])
    return x_data, y_data

    #artists = []

    #axes = plt.subplot(111)
    #traj = axes.plot(x_data, y_data, '-', label=
    ##real_e, = axes.plot(time_data, real_e_data, '-', label=r'$\Vert e\Vert$')
    #axes.grid()
    ##lgd = axes.legend(ncol=1, loc='center right', bbox_to_anchor=(-0.15, 0.5))
    #lgd = axes.legend(loc='upper left', prop={'size':20})
    #artists.append(lgd)

    #fig.tight_layout()

    #plt.rcParams['lines.linewidth'] = 1.0

    #fig.subplots_adjust(hspace=0.3, wspace=1.0)

    #plt.savefig(output_filename, dpi=150, bbox_extra_artists=artists,
    #            bbox_inches='tight')
    #plt.show()


def parse_arguments():
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('input', metavar='input', type=str,
                        help='File containing simulation data')
    parser.add_argument('output_prefix', type=str, nargs='?',
                        help='Output file prefix', default='plot_trajectory')
    parser.add_argument('--start', '-s', type=float,
                        default=0.0,
                        help='Start of the time interval')
    parser.add_argument('--end', '-e', type=float,
                        default=1e+5,
                        help='End of the time interval')
    parser.add_argument('--pdf', action='store_true', default=False)
    parser.add_argument('--id', type=str, default='all',
                        help='Robot id to plot data for')
    parser.add_argument('--loc', type=str,
                        default='best')
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_arguments())
