#! /usr/bin/env python2


import argparse
import pygame
import os
from sys import argv

import engine
import behaviors
import models
from engine.vector import Point, Vector, normalize
from engine.graphics import Graphics
from engine.field import Field
from engine.bot import Bot
from obstacle_maps import maps
from mock_trajectories import *


FRAMERATE = 60
FRAMES_PER_BOT_UPDATE = 1

NUM_FOLLOWERS = 10


def create_dump_file():
    fname = "data"
    fnumber = 1
    # find first number X such that file "dataX.txt" doesn't exist
    while os.access(fname + str(fnumber) + ".txt", os.F_OK):
        fnumber += 1
    return open(fname + str(fnumber) + ".txt", "w")


def reset(eng, obstacle_map, model=models.DifferentialModel, interactive=False, dump_data=False):
    dump_file = None
    if dump_data:
        dump_file = create_dump_file()
        dump_dict = {"num_bots": NUM_FOLLOWERS}
        print >> dump_file, dump_dict

    eng.bots = []
    eng.obstacles = []
    eng.targets = []

    if interactive:
        pos_fun=None
        start_pos = Point(0.0, 0.0)
        start_dir = Vector(1.0, 0.0)
        eng.bots.append(Bot(models.MockModel(pos=start_pos, dir=start_dir, vel=0.0, pos_fun=pos_fun),
                            behavior=behaviors.Leader()))
    else:
        #pos_fun=make_figure8()
        pos_fun = make_lissajous(35.0, 4, 3, 1, 4)
        eng.bots.append(Bot(models.MockModel(pos=(0.0, 0.0), dir=(1.0, 0.0), vel=0.0, pos_fun=pos_fun),
                            behavior=behaviors.Leader()))
        start_pos = eng.bots[0].real.pos_fun(0)
        start_dir = normalize(eng.bots[0].real.vel_fun(0, 0.01))

    for i in xrange(NUM_FOLLOWERS):
        eng.bots.append(Bot(model(pos=start_pos, dir=start_dir, vel=0.0),
                        behavior=behaviors.Follower(g=30, zeta=0.9,
                                                    leader=eng.bots[i],
                                                    trajectory_delay=1.0,
                                                    orig_leader=eng.bots[0],
                                                    orig_leader_delay=1.0 * (i + 1),
                                                    noise_sigma=0.0,
                                                    dump_file=dump_file,
                                                    id="follower %02d" % (i + 1))))

    eng.obstacles = maps[obstacle_map][:]

    eng.time = 0


def show_state(movement):
    if movement == engine.Movement.Accel:
        return "Acceleration mode"
    elif movement == engine.Movement.Speed:
        return "Velocity mode"
    else:
        return "Direction mode"


def main(args):
    # This seems to set initial window position on Windows
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (1,20)

    pygame.init()

    #font = pygame.font.Font(None, 30)
    font = pygame.font.SysFont("Arial", 25)

    #pygame.display.init()
    info = pygame.display.Info()
    size = (info.current_w - 1, info.current_h - 20)
    field = Field((0.01 * size[0], 0.01 * size[1]), size)
    graph = Graphics(field, size)
    eng = engine.Engine(field)

    cur_group = True
    cur_obstacle_map = 0
    cur_movement = engine.Movement.Speed
    cur_model = models.DifferentialModel
    reset(eng, obstacle_map=cur_obstacle_map, model=cur_model,
          interactive=args.interactive, dump_data=args.dump_data)

    finished = False
    clock = pygame.time.Clock()
    iter_counter = 1

    paused = False

    pressed_keys = set()

    while not finished:
        cleft = field.left
        cright = field.right
        ctop = field.top
        cbottom = field.bottom
        for bot in eng.bots:
            cleft = min(cleft, bot.real.pos.x)
            cright = max(cright, bot.real.pos.x)
            ctop = max(ctop, bot.real.pos.y)
            cbottom = min(cbottom, bot.real.pos.y)
        scale_w = (cright - cleft) / field.width
        scale_h = (ctop - cbottom) / field.height
        scale = max(scale_w, scale_h)
        field.width *= scale
        field.height *= scale
        field.left *= scale
        field.right *= scale
        field.top *= scale
        field.bottom *= scale
        #field.width = field.right - field.left
        #field.height = field.top - field.bottom

        delta_time = 0.001 * clock.tick(FRAMERATE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                finished = True
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pass
            elif event.type == pygame.KEYUP:
                event.key in pressed_keys and pressed_keys.remove(event.key)
            elif event.type == pygame.KEYDOWN:
                recognized_key = False
                reset_needed = False
                pressed_keys.add(event.key)
                if event.key == pygame.K_z:
                    cur_model = models.HolonomicModel
                    recognized_key = True
                    reset_needed = True
                    paused = False
                elif event.key == pygame.K_x:
                    cur_model = models.DifferentialModel
                    recognized_key = True
                    reset_needed = True
                    paused = False
                elif event.key == pygame.K_p:
                    paused = not paused
                elif event.key == pygame.K_r:
                    reset_needed = True
                elif event.key >= pygame.K_1 and event.key <= pygame.K_9:
                    cur_obstacle_map = event.key - pygame.K_1
                    reset_needed = True

                if reset_needed:
                    reset(eng, obstacle_map=cur_obstacle_map, model=cur_model,
                          interactive=args.interactive, dump_data=args.dump_data)

        if pygame.K_LEFT in pressed_keys:
            eng.bots[0].real.dir = engine.vector.rotate(eng.bots[0].real.dir, 1 * delta_time)
        if pygame.K_RIGHT in pressed_keys:
            eng.bots[0].real.dir = engine.vector.rotate(eng.bots[0].real.dir, -1 * delta_time)
        if pygame.K_UP in pressed_keys:
            eng.bots[0].real.vel += 1 * eng.bots[0].real.max_accel * delta_time
        if pygame.K_DOWN in pressed_keys:
            eng.bots[0].real.vel -= 1 * eng.bots[0].real.max_accel * delta_time

        if paused:
            continue

        eng.time += delta_time
        #eng.time = 0.001 * pygame.time.get_ticks() - time_start

        iter_counter += 1
        if iter_counter % FRAMES_PER_BOT_UPDATE == 0:
            eng.update_bots()
        eng.update_physics(delta_time)

        graph.render(bots = eng.bots,
                     obstacles = eng.obstacles,
                     targets = eng.targets)

        text = "Time: " + str(eng.time) + "s"
        ren = font.render(text, 0, (255, 255, 255))
        text_size = font.size(text)
        graph.screen.blit(ren, (30, graph.size[1] - text_size[1]))
        pygame.display.flip()

    pygame.quit()


def parse_arguments():
    parser = argparse.ArgumentParser(description='')

    parser.add_argument('--dump-data', '--dump', action='store_true', default=False)
    parser.add_argument('--interactive', '--int', action='store_true', default=False)

    return parser.parse_args()


if __name__ == "__main__":
    main(parse_arguments())
