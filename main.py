#! /usr/bin/env python2


import argparse
import os
import pygame

import engine
import behaviors
import models
from engine.vector import Point, Vector, normalize
from engine.graphics import Graphics
from engine.field import Field
from engine.bot import Bot, BOT_RADIUS
from obstacle_maps import maps
from mock_trajectories import *


FRAMERATE = 60
FRAMES_PER_BOT_UPDATE = 1

NUM_FOLLOWERS = 1


DEFAULT_START_POS = Point(4.0, 0.0)

NOISE_SIGMA = 0.01


def create_log_file():
    fname = "data"
    fnumber = 1
    # find first number X such that file "dataX.txt" doesn't exist
    while os.access(fname + str(fnumber) + ".txt", os.F_OK):
        fnumber += 1
    return open(fname + str(fnumber) + ".txt", "w")


def reset(eng, obstacle_map, model=models.DifferentialModel, interactive=False, log_data=False):
    log_file = None
    if log_data:
        log_file = create_log_file()
        log_dict = {"num_bots": NUM_FOLLOWERS}
        print >> log_file, log_dict

    eng.bots = []
    eng.obstacles = []
    eng.targets = []

    if interactive:
        pos_fun = None
        start_pos = DEFAULT_START_POS
        start_dir = Vector(0.0, 1.0)
        eng.bots.append(Bot(models.MockModel(pos=start_pos, dir=start_dir, vel=0.0,
                                             pos_fun=pos_fun, collidable=True),
                            behavior=behaviors.Leader()))
    else:
        pos_fun=make_lissajous(15.0, 4, 0.5, 1, 3)
        #pos_fun = make_lissajous(35.0, 4, 3, 1, 4)
        eng.bots.append(Bot(models.MockModel(pos=(0.0, 0.0), dir=(1.0, 0.0), vel=0.0, pos_fun=pos_fun, collidable=True),
                            behavior=behaviors.Leader()))
        start_pos = eng.bots[0].real.pos_fun(0.0)
        start_dir = normalize(eng.bots[0].real.vel_fun(0.0, 0.01))
        eng.bots[0].real.pos = start_pos
        eng.bots[0].real.dir = start_dir

    displacement = -start_dir * 2.1 * BOT_RADIUS
    for i in xrange(NUM_FOLLOWERS):
        eng.bots.append(Bot(model(pos=start_pos + (i + 1) * displacement,
                                  dir=start_dir, vel=0.0),
                            behavior=behaviors.Follower(g=30, zeta=0.9,
                                                        leader=eng.bots[i],
                                                        trajectory_delay=1.0,
                                                        update_delta_t=0.01,
                                                        orig_leader=eng.bots[0],
                                                        orig_leader_delay=1.0 * (i + 1),
                                                        noise_sigma=NOISE_SIGMA,
                                                        log_file=log_file,
                                                        id="%02d" % (i + 1))))

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
    size = (info.current_w - 10, info.current_h - 100)
    field = Field((0.01 * size[0], 0.01 * size[1]), size)
    graph = Graphics(field, size)
    eng = engine.Engine(field)

    cur_group = True
    cur_obstacle_map = 0
    cur_movement = engine.Movement.Speed
    cur_model = models.DifferentialModel
    reset(eng, obstacle_map=cur_obstacle_map, model=cur_model,
          interactive=args.interactive, log_data=args.log_data)

    finished = False
    clock = pygame.time.Clock()
    iter_counter = 1

    paused = False

    pressed_keys = set()

    collision_has_occurred = False

    while not finished:
        eng.field.resize_to_contain(bot.real.pos for bot in eng.bots)

        real_delta_time = 0.001 * clock.tick(FRAMERATE)
        delta_time = 1.0 / FRAMERATE
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                finished = True
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pass
            elif event.type == pygame.KEYUP:
                if event.key in pressed_keys:
                    pressed_keys.remove(event.key)
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
                elif event.key == pygame.K_ESCAPE:
                    finished = True

                if reset_needed:
                    reset(eng, obstacle_map=cur_obstacle_map, model=cur_model,
                          interactive=args.interactive, log_data=args.log_data)
                    collision_has_occurred = False

        if pygame.K_LEFT in pressed_keys:
            try:
                max_rot_vel = 2 * (eng.bots[0].real.max_vel - eng.bots[0].real.vel) / (eng.bots[0].real.width * eng.bots[0].real.vel)
                #max_rot_vel = 2.0
                eng.bots[0].real.dir = engine.vector.rotate(eng.bots[0].real.dir, max_rot_vel * delta_time)
            except ZeroDivisionError:
                pass
        if pygame.K_RIGHT in pressed_keys:
            try:
                max_rot_vel = 2 * (eng.bots[0].real.max_vel - eng.bots[0].real.vel) / (eng.bots[0].real.width * eng.bots[0].real.vel)
                #max_rot_vel = 2.0
                eng.bots[0].real.dir = engine.vector.rotate(eng.bots[0].real.dir, -max_rot_vel * delta_time)
            except ZeroDivisionError:
                pass
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

        collided_set = eng.check_collisions()

        graph.render(bots = eng.bots,
                     obstacles = eng.obstacles,
                     targets = eng.targets)

        text = ("Time: %.3f" % eng.time) + ("s, FPS: %.2f" % (1.0 / real_delta_time))
        if collided_set: # a collision occurred
            text += ". COLLISION"
            collision_has_occurred = True
        if collision_has_occurred:
            text += ". A collision has been recorded"
        ren = font.render(text, 0, (255, 255, 255))
        text_size = font.size(text)
        graph.screen.blit(ren, (30, graph.size[1] - text_size[1]))
        pygame.display.flip()

    pygame.quit()


def parse_arguments():
    parser = argparse.ArgumentParser(description='')

    parser.add_argument('--log-data', '--log', action='store_true', default=False)
    parser.add_argument('--interactive', '--int', action='store_true', default=False)

    return parser.parse_args()


if __name__ == "__main__":
    main(parse_arguments())
