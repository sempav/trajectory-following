from engine.bot import BOT_VEL_CAP, BOT_ACCEL_CAP, BOT_RADIUS
from engine.vector import Point, normalize, length
from math import pi, copysign, sin, cos
from engine.graphics import draw_circle, draw_line, BOT_COLOR

class MockModel(object):
    def __init__(self, pos, dir, vel=0.0,
                       pos_fun=None,
                       max_vel=BOT_VEL_CAP,
                       max_accel=BOT_ACCEL_CAP,
                       radius=BOT_RADIUS):
        self.pos = Point(*pos)
        self.dir = normalize(dir)
        self.vel = vel
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.radius = radius
        self.width = 2.0 * radius
        self.max_rot_vel = 2 * max_vel / self.width
        self.time = 0
        self.pos_fun = pos_fun


    def update_vel(self, delta_time, desired_vel):
        pass


    def vel_fun(self, time, delta_time):
        return (self.pos_fun(time + 1.0 * delta_time) - 
                self.pos_fun(time - 1.0 * delta_time)) / (2.0 * delta_time)


    def update_state(self, delta_time):
        if self.pos_fun is not None:
            self.time += delta_time
            self.pos = self.pos_fun(self.time)
            v = self.vel_fun(self.time, delta_time)
            self.vel = length(v)
            try:
                self.dir = normalize(v)
            except ZeroDivisionError:
                # keep the previous value of dir
                pass
        else: # bot is user-controlled
            if abs(self.vel) > self.max_vel:
                self.vel = copysign(self.max_vel, self.vel)
            if self.vel < 0:
                self.vel = 0
            self.pos += self.dir * self.vel * delta_time


    def draw(self, screen, field):
        draw_circle(screen, field, BOT_COLOR,
                           self.pos,
                           self.radius, 1)
