from engine.bot import BOT_VEL_CAP, BOT_ACCEL_CAP, BOT_RADIUS
from engine.graphics import draw_directed_circle, BOT_COLOR
from engine.shapes import MockShape, Circle
from engine.vector import Point, normalize, length
from math import pi, copysign, sin, cos

class MockModel(object):
    def __init__(self, pos, dir, vel=0.0,
                       pos_fun=None,
                       max_vel=BOT_VEL_CAP,
                       max_accel=BOT_ACCEL_CAP,
                       radius=BOT_RADIUS,
                       collidable=False):
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
        if collidable:
            self.shape = Circle(self.pos, self.radius)
        else:
            self.shape = MockShape()
            # avoid AttributeErrors in position update code
            self.shape.center = self.pos


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
        self.shape.center = self.pos


    def draw(self, screen, field, collided, has_collided_before):
        draw_directed_circle(screen, field, BOT_COLOR,
                             self.pos,
                             self.radius, self.dir, 1 + collided * 4 + has_collided_before)
