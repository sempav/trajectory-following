from engine.bot import BOT_VEL_CAP, BOT_ACCEL_CAP, BOT_RADIUS
from engine.vector import Point, Vector, length, normalize, rotate, signed_angle
from math import pi, copysign
from engine.graphics import draw_circle, draw_line, draw_directed_circle, BOT_COLOR
from engine.shapes import Circle

MIN_ROTATION_ANGLE = 0.00 * pi
ROTATION_GAIN = 1.0 # !before changing check abs_vel *= cos(ang) line
MAX_VEL_ANGLE = pi / 3

class DifferentialModel(object):
    def __init__(self, pos, dir, vel = 0.0,
                       max_vel = BOT_VEL_CAP,
                       max_accel = BOT_ACCEL_CAP,
                       radius = BOT_RADIUS):
        self.pos = Point(*pos)
        self.dir = normalize(dir)
        self.lvel = vel
        self.rvel = vel
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.radius = radius
        self.width = 2.0 * radius
        self.max_rot_vel = 2 * max_vel / self.width

        self.shape = Circle(self.pos, self.radius)


    @property
    def vel(self):
        return 0.5 * (self.lvel + self.rvel)

    @property
    def rot_vel(self):
        return (self.rvel - self.lvel) / self.width


    def update_vel(self, delta_time, desired_vel):
        v = float(desired_vel.v)
        omega = float(desired_vel.omega)

        sigma = max(v/self.max_vel, omega/self.max_rot_vel, 1.0)
        if sigma != 1.0:
            if v/self.max_vel > omega/self.max_rot_vel:
                v = copysign(self.max_vel, v)
                omega /= sigma
            else: # omega/self.max_rot_vel > v/self.max_vel
                v /= sigma
                omega = copysign(self.max_rot_vel, omega)

        # these are the velocities wheels would get
        # if they didn't have to accelerate smoothly
        target_lvel = v - 0.5 * self.width * omega
        target_rvel = v + 0.5 * self.width * omega
        if abs(self.lvel - target_lvel) < delta_time * BOT_ACCEL_CAP:
            self.lvel = target_lvel
        else:
            self.lvel += copysign(BOT_ACCEL_CAP, target_lvel - self.lvel) * delta_time
        if abs(self.rvel - target_rvel) < delta_time * BOT_ACCEL_CAP:
            self.rvel = target_rvel
        else:
            self.rvel += copysign(BOT_ACCEL_CAP, target_rvel - self.rvel) * delta_time
        # cap velocity
        v = max(abs(self.lvel), abs(self.rvel))
        if v > self.max_vel:
            self.lvel *= self.max_vel / v
            self.rvel *= self.max_vel / v


    def update_state(self, delta_time):
        self.pos += delta_time * self.vel * self.dir
        self.dir = rotate(self.dir, delta_time * self.rot_vel)
        self.shape.center = self.pos


    def draw(self, screen, field, collided, has_collided_before):
        draw_directed_circle(screen, field, BOT_COLOR,
                             self.shape.center, self.shape.radius,
                             self.dir, 1 + collided * 4 + has_collided_before)
