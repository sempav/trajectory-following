from base import BehaviorBase
from engine.bot import BOT_RADIUS, BOT_VEL_CAP, BOT_ACCEL_CAP
from engine.vector import Vector, length, normalize, dist
from math import sin, cos, pi, sqrt
from collections import namedtuple

class Leader(BehaviorBase):
    def __init__(self):
        self.radius = BOT_RADIUS


    def calc_desired_velocity(self, bots, obstacles, targets, eng):
        # Not used
        pass


    def draw(self, screen, field):
        pass
