from engine.vector import Vector

import numpy as np
from math import isnan


class Trajectory(object):
    """ Represents a continuous trajectory.
    
    Provides position, its first and second derivatives,
    and curvature as functions of time.
    """
    def __init__(self, x, y, dx, dy, ddx, ddy):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.d = lambda t: Vector(self.dx(t),  self.dy(t))
        self.ddx = ddx
        self.ddy = ddy
        self.dd = lambda t: Vector(self.ddx(t), self.ddy(t))


    @classmethod
    def from_poly(cls, x_poly, y_poly):
        x = np.poly1d(x_poly)
        y = np.poly1d(y_poly)
        dx = np.polyder(x)
        dy = np.polyder(y)
        ddx = np.polyder(dx)
        ddy = np.polyder(dy)
        return Trajectory(x=x, y=y,
                          dx=dx, dy=dy,
                          ddx=ddx, ddy=ddy)


    def curvature(self, t):
        k = ((self.dx(t) * self.ddy(t) - self.dy(t) * self.ddx(t)) /
             (self.dx(t)**2 + self.dy(t)**2)**1.5)
        if isnan(k):
            raise ValueError, "curvature is nan"
        return k
