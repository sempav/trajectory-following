from engine.vector import Point
from math import pi, sin

def make_lissajous(period, scale_x, scale_y, a, b, delta = 0.5 * pi):
    w = 2 * pi / period
    return lambda time: Point(scale_x * sin(a * time * w + delta),
                              scale_y * sin(b * time * w))

def make_figure8():
    return make_lissajous(25.0, 4, 3, 1, 2)

def make_circle():
    return make_lissajous(20.0, 4, 4, 1, 1)

def make_ellipse():
    return make_lissajous(20.0, 6, 1.0, 1, 1)
