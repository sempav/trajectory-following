import pygame
from pygame import gfxdraw

from vector import Point, Vector, signed_angle, rotate
from math import degrees, pi, sin, cos

GRID_COLOR = 50,50,50
AXES_COLOR = 90,90,90
OBSTACLE_COLOR = 200,200,200
BOT_COLOR = 255,0,0
TARGET_COLOR = 255,255,0
SENSOR_COLOR = 50,50,155
TRAJECTORY_POINTS_COLOR = 155,155,155
USED_TRAJECTORY_POINTS_COLOR = 155,155,155
APPROX_TRAJECTORY_COLOR = 200,200,0
TARGET_POINT_COLOR = 200,0,200
VISIBILITY_COLOR = 0,200,200

TARGET_RADIUS = 5

DRAW_COORD_GRID = True
DRAW_SENSOR_RANGE = True
DRAW_SENSOR_RAYS = False
DRAW_VISIBILITY = True
DRAW_VELOCITY = True
DRAW_DIRECTION = False
DRAW_APPROX_TRAJECTORY = True
DRAW_REFERENCE_POS = True
DRAW_DELAYED_LEADER_POS = False


def draw_circle(screen, field, color, center, radius, thickness=2):
    pygame.draw.circle(screen, color,
                       field.fit_on_screen(center),
                       max(2 * thickness, field.scale(radius)),
                       thickness)


def draw_line(screen, field, color, a, b, thickness=2):
    pygame.draw.line(screen, color,
                     field.fit_on_screen(a),
                     field.fit_on_screen(b),
                     thickness)


def draw_arc(screen, field, color, center, radius,
                            start_angle, end_angle,
                            thickness=2):
    p = field.fit_on_screen(center + Vector(-radius, radius))
    x, y = field.fit_on_screen(center)
    r = pygame.Rect(p.x, p.y,
                    field.scale(2 * radius),
                    field.scale(2 * radius))
    #pygame.draw.ellipse(screen, color, r, thickness)
    #pygame.draw.arc(screen, color, r, start_angle, end_angle, thickness)
    gfxdraw.arc(screen, x, y, int(field.scale(radius)),
                       int(degrees(-end_angle)), int(degrees(-start_angle)),
                       color)


def draw_directed_circle(screen, field, color, center, radius, dir, thickness=2):
    draw_circle(screen, field, color,
                       center,
                       radius, thickness)
    top_angle = 40.0 * pi / 180.0
    x = radius * sin(top_angle)
    y = radius * cos(top_angle)
    ang = signed_angle(Vector(0.0, 1.0), dir)
    pa = center + rotate(Vector(-x, -y), ang)
    pb = center + rotate(Vector( x, -y), ang)
    pc = center + rotate(Vector(0.0, radius), ang)
    draw_line(screen, field, color, pa, pb, thickness)
    draw_line(screen, field, color, pa, pc, thickness)
    draw_line(screen, field, color, pb, pc, thickness)


class Graphics(object):
    def __init__(self, field, size = (1024, 768)):
        self.field = field
        self.size = size
        self.screen = pygame.display.set_mode(size)
        pygame.display.set_caption("potential")


    def draw_coordinate_grid(self, num = (10,10)):
        for x in xrange(int(self.field.left), 1 + int(self.field.right), 1):
            pygame.draw.line(self.screen, GRID_COLOR if x != 0 else AXES_COLOR,
                             self.field.fit_on_screen(Point(x, self.field.bottom)),
                             self.field.fit_on_screen(Point(x, self.field.top)))
        for y in xrange(int(self.field.bottom), 1 + int(self.field.top), 1):
            pygame.draw.line(self.screen, GRID_COLOR if y != 0 else AXES_COLOR,
                             self.field.fit_on_screen(Point(self.field.left,  y)),
                             self.field.fit_on_screen(Point(self.field.right, y)))


    def render(self, bots, obstacles = [], targets = []):
        self.screen.fill((0, 0, 0))

        if DRAW_COORD_GRID:
            self.draw_coordinate_grid()

        for obstacle in obstacles:
            obstacle.draw(self.screen, self.field)

        for bot in bots:
            bot.draw(self.screen, self.field)

        for target in targets:
            pygame.draw.circle(self.screen, TARGET_COLOR,
                               self.field.fit_on_screen(target),
                               TARGET_RADIUS, 1)
        
        if DRAW_VELOCITY:
            for bot in bots:
                draw_line(self.screen, self.field, (0, 115, 0),
                          bot.real.pos, 
                          bot.real.pos + bot.real.vel * bot.real.dir,
                          1)
        if DRAW_DIRECTION:
            for bot in bots:
                draw_line(self.screen, self.field, (115, 115, 0),
                          bot.real.pos,
                          bot.real.pos + bot.real.dir,
                          1)
