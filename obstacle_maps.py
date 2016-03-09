from engine.vector import Point
from engine import obstacle

maps = [[] for i in xrange(10)]

maps[1] = [obstacle.create_obstacle_polygon([Point(1.5, 1.5),
                                             Point(1.5, 2.5),
                                             Point(2.5, 2.5),
                                             Point(2.5, 1.5)]),
           obstacle.create_obstacle_polygon([Point(1.5, -1.5),
                                             Point(1.5, -2.5),
                                             Point(2.5, -2.5),
                                             Point(2.5, -1.5)]),
           obstacle.create_obstacle_polygon([Point(-1.5, -1.5),
                                             Point(-1.5, -2.5),
                                             Point(-2.5, -2.5),
                                             Point(-2.5, -1.5)]),
           obstacle.create_obstacle_polygon([Point(-1.5, 1.5),
                                             Point(-1.5, 2.5),
                                             Point(-2.5, 2.5),
                                             Point(-2.5, 1.5)])]

maps[2] = [obstacle.create_obstacle_circle(Point(1.25, 0.5), 1.1),
           obstacle.create_obstacle_circle(Point(-1.25, 0.0), 0.2)]

maps[3] = [obstacle.create_obstacle_circle(Point(3 * x, 2 * y), 0.2)
           for x in xrange(-1, 2) for y in xrange(-1, 2)]

maps[4] = [obstacle.create_obstacle_polygon([Point(0.0, 0.0),
                                             Point(3.0, 1.5),
                                             Point(3.0, -1.5)])]

maps[5] = [obstacle.create_obstacle_polygon([Point(0.0, -3.0),
                                              Point(3.0, -3.0),
                                              Point(3.0,  3.0),
                                              Point(0.0,  3.0),
                                              Point(0.0,  2.0),
                                              Point(2.0,  2.0),
                                              Point(2.0, -2.0),
                                              Point(0.0, -2.0)])]

