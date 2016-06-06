from engine.vector import Point
from math import pi

class Config(object):
    def __init__(self):
        # engine/bot.py
        self.BOT_RADIUS = 0.3
        self.BOT_ACCEL_CAP = 5.0# * 1e5
        self.BOT_VEL_CAP = 2.0# * 1e5

        self.FOV_RADIUS = 3.5
        self.KNOW_BOT_POSITIONS = True

        # behaviors/follower.py
        self.POSITIONS_BUFFER_SIZE = 150
        self.SAMPLE_COUNT = 40
        self.MIN_DISTANCE_TO_LEADER = 2.2 * self.BOT_RADIUS
        self.DISABLE_CIRCLES = False
        self.DISABLE_FEEDBACK = False
        self.DISABLE_UKF = True

        # small circles:
        self.MIN_CIRCLE_CURVATURE = 8.0
        self.MAX_CURVATURE = self.MIN_CIRCLE_CURVATURE
        # keep curvature:
        #self.MIN_CIRCLE_CURVATURE = 0.0
        #self.MAX_CURVATURE = 100.0 # disabled

        self.TRAJECTORY_SEGMENT_COUNT = 10
        self.FOV_ANGLE = 0.25 * pi

        # engine/field.py
        self.FIELD_CENTERED = True

        # engine/graphics.py
        self.BACKGROUND_COLOR = 0,0,0
        self.GRID_COLOR = 50,50,50
        self.AXES_COLOR = 90,90,90
        self.OBSTACLE_COLOR = 200,200,200
        self.BOT_COLOR = 255,0,0
        self.TARGET_COLOR = 255,255,0
        self.SENSOR_COLOR = 50,50,155
        self.TRAJECTORY_POINTS_COLOR = 155,155,155
        self.USED_TRAJECTORY_POINTS_COLOR = 155,155,155
        self.NOISY_TRAJECTORY_POINTS_COLOR = 155,55,55
        self.APPROX_TRAJECTORY_COLOR = 200,200,0
        self.TARGET_POINT_COLOR = 200,0,200
        self.VISIBILITY_COLOR = 0,200,200

        self.TARGET_RADIUS = 5

        self.DRAW_COORD_GRID = True
        self.DRAW_SENSOR_RANGE = True
        self.DRAW_SENSOR_RAYS = False
        self.DRAW_VISIBILITY = True
        self.DRAW_VELOCITY = True
        self.DRAW_DIRECTION = False
        self.DRAW_APPROX_TRAJECTORY = True
        self.DRAW_REFERENCE_POS = True
        self.DRAW_DELAYED_LEADER_POS = False

        self.DISPLAYED_POINTS_COUNT = 10
        self.DISPLAYED_USED_POINTS_COUNT = 0

        # main.py
        self.FRAMERATE = 60
        self.FRAMES_PER_BOT_UPDATE = 1
        self.SEED = 23

        self.NUM_FOLLOWERS = 5

        self.DEFAULT_START_POS = Point(4.0, 0.0)

        self.MEASUREMENT_SIGMA = 0.05
        self.MOVEMENT_SIGMA = 0.5 * self.BOT_ACCEL_CAP / self.FRAMERATE

        self.MAX_INTERACTIVE_ROT_VEL = 5.0

        self.INTERACTIVE = False


    def read_from_file(self, filename):
        with open(filename, "r") as f:
            for line in f:
                if len(line.strip()) == 0 or line[0] == '#':
                    continue
                try:
                    exec(line.strip(), globals(), self.__dict__)
                except Exception, e:
                    print(str(e))

config = Config()
