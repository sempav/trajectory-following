BOT_RADIUS = 0.3
BOT_ACCEL_CAP = 5.0# * 1e5
BOT_VEL_CAP = 2.0# * 1e5

MAX_SENSING_DISTANCE = 5 * BOT_RADIUS
KNOW_BOT_POSITIONS = True


class Bot(object):
    def __init__(self, model, behavior):
        self.virtual = behavior
        self.real = model
        self.virtual.sync_to_real(self.real)

        self.collided = False
        self.has_collided_before = False

    def draw(self, screen, field):
        self.real.draw(screen, field, self.collided, self.has_collided_before)
        self.virtual.draw(screen, field)
