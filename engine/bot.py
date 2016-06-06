from config import config

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
