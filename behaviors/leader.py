from base import BehaviorBase
from engine.bot import BOT_RADIUS

class Leader(BehaviorBase):
    def __init__(self):
        self.radius = BOT_RADIUS


    def calc_desired_velocity(self, bots, obstacles, targets, eng):
        # Not used
        pass


    def draw(self, screen, field):
        pass
