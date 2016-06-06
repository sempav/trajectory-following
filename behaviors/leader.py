from base import BehaviorBase
from config import config

class Leader(BehaviorBase):
    def __init__(self, id="leader", log_file=None):
        super(Leader, self).__init__()
        self.radius = config.BOT_RADIUS
        self.id = id
        self.log_file = log_file
        if self.log_file is not None:
            log_dict = {"id": self.id}
            print >> self.log_file, log_dict


    def calc_desired_velocity(self, bots, obstacles, targets, engine):
        # Not used
        if self.log_file is not None:
            log_dict = {"id": self.id,
                        "time": engine.time,
                        "delta_time": engine.time_since_last_bot_update,
                        "x": self.pos.x,
                        "y": self.pos.y}
            print >> self.log_file, log_dict


    def draw(self, screen, field):
        pass
