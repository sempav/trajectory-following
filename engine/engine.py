import enum
from vector import dist


class Movement(enum.Enum):
    Accel = 1
    Speed = 2
    Dir = 3


class Engine(object):

    def __init__(self, field):
        self.field = field

        self.bots = []
        self.obstacles = []
        self.targets = []

        self._current_time = 0
        self._delta_time = 0
        self._last_bot_update_time = 0
        self._last_physics_update_time = 0


    @property
    def time(self):
        return self._current_time


    @time.setter
    def time(self, t):
        self._delta_time = t - self._current_time
        self._current_time = t


    @property
    def delta_time(self):
        return _delta_time


    @property
    def time_since_last_bot_update(self):
        return self.time - self._last_bot_update_time


    @property
    def time_since_last_physics_update(self):
        return self.time - self._last_physics_update_time


    def update_bots(self):
        for bot in self.bots:
            bot.virtual.sync_to_real(bot.real)

        for bot in self.bots:
            bot.virtual.update_vel(self.bots, self.obstacles, self.targets, self)

        self._last_bot_update_time = self.time


    def update_physics(self, delta_time):
        for bot in self.bots:
            bot.real.update_vel(delta_time, bot.virtual.desired_vel)

        for bot in self.bots:
            bot.real.update_state(delta_time)

        self._last_physics_update_time = self.time


    def check_collisions(self, include_bot_collisions=True):
        collided_set = set()
        for bot in self.bots:
            collided_flag = bot in collided_set
            if collided_flag:
                continue
            if include_bot_collisions:
                # check for collisions with other bots
                for other in self.bots:
                    if bot is not other: # don't check for collisions with self
                        if bot.real.shape.intersect(other.real.shape) is not None:
                            collided_set.add(bot)
                            collided_set.add(other)
                            collided_flag = True
                            break
                if collided_flag:
                    continue
            # check for collisions with obstacles
            for obstacle in self.obstacles:
                if obstacle.intersect(bot.real.shape) is not None:
                    collided_set.add(bot)
                    collided_flag = True
                    break
            bot.collided = collided_flag
            bot.has_collided_before = bot.has_collided_before or collided_flag
        return collided_set
