import enum


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
