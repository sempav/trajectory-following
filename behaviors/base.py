from collections import namedtuple

Instr = namedtuple('Instr', 'v, omega')

class BehaviorBase(object):
    """
    Base class for movement logic.

    Any concrete subclass must define calc_desired_velocity.
    """

    def __init__(self):
        self.desired_vel = Instr(0, 0)

    def calc_desired_velocity(self, bots, obstacles, targets):
        raise NotImplemented


    def update_vel(self, bots, obstacles, targets, eng):
        self.desired_vel = self.calc_desired_velocity(bots, obstacles, targets, eng)


    def sync_to_real(self, real):
        self.pos = real.pos
        self.real_vel = real.vel
        self.real_dir = real.dir
