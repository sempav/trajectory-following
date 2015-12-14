from base import BehaviorBase
from engine.bot import BOT_RADIUS, BOT_VEL_CAP, BOT_ACCEL_CAP, Bot
from engine.vector import Vector, Point, length, normalize, dist, rotate, cross, dot
from engine.graphics import draw_circle, draw_line, draw_arc, \
                            draw_directed_circle, \
                            APPROX_TRAJECTORY_COLOR, \
                            TARGET_POINT_COLOR, \
                            SENSOR_COLOR, \
                            VISIBILITY_COLOR, \
                            TRAJECTORY_POINTS_COLOR, \
                            USED_TRAJECTORY_POINTS_COLOR, \
                            DRAW_APPROX_TRAJECTORY, \
                            DRAW_REFERENCE_POS, \
                            DRAW_SENSOR_RANGE, \
                            DRAW_VISIBILITY, \
                            DRAW_DELAYED_LEADER_POS
from engine.shapes import Ray, first_intersection
from ring_buffer import RingBuffer, get_interval

from collections import namedtuple
from math import sin, cos, pi, atan2, sqrt, copysign, isnan
from random import gauss
import numpy as np

TRAJECTORY_SIZE = 100
SAMPLE_COUNT = 40
MIN_DISTANCE_TO_LEADER = 0.1

TRAJECTORY_SEGMENT_COUNT = 10
DISPLAYED_POINTS_COUNT = 0
DISPLAYED_USED_POINTS_COUNT = 0

Instr = namedtuple('Instr', 'v, omega')
TrajectoryPoint = namedtuple('TrajectoryPoint', 'time, pos')
LeaderState = namedtuple('LeaderState', 'time, pos, theta')


def lerp(a, b, coeff):
    return a + coeff * (b - a)


def lerp_angles(a, b, coeff):
    a = a % (2 * pi)
    b = b % (2 * pi)
    if b < a:
        a, b = b, a
    if b - a > pi:
        a, b = b - 2 * pi, a
    return a + coeff * (b - a)


class Follower(BehaviorBase):
    def __init__(self, g, zeta,
                 leader, trajectory_delay=2.0,
                 orig_leader=None, orig_leader_delay=None,
                 noise_sigma=0.0, dump_file=None,
                 visibility_fov=0.5 * pi, visibility_radius=None,
                 id=None):

        """
        Construct a follower Behavior.

        leader is the Bot to be followed
        g > 0 is a tuning parameter
        zeta in (0, 1) is a damping coefficient
        trajectory_delay is the time interval between leader and follower
        """

        assert(isinstance(leader, Bot))
        assert(isinstance(orig_leader, Bot))

        self.radius = BOT_RADIUS
        self.state = 1
        self.leader = leader
        self.orig_leader = orig_leader
        self.trajectory_delay = trajectory_delay

        assert g > 0, "Follower: g parameter must be positive"
        self.g = g
        assert 0 < zeta < 1, "Follower: zeta parameter must be in (0, 1)"
        self.zeta = zeta

        # trajectory stores TRAJECTORY_SIZE tuples;
        # tuple's first field is time, second is the
        # corresponding Point on the trajectory
        self.trajectory = RingBuffer(TRAJECTORY_SIZE)

        # orig_leader_states stores leader's precise state;
        # used to plot errors
        self.orig_leader_states = []

        self.update_interval = trajectory_delay / (TRAJECTORY_SIZE - SAMPLE_COUNT // 2)
        self.last_update_time = 0.0

        self.noise_sigma = noise_sigma

        self.dump_file = dump_file

        self.visibility_fov = visibility_fov
        if visibility_radius is None:
            visibility_radius = 2.0 * trajectory_delay * BOT_VEL_CAP
        self.visibility_radius = visibility_radius

        self.id = id;

        if orig_leader_delay is None:
            orig_leader_delay = trajectory_delay
        self.orig_leader_delay = orig_leader_delay


        if self.dump_file is not None:
            dump_dict = {"id": self.id,
                         "g": self.g,
                         "zeta": self.zeta,
                         "noise_sigma": self.noise_sigma,
                         "reference_points_cnt": SAMPLE_COUNT,
                         "trajectory_delay": trajectory_delay}
            print >> self.dump_file, dump_dict


    def point_in_fov(self, p):
        if length(p - self.pos) > self.visibility_radius:
            return False
        d = p - self.pos
        ang = atan2(cross(self.real_dir, d), dot(self.real_dir, d))
        return -0.5 * self.visibility_fov < ang < 0.5 * self.visibility_fov


    def visible_point(self, p, obstacles):
        if not self.point_in_fov(p):
            return False
        try:
            ray = Ray(self.pos, p - self.pos)
        except ZeroDivisionError:
            ray = Ray(self.pos, Vector(1.0, 0.0))
        i = first_intersection(ray, obstacles)
        return (i is None) or (length(i - self.pos) > length(p - self.pos))


    def calc_desired_velocity(self, bots, obstacles, targets, engine):
        # update trajectory
        if engine.time - self.last_update_time > self.update_interval:
            if self.visible_point(self.leader.real.pos, obstacles):
                self.leader_is_visible = True

                noisy_pos = self.leader.real.pos
                noisy_pos += Vector(gauss(0.0, self.noise_sigma),
                                    gauss(0.0, self.noise_sigma))
                self.leader_noisy_pos = noisy_pos
                self.trajectory.append(TrajectoryPoint(engine.time, noisy_pos))
                self.last_update_time = engine.time
            else:
                self.leader_is_visible = False

            orig_leader_theta = atan2(self.orig_leader.real.dir.y,
                                      self.orig_leader.real.dir.x)
            self.orig_leader_states.append(LeaderState(time=engine.time,
                                                       pos=self.orig_leader.real.pos,
                                                       theta=orig_leader_theta))

        #arr = [self.trajectory[x] for x in xrange(SAMPLE_COUNT)]
        t = engine.time - self.trajectory_delay
        arr = get_interval(self.trajectory, t, SAMPLE_COUNT)
        self.traj_interval = arr
        if len(arr) == 0:
            return Instr(0.0, 0.0)
        if self.leader_is_visible and length(self.pos - self.leader_noisy_pos) < MIN_DISTANCE_TO_LEADER:
            return Instr(0.0, 0.0)


        x_pos = np.array([el.pos.x for el in arr])
        y_pos = np.array([el.pos.y for el in arr])
        times = np.array([el.time   for el in arr])

        # calculate quadratic approximation of the reference trajectory
        x_poly = np.polyfit(times, x_pos, deg=2)
        y_poly = np.polyfit(times, y_pos, deg=2)
        x_approx = np.poly1d(x_poly)
        y_approx = np.poly1d(y_poly)
        self.x_approx = x_approx
        self.y_approx = y_approx
        self.t_st = times[0]
        self.t_fn = max(times[-1], t)

        # t = engine.time - self.trajectory_delay

        # pick time value from the middle of the sample
        # convert from np.float64 to float to avoid implicit casts to ndarrays
        #t = float(times[len(times) / 2])
        self.target_point = Point(x_approx(t), y_approx(t))
        # these two values are often used in the following formulas
        x02t_x1 = 2 * x_poly[0] * t + x_poly[1]
        y02t_y1 = 2 * y_poly[0] * t + y_poly[1]

        # x_r, y_r and theta_r denote the reference robot's state
        x_r = x_approx(t)
        y_r = y_approx(t)
        # remember that atan2 already adds/subtracts pi as needed!
        theta_r = atan2(y02t_y1, x02t_x1)

        # calculate the feed-forward velocities
        v_ff = sqrt(x02t_x1**2 + y02t_y1**2)
        omega_ff = (x02t_x1 * 2 * y_poly[0] - y02t_y1 * 2 * x_poly[0]) / (x02t_x1**2 + y02t_y1**2)

        if isnan(v_ff):
            v_ff = 0.0
        if isnan(omega_ff):
            omega_ff = 0.0

        # cur_x, cur_y and cur_theta form the current state vector
        cur_x = self.pos.x
        cur_y = self.pos.y
        cur_theta = atan2(self.real_dir.y, self.real_dir.x)

        # calculate error in the global (fixed) reference frame
        delta_x = x_r - cur_x
        delta_y = y_r - cur_y
        delta_theta = theta_r - cur_theta

        # translate error into the follower's reference frame
        e_x =  cos(cur_theta) * delta_x + sin(cur_theta) * delta_y
        e_y = -sin(cur_theta) * delta_x + cos(cur_theta) * delta_y
        e_theta = delta_theta % (2 * pi)
        if e_theta > pi:
            e_theta -= 2 * pi
        #e_theta = delta_theta

        # calculate gains k_x, k_y, k_theta
        omega_n = sqrt(omega_ff**2 + self.g * v_ff**2)
        k_x = 2 * self.zeta * omega_n
        k_y = self.g
        k_theta = 2 * self.zeta * omega_n

        # calculate control velocities
        v = v_ff * cos(e_theta) + k_x * e_x
        se = sin(e_theta) / e_theta if e_theta != 0 else 1.0
        omega = omega_ff + k_y * v_ff * se * e_y + k_theta * e_theta

        #v = v_ff
        #omega = omega_ff

        orig_t = engine.time - self.orig_leader_delay
        orig_arr = get_interval(self.orig_leader_states, orig_t, 3)
        real_e_x = 0.0
        real_e_y = 0.0
        real_e_theta = 0.0
        if len(orig_arr) >= 2:
            a = orig_arr[-2]
            b = orig_arr[-1]
            # lerp nearest states to get orig_leader's state at moment t
            coeff = (orig_t - a.time) / (b.time - a.time)
            leader_pos = lerp(a.pos, b.pos, coeff)
            leader_theta = lerp_angles(a.theta, b.theta, coeff)
            self.orig_leader_pos = leader_pos
            self.orig_leader_theta = leader_theta
            real_delta_x = leader_pos.x - cur_x
            real_delta_y = leader_pos.y - cur_y
            real_delta_theta = leader_theta - cur_theta
            real_e_x =  cos(cur_theta) * real_delta_x + sin(cur_theta) * real_delta_y
            real_e_y = -sin(cur_theta) * real_delta_x + cos(cur_theta) * real_delta_y
            real_e_theta = real_delta_theta % (2 * pi)
            if real_e_theta > pi:
                real_e_theta -= 2 * pi;

        #if abs(real_e_theta) > 0.5:
        #    print str(self.id) + ":", cur_theta, "ref:", self.orig_leader_theta, "err:", real_e_theta

        if self.dump_file is not None:
            dump_dict = {"id": self.id,
                         "time": engine.time,
                         "delta_time": engine.time_since_last_bot_update,
                         "v": v,
                         "omega": omega,
                         "v_ff": v_ff,
                         "omega_ff": omega_ff,
                         "e_x": e_x,
                         "e_y": e_y,
                         "e_theta": e_theta,
                         "real_e_x": real_e_x,
                         "real_e_y": real_e_y,
                         "real_e_theta": real_e_theta}
            print >> self.dump_file, dump_dict

        return Instr(v,  omega)


    def draw(self, screen, field):
        if DISPLAYED_POINTS_COUNT > 0:
            k = TRAJECTORY_SIZE / DISPLAYED_POINTS_COUNT
            for index, (time, point) in enumerate(self.trajectory):
                if index % k == 0:
                    draw_circle(screen, field, TRAJECTORY_POINTS_COLOR, point, 0.03, 1)

        if DISPLAYED_USED_POINTS_COUNT > 0:
            k = len(self.traj_interval) / DISPLAYED_USED_POINTS_COUNT
            for index, (time, point) in enumerate(self.traj_interval):
                if index % k == 0:
                    draw_circle(screen, field, USED_TRAJECTORY_POINTS_COLOR, point, 0.03, 1)

        if DRAW_DELAYED_LEADER_POS:
            try:
                orig_leader_dir = Vector(cos(self.orig_leader_theta),
                                         sin(self.orig_leader_theta))
                draw_directed_circle(screen, field, (0, 255, 0),
                                     self.orig_leader_pos, 0.2,
                                     orig_leader_dir, 1)
            except AttributeError:
                pass

        if DRAW_SENSOR_RANGE:
            ang = atan2(self.real_dir.y, self.real_dir.x)
            draw_arc(screen, field, SENSOR_COLOR, self.pos, self.visibility_radius,
                                    ang - 0.5 * self.visibility_fov,
                                    ang + 0.5 * self.visibility_fov,
                                    1)
            draw_line(screen, field, SENSOR_COLOR,
                      self.pos,
                      self.pos + rotate(self.real_dir * self.visibility_radius,
                                        0.5 * self.visibility_fov),
                      1)
            draw_line(screen, field, SENSOR_COLOR,
                      self.pos,
                      self.pos + rotate(self.real_dir * self.visibility_radius,
                                        -0.5 * self.visibility_fov),
                      1)

        try:
            if DRAW_VISIBILITY and self.leader_is_visible:
                draw_circle(screen, field, VISIBILITY_COLOR, self.leader.real.pos,
                            0.5 * BOT_RADIUS)
        except AttributeError:
            pass

        try:
            if DRAW_REFERENCE_POS:
                draw_circle(screen, field, TARGET_POINT_COLOR, self.target_point, 0.2)

            if DRAW_APPROX_TRAJECTORY:
                #if len(self.traj_interval) > 1:
                #    p2 = Point(self.traj_interval[0].pos.x,
                #               self.traj_interval[0].pos.y)
                #    for t, p in self.traj_interval:
                #        draw_line(screen, field, APPROX_TRAJECTORY_COLOR, p, p2)
                #        p2 = p

                step = 0.1 * (self.t_fn - self.t_st)
                for t in (self.t_st + k * step for k in xrange(TRAJECTORY_SEGMENT_COUNT)):
                    p = Point(self.x_approx(t),
                              self.y_approx(t))
                    p2 = Point(self.x_approx(t + step),
                               self.y_approx(t + step))
                    draw_line(screen, field, APPROX_TRAJECTORY_COLOR, p, p2)

                p_st = Point(self.x_approx(self.t_st), self.y_approx(self.t_st))
                p_fn = Point(self.x_approx(self.t_fn), self.y_approx(self.t_fn))

                p = p_st
                p2 = p
                t = self.t_st
                it = 0
                while it < TRAJECTORY_SEGMENT_COUNT and min(dist(p, p_st), dist(p, p_fn)) < 0.5:
                    it += 1
                    draw_line(screen, field, APPROX_TRAJECTORY_COLOR, p, p2)
                    t -= step
                    p2 = p
                    p = Point(self.x_approx(t), self.y_approx(t))

                p = p_fn
                p2 = p
                t = self.t_fn
                it = 0
                while it < TRAJECTORY_SEGMENT_COUNT and min(dist(p, p_fn), dist(p, p_st)) < 0.5:
                    it += 1
                    draw_line(screen, field, APPROX_TRAJECTORY_COLOR, p, p2)
                    t += step
                    p2 = p
                    p = Point(self.x_approx(t), self.y_approx(t))

        except AttributeError as e: # approximation hasn't been calculated yet
            pass
            #if e.message != "'Follower' object has no attribute 'target_point'":
            #    raise e
