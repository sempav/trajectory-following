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
from trajectory import Trajectory

from collections import namedtuple
from math import sin, cos, pi, atan2, sqrt, copysign, isnan, log
from random import gauss
import numpy as np


RECORDED_POSITIONS_CNT = 100
SAMPLE_COUNT = 40
MIN_DISTANCE_TO_LEADER = 2 * BOT_RADIUS
DISABLE_CIRCLES = False
MIN_CIRCLE_CURVATURE = 10.0

TRAJECTORY_SEGMENT_COUNT = 10
DISPLAYED_POINTS_COUNT = 0
DISPLAYED_USED_POINTS_COUNT = 0

DEFAULT_FOV = 0.25 * pi

Instr = namedtuple('Instr', 'v, omega')
TimedPosition = namedtuple('TimedPosition', 'time, pos')
TimedState = namedtuple('TimedState', 'time, pos, theta')
# used to represent bot's state vector: x, y, theta
State = namedtuple('State', 'x, y, theta')


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
                 noise_sigma=0.0, log_file=None,
                 visibility_fov=DEFAULT_FOV, visibility_radius=None,
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
        self.leader = leader
        self.orig_leader = orig_leader
        self.trajectory_delay = trajectory_delay

        assert g > 0, "Follower: g parameter must be positive"
        self.g = g
        assert 0 < zeta < 1, "Follower: zeta parameter must be in (0, 1)"
        self.zeta = zeta

        # leader_positions stores RECORDED_POSITIONS_CNT tuples;
        # tuple's first field is time, second is the
        # corresponding position of the leader
        self.leader_positions = RingBuffer(RECORDED_POSITIONS_CNT)

        # orig_leader_states stores leader's precise state;
        # used to export "real" errors (as opposed to calculated
        # w.r.t. the approximation curve)
        self.orig_leader_states = []

        self.update_interval = trajectory_delay / (RECORDED_POSITIONS_CNT - SAMPLE_COUNT // 2)
        self.last_update_time = 0.0

        self.noise_sigma = noise_sigma

        self.log_file = log_file

        self.visibility_fov = visibility_fov
        if visibility_radius is None:
            visibility_radius = 2.0 * trajectory_delay * BOT_VEL_CAP
        self.visibility_radius = visibility_radius

        self.id = id;

        if orig_leader_delay is None:
            orig_leader_delay = trajectory_delay
        self.orig_leader_delay = orig_leader_delay


        if self.log_file is not None:
            log_dict = {"id": self.id,
                         "g": self.g,
                         "zeta": self.zeta,
                         "noise_sigma": self.noise_sigma,
                         "reference_points_cnt": SAMPLE_COUNT,
                         "trajectory_delay": trajectory_delay}
            print >> self.log_file, log_dict


    def point_in_fov(self, p):
        if length(p - self.pos) > self.visibility_radius:
            return False
        d = p - self.pos
        ang = atan2(cross(self.real_dir, d), dot(self.real_dir, d))
        return -0.5 * self.visibility_fov < ang < 0.5 * self.visibility_fov


    def point_is_visible(self, p, obstacles):
        if not self.point_in_fov(p):
            return False
        try:
            ray = Ray(self.pos, p - self.pos)
        except ValueError:
            ray = Ray(self.pos, Vector(1.0, 0.0))
        i = first_intersection(ray, obstacles)
        return (i is None) or (length(i - self.pos) > length(p - self.pos))


    def store_leaders_state(self, engine, obstacles):
        if self.point_is_visible(self.leader.real.pos, obstacles):
            self.leader_is_visible = True

            noisy_pos = self.leader.real.pos
            noisy_pos += Vector(gauss(0.0, self.noise_sigma),
                                gauss(0.0, self.noise_sigma))
            self.leader_noisy_pos = noisy_pos
            self.leader_positions.append(TimedPosition(engine.time, noisy_pos))
            self.last_update_time = engine.time
        else:
            self.leader_is_visible = False

        orig_leader_theta = atan2(self.orig_leader.real.dir.y,
                                  self.orig_leader.real.dir.x)
        self.orig_leader_states.append(TimedState(time=engine.time,
                                                   pos=self.orig_leader.real.pos,
                                                   theta=orig_leader_theta))


    def polyfit_trajectory(self, pos_data, t):
        x_pos = np.array([el.pos.x for el in pos_data])
        y_pos = np.array([el.pos.y for el in pos_data])
        times = np.array([el.time   for el in pos_data])

        # needed for trajectory rendering
        self.t_st = times[0]
        self.t_fn = max(times[-1], t)

        # calculate quadratic approximation of the reference trajectory
        x_poly = np.polyfit(times, x_pos, deg=2)
        y_poly = np.polyfit(times, y_pos, deg=2)
        known = Trajectory.from_poly(x_poly, y_poly)
        return known, x_pos[-1], y_pos[-1], times[-1]


    def extend_trajectory(self, known, last_x, last_y, last_t):
        # now adding a circle to the end of known trajectory
        # k is signed curvature of the trajectry at t_fn
        try:
            k = known.curvature(last_t)
        except (ValueError, ZeroDivisionError):
            k = MIN_CIRCLE_CURVATURE
        if abs(k) < MIN_CIRCLE_CURVATURE:
            k = copysign(MIN_CIRCLE_CURVATURE, k)

        radius = abs(1.0/k)
        # trajectory direction at time t_fn
        try:
            d = normalize(Vector(known.dx(last_t), known.dy(last_t)))
        except ValueError:
            d = self.real_dir

        r = Vector(-d.y, d.x) / k
        center = Point(last_x, last_y) + r
        phase = atan2(-r.y, -r.x)
        freq = known.dx(last_t) / r.y
        self.x_approx = lambda time: known.x(time) if time < last_t else \
                                     center.x + radius * cos(freq * (time - last_t) + phase)
        self.y_approx = lambda time: known.y(time) if time < last_t else \
                                     center.y + radius * sin(freq * (time - last_t) + phase)
        dx = lambda time: known.dx(time) if time < last_t else \
                          -radius * freq * sin(freq * (time - last_t) + phase)
        dy = lambda time: known.dy(time) if time < last_t else \
                          radius * freq * cos(freq * (time - last_t) + phase)
        ddx = lambda time: known.ddx(time) if time < last_t else \
                          -radius * freq * freq * cos(freq * (time - last_t) + phase)
        ddy = lambda time: known.ddy(time) if time < last_t else \
                          -radius * freq * freq * sin(freq * (time - last_t) + phase)
        # FIXME: don't use division by y if y == 0
        try:
            if isnan(self.x_approx(last_t + 1)):
                return known
        except Exception:
            return known
        return Trajectory(x=self.x_approx, y=self.y_approx,
                          dx=dx, dy=dy, ddx=ddx, ddy=ddy)



    def generate_trajectory(self, leader_positions, t):
        arr = get_interval(self.leader_positions, t, SAMPLE_COUNT)
        self.traj_interval = arr
        if len(arr) == 0:
            return None
        known, last_x, last_y, last_t = self.polyfit_trajectory(arr, t)
        if DISABLE_CIRCLES:
            return known
        else:
            return self.extend_trajectory(known, last_x, last_y, last_t)


    def calc_desired_velocity(self, bots, obstacles, targets, engine):
        # update trajectory
        if engine.time - self.last_update_time > self.update_interval:
            self.store_leaders_state(engine, obstacles)

        # reduce random movements at the start
        if self.leader_is_visible and length(self.pos - self.leader_noisy_pos) < MIN_DISTANCE_TO_LEADER:
            return Instr(0.0, 0.0)

        t = engine.time - self.trajectory_delay
        self.trajectory = self.generate_trajectory(self.leader_positions, t)
        if self.trajectory is None:
            return Instr(0.0, 0.0)

        dx = self.trajectory.dx
        dy = self.trajectory.dy
        ddx = self.trajectory.ddx
        ddy = self.trajectory.ddy

        # calculate the feed-forward velocities
        v_fun = lambda time: sqrt(dx(time)**2 + dy(time)**2)
        #omega_fun = lambda time: (dx(time) * 2 * y_poly[0] - dy(time) * 2 * x_poly[0]) / (dx(time)**2 + dy(time)**2)
        omega_fun = lambda time: (dx(time) * ddy(time) - dy(time) * ddx(time)) / (dx(time)**2 + dy(time)**2)
        v_ff = v_fun(t)
        omega_ff = omega_fun(t)

        # x_r, y_r and theta_r denote the reference robot's state
        r = State(x=self.trajectory.x(t),
                  y=self.trajectory.y(t),
                  theta=atan2(self.trajectory.dy(t),
                              self.trajectory.dx(t)))
        self.target_point = Point(r.x, r.y)

        if isnan(v_ff):
            v_ff = 0.0
        if isnan(omega_ff):
            omega_ff = 0.0

        # cur is the current state
        cur = State(x=self.pos.x,
                    y=self.pos.y,
                    theta=atan2(self.real_dir.y, self.real_dir.x))

        # error in the global (fixed) reference frame
        delta = State(x=r.x - cur.x,
                      y=r.y - cur.y,
                      theta=(r.theta - cur.theta) % (2 * pi))
        if delta.theta > pi:
            delta = State(x=delta.x,
                          y=delta.y,
                          theta=delta.theta - 2 * pi)

        # translate error into the follower's reference frame
        e = State(x= cos(cur.theta) * delta.x + sin(cur.theta) * delta.y,
                  y=-sin(cur.theta) * delta.x + cos(cur.theta) * delta.y,
                  theta=delta.theta)

        # calculate gains k_x, k_y, k_theta
        # these are just parameters, not a state in any sense!
        omega_n = sqrt(omega_ff**2 + self.g * v_ff**2)
        k_x = 2 * self.zeta * omega_n
        k_y = self.g
        k_theta = 2 * self.zeta * omega_n

        # calculate control velocities
        v = v_ff * cos(e.theta) + k_x * e.x
        se = sin(e.theta) / e.theta if e.theta != 0 else 1.0
        omega = omega_ff + k_y * v_ff * se * e.y + k_theta * e.theta

        #v = v_ff
        #omega = omega_ff

        orig_t = engine.time - self.orig_leader_delay
        orig_arr = get_interval(self.orig_leader_states, orig_t, 3)
        real_e = State(0.0, 0.0, 0.0)
        if len(orig_arr) >= 2:
            a = orig_arr[-2]
            b = orig_arr[-1]
            # lerp nearest states to get orig_leader's state at moment t
            coeff = (orig_t - a.time) / (b.time - a.time)
            leader_pos = lerp(a.pos, b.pos, coeff)
            leader_theta = lerp_angles(a.theta, b.theta, coeff)
            self.orig_leader_pos = leader_pos
            self.orig_leader_theta = leader_theta
            real_delta = State(x=leader_pos.x - cur.x,
                               y=leader_pos.y - cur.y,
                               theta=(leader_theta - cur.theta) % (2 * pi))
            if real_delta.theta > pi:
                real_delta = State(x=real_delta.x, y=real_delta.y, theta=real_delta.theta - 2 * pi)
            real_e = real_delta

        if self.log_file is not None:
            log_dict = {"id": self.id,
                         "time": engine.time,
                         "delta_time": engine.time_since_last_bot_update,
                         "v": v,
                         "omega": omega,
                         "v_ff": v_ff,
                         "omega_ff": omega_ff,
                         "e_x": e.x,
                         "e_y": e.y,
                         "e_theta": e.theta,
                         "real_e_x": real_e.x,
                         "real_e_y": real_e.y,
                         "real_e_theta": real_e.theta}
            print >> self.log_file, log_dict

        return Instr(v,  omega)


    def draw(self, screen, field):
        if DISPLAYED_POINTS_COUNT > 0:
            k = RECORDED_POSITIONS_CNT / DISPLAYED_POINTS_COUNT
            for index, (time, point) in enumerate(self.leader_positions):
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

            if DRAW_APPROX_TRAJECTORY and self.trajectory is not None:
                #if len(self.traj_interval) > 1:
                #    p2 = Point(self.traj_interval[0].pos.x,
                #               self.traj_interval[0].pos.y)
                #    for t, p in self.traj_interval:
                #        draw_line(screen, field, APPROX_TRAJECTORY_COLOR, p, p2)
                #        p2 = p

                step = (self.t_fn - self.t_st) / TRAJECTORY_SEGMENT_COUNT
                for t in (self.t_st + k * step for k in xrange(TRAJECTORY_SEGMENT_COUNT)):
                    p = Point(self.trajectory.x(t),
                              self.trajectory.y(t))
                    p2 = Point(self.trajectory.x(t + step),
                               self.trajectory.y(t + step))
                    draw_line(screen, field, APPROX_TRAJECTORY_COLOR, p, p2)

                p_st = Point(self.trajectory.x(self.t_st), self.trajectory.y(self.t_st))
                p_fn = Point(self.trajectory.x(self.t_fn), self.trajectory.y(self.t_fn))

                step = 0.5 / TRAJECTORY_SEGMENT_COUNT
                p = p_fn
                p2 = p
                t = self.t_fn
                it = 0
                while it < TRAJECTORY_SEGMENT_COUNT and min(dist(p, p_fn), dist(p, p_st)) < 1.0:
                    it += 1
                    t += step
                    p2 = p
                    p = Point(self.trajectory.x(t), self.trajectory.y(t))
                    draw_line(screen, field, APPROX_TRAJECTORY_COLOR, p, p2)

        except AttributeError as e: # approximation hasn't been calculated yet
            pass
            #if e.message != "'Follower' object has no attribute 'target_point'":
            #    raise e
