"""Top-down 2D stand-in for the robot, ball, goal and camera, for testing the controller offline.

Every physical constant here is a guess, not a measurement of the real robot, so a
good result in simulation only shows the decision logic is sound. Retune the real
thresholds on the field.
"""
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from control import BALL, GOAL, Detection, Move, Observation  # noqa: E402


@dataclass
class SimParams:
    field_w: float = 2.0             # metres along x
    field_h: float = 1.6             # metres along y, goal mouth sits on the y = field_h edge
    goal_w: float = 0.45
    ball_r: float = 0.05
    robot_half_len: float = 0.11
    robot_half_wid: float = 0.09
    cam_height: float = 0.12
    cam_pitch_deg: float = 20.0      # downward tilt
    hfov_deg: float = 70.0
    vfov_deg: float = 52.0
    max_speed_mps: float = 0.6       # forward speed at drive speed 255
    lateral_factor: float = 0.7      # mecanum strafing is slower than driving forward
    max_turn_rps: float = 3.0        # rad/s at speed 255
    motion_noise: float = 0.10       # relative error on every pulse
    ball_friction: float = 1.5       # rolling deceleration, m/s^2
    detect_far_m: float = 2.5
    pixel_noise: float = 0.015
    pulse_overhead_s: float = 0.10   # robot.drive() sleeps this long after each pulse
    observe_s: float = 0.10          # frame wait plus inference
    # FOMO at 96x96 reports centroids on a 12x12 grid and one class per cell; 0 disables this.
    fomo_cells: int = 0


@dataclass
class Pose:
    x: float
    y: float
    theta: float  # radians, 0 points along +x, pi/2 points at the goal


@dataclass
class World:
    params: SimParams
    robot: Pose
    ball_x: float
    ball_y: float
    rng: random.Random
    ball_vx: float = 0.0
    ball_vy: float = 0.0
    time_s: float = 0.0
    scored: bool = False

    def clock(self) -> float:
        return self.time_s


class Scored(Exception):
    pass


class BallOut(Exception):
    """The ball crossed the taped boundary outside the goal mouth; the field has no walls."""


class SimPerception:
    def __init__(self, world: World) -> None:
        self._world = world
        self._seq = 0

    def observe(self, wait_s: float) -> Optional[Observation]:
        w = self._world
        w.time_s += w.params.observe_s
        self._seq += 1
        ball = self._project(BALL, w.ball_x, w.ball_y)
        goal = self._project(GOAL, w.params.field_w / 2, w.params.field_h)
        cells = w.params.fomo_cells
        if cells:
            ball, goal = self._snap(ball, cells), self._snap(goal, cells)
            if ball is not None and goal is not None and (ball.x, ball.y) == (goal.x, goal.y):
                goal = None  # one class per cell, and the nearer ball dominates it
        return Observation(ball=ball, goal=goal, frame_age_s=0.05, seq=self._seq)

    @staticmethod
    def _snap(det: Optional[Detection], cells: int) -> Optional[Detection]:
        if det is None:
            return None
        def centre(v: float) -> float:
            return (min(cells - 1, max(0, int(v * cells))) + 0.5) / cells
        return Detection(det.label, centre(det.x), centre(det.y), det.score)

    def _project(self, label: str, gx: float, gy: float) -> Optional[Detection]:
        w, p = self._world, self._world.params
        dx, dy = gx - w.robot.x, gy - w.robot.y
        cos_t, sin_t = math.cos(w.robot.theta), math.sin(w.robot.theta)
        forward = dx * cos_t + dy * sin_t - p.robot_half_len
        left = -dx * sin_t + dy * cos_t
        if forward <= 0.02:
            return None
        img_x = 0.5 - math.tan(math.atan2(left, forward)) / (2 * math.tan(math.radians(p.hfov_deg) / 2))
        below = math.atan2(p.cam_height, forward) - math.radians(p.cam_pitch_deg)
        img_y = 0.5 + math.tan(below) / (2 * math.tan(math.radians(p.vfov_deg) / 2))
        if not (0.0 <= img_x <= 1.0 and 0.0 <= img_y <= 1.0):
            return None
        distance = math.hypot(forward, left)
        if w.rng.random() > max(0.3, 1.0 - 0.6 * distance / p.detect_far_m):
            return None
        return Detection(label=label,
                         x=img_x + w.rng.gauss(0, p.pixel_noise),
                         y=img_y + w.rng.gauss(0, p.pixel_noise),
                         score=0.8)


class SimExecutor:
    STEP_S = 0.01

    def __init__(self, world: World) -> None:
        self._world = world

    def execute(self, move: Move) -> None:
        w, p = self._world, self._world.params
        scale = move.speed / 255.0 * (1 + w.rng.gauss(0, p.motion_noise))
        v_fwd = v_left = omega = 0.0
        if move.direction == "forward":
            v_fwd = scale * p.max_speed_mps
        elif move.direction == "backward":
            v_fwd = -scale * p.max_speed_mps
        elif move.direction == "left":
            v_left = scale * p.max_speed_mps * p.lateral_factor
        elif move.direction == "right":
            v_left = -scale * p.max_speed_mps * p.lateral_factor
        elif move.direction == "rotate_left":
            omega = scale * p.max_turn_rps
        elif move.direction == "rotate_right":
            omega = -scale * p.max_turn_rps
        drift = w.rng.gauss(0, p.motion_noise * 0.3)
        for _ in range(int(move.ms / 1000.0 / self.STEP_S)):
            self._advance_robot(v_fwd, v_left, omega + drift * (abs(v_fwd) + abs(v_left)))
        self.wait(int(p.pulse_overhead_s * 1000))

    def wait(self, ms: int) -> None:
        for _ in range(int(ms / 1000.0 / self.STEP_S)):
            self._advance_robot(0.0, 0.0, 0.0)

    def _advance_robot(self, v_fwd: float, v_left: float, omega: float) -> None:
        w, dt = self._world, self.STEP_S
        r = w.robot
        cos_t, sin_t = math.cos(r.theta), math.sin(r.theta)
        vx = v_fwd * cos_t - v_left * sin_t
        vy = v_fwd * sin_t + v_left * cos_t
        r.x += vx * dt
        r.y += vy * dt
        r.theta = (r.theta + omega * dt) % (2 * math.pi)
        self._push_ball(vx, vy)
        self._roll_ball()
        w.time_s += dt
        if w.scored:
            raise Scored

    def _push_ball(self, vx: float, vy: float) -> None:
        w, p, r = self._world, self._world.params, self._world.robot
        dx, dy = w.ball_x - r.x, w.ball_y - r.y
        cos_t, sin_t = math.cos(r.theta), math.sin(r.theta)
        forward = dx * cos_t + dy * sin_t
        left = -dx * sin_t + dy * cos_t
        reach = p.robot_half_len + p.ball_r
        if not (0 < forward < reach and abs(left) < p.robot_half_wid + p.ball_r):
            return
        # Contact: shove the ball to the bumper and give it the robot's velocity,
        # deflected sideways when it is hit off-centre.
        deflect = left / (p.robot_half_wid + p.ball_r)
        w.ball_x += (reach - forward) * cos_t
        w.ball_y += (reach - forward) * sin_t
        w.ball_vx = vx - deflect * 0.3 * sin_t * abs(vx + vy)
        w.ball_vy = vy + deflect * 0.3 * cos_t * abs(vx + vy)

    def _roll_ball(self) -> None:
        w, p, dt = self._world, self._world.params, self.STEP_S
        speed = math.hypot(w.ball_vx, w.ball_vy)
        if speed > 0:
            slowed = max(0.0, speed - p.ball_friction * dt)
            w.ball_vx *= slowed / speed
            w.ball_vy *= slowed / speed
        w.ball_x += w.ball_vx * dt
        w.ball_y += w.ball_vy * dt
        goal_left = (p.field_w - p.goal_w) / 2
        if w.ball_y >= p.field_h and goal_left <= w.ball_x <= goal_left + p.goal_w:
            w.scored = True
            return
        if not (0 <= w.ball_x <= p.field_w and 0 <= w.ball_y <= p.field_h):
            raise BallOut


class SimLineSensor:
    """Fires when the robot's nose crosses the taped boundary, like the real 4-channel sensor."""

    def __init__(self, world: World) -> None:
        self._world = world

    def triggered(self) -> bool:
        w, p = self._world, self._world.params
        nose_x = w.robot.x + p.robot_half_len * math.cos(w.robot.theta)
        nose_y = w.robot.y + p.robot_half_len * math.sin(w.robot.theta)
        return not (0 <= nose_x <= p.field_w and 0 <= nose_y <= p.field_h)


def random_world(seed: int, params: Optional[SimParams] = None) -> World:
    p = params or SimParams()
    rng = random.Random(seed)
    robot = Pose(x=rng.uniform(0.3, p.field_w - 0.3), y=rng.uniform(0.2, 0.5),
                 theta=rng.uniform(0, 2 * math.pi))
    return World(params=p, robot=robot, rng=rng,
                 ball_x=rng.uniform(0.3, p.field_w - 0.3), ball_y=rng.uniform(0.6, p.field_h - 0.4))
