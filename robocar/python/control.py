import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union

BALL = "soccerball"
GOAL = "goal"
DIRECTIONS = ("forward", "backward", "left", "right", "rotate_left", "rotate_right")
ROTATIONS = ("rotate_left", "rotate_right")


@dataclass(frozen=True)
class Detection:
    label: str
    x: float  # centroid as a fraction of image width, 0 = left edge
    y: float  # centroid as a fraction of image height, 0 = top edge
    score: float


@dataclass(frozen=True)
class Observation:
    ball: Optional[Detection]
    goal: Optional[Detection]
    frame_age_s: float
    seq: int


class DetectorTimeout(RuntimeError):
    """Inference took too long; defined here so the run loop needs no OpenCV import to catch it."""


@dataclass(frozen=True)
class Move:
    direction: str
    speed: int
    ms: int

    def __post_init__(self) -> None:
        if self.direction not in DIRECTIONS:
            raise ValueError(f"unknown direction {self.direction!r}")
        if self.ms <= 0:
            raise ValueError("ms must be positive; firmware treats 0 as continuous motion")


@dataclass(frozen=True)
class Wait:
    ms: int


@dataclass(frozen=True)
class Halt:
    reason: str


Action = Union[Move, Wait, Halt]


class Phase(Enum):
    SEARCH = "search"
    APPROACH = "approach"
    LINEUP = "lineup"
    SCAN = "scan"
    PUSH = "push"
    RECOVER = "recover"


@dataclass(frozen=True)
class ControlConfig:
    """Every threshold here is a starting guess; retune each one on the real field."""

    # Image coordinates are fractions: x from the left edge, y from the top edge.
    center_band: float = 0.08        # |ball.x - 0.5| below this counts as centred
    push_band: float = 0.15          # wider band tolerated once a push has started
    align_band: float = 0.07         # |goal.x - ball.x| below this counts as lined up
    push_align_band: float = 0.15    # misalignment that sends a running push back to lineup
    near_y: float = 0.60             # ball.y >= this (lower in the frame) is close enough to line up
    drift_margin: float = 0.10       # lineup goes back to approach when ball.y < near_y - this
    ball_lost_grace: int = 3         # consecutive misses tolerated before acting on a loss
    # robocar/README.md says to stop and end the run when the ball is lost, while the
    # organiser said that finding the ball again after a miss earns goal-directed points.
    # Ask the judges, then set this; False halts on a loss.
    search_when_lost: bool = True

    turn_speed: int = 120
    drive_speed: int = 130           # also the speed of backward pulses in search and recover
    strafe_speed: int = 140
    push_speed: int = 170
    min_pulse_ms: int = 80
    max_turn_ms: int = 220
    max_approach_ms: int = 300       # also the length of backward pulses in search and recover
    strafe_ms: int = 150
    push_ms: int = 250
    wait_ms: int = 60
    turn_ms_per_unit: float = 600.0  # rotate pulse length per unit of horizontal error
    approach_ms_per_unit: float = 900.0  # forward pulse length per unit of gap between ball.y and near_y

    # Measure both on the robot. turn_rad_per_speed_ms = radians turned by a rotate pulse
    # (protractor) / (speed * ms) and assumes the angle is linear in both, so measure
    # again after changing turn_speed or max_turn_ms. hfov_deg comes from a ruler at a
    # known distance.
    hfov_deg: float = 70.0
    turn_rad_per_speed_ms: float = 1.2e-5
    align_angle_deg: float = 6.0     # remembered goal this close behind the ball counts as lined up
    goal_memory_pulses: int = 40     # forget the remembered goal after this many moves of drift
    trusted_goal_age: int = 4        # a goal seen within this many moves may justify a push unseen

    blind_push_pulses: int = 2       # the ball drops below the frame when touching the bumper
    max_push_pulses: int = 20
    recover_back_pulses: int = 2
    max_orbit_pulses: int = 24       # give up lining up and push anyway after this many pulses
    search_back_pulses: int = 2      # back off after each fruitless lap to uncover a ball at the bumper
    max_search_laps: int = 3

    def __post_init__(self) -> None:
        # Phase handlers hand over to each other within one step, so overlapping bands
        # make two phases hand the same observation back and forth until RecursionError.
        if self.push_band < self.center_band:
            raise ValueError("push_band must be >= center_band")
        if self.push_align_band < self.align_band:
            raise ValueError("push_align_band must be >= align_band")
        if self.drift_margin < 0:
            raise ValueError("drift_margin must be >= 0")
        # _enter zeroes the phase counter, so a budget below 1 fires the moment a phase starts.
        for name in ("max_push_pulses", "max_orbit_pulses"):
            if getattr(self, name) < 1:
                raise ValueError(f"{name} must be >= 1")
        # pulses_per_lap and the scan's return turn divide by these, and a 0 placeholder
        # would only crash on the first search or scan.
        # The pulse lengths feed Move, which rejects 0 because firmware treats it as non-stop.
        for name in ("turn_speed", "max_turn_ms", "turn_rad_per_speed_ms",
                     "min_pulse_ms", "max_approach_ms", "strafe_ms", "push_ms"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")

    @property
    def pulses_per_lap(self) -> int:
        """Full-length rotate pulses for one full turn, with 10% to spare.

        Searching and scanning both rely on this, so turn_rad_per_speed_ms is the
        first value to calibrate. In simulation, turns 40% slower than assumed cost
        about a quarter of the goals.
        """
        per_pulse = self.turn_speed * self.max_turn_ms * self.turn_rad_per_speed_ms
        return math.ceil(1.1 * 2 * math.pi / per_pulse)


class HeadingMemory:
    """Dead-reckons heading from issued rotate pulses and remembers which way the goal lies.

    Only rotations change heading here, and every pulse adds drift, so the remembered
    goal expires after goal_memory_pulses moves.
    """

    def __init__(self, cfg: ControlConfig) -> None:
        self._cfg = cfg
        self._half_fov = math.radians(cfg.hfov_deg) / 2
        self._half_fov_tan = math.tan(self._half_fov)
        self._heading = 0.0
        self._goal_heading: Optional[float] = None
        self._goal_age = 0
        self._ball_heading: Optional[float] = None

    def bearing(self, x: float) -> float:
        """Angle of an image column from the camera axis, positive to the left."""
        return math.atan((0.5 - x) * 2 * self._half_fov_tan)

    def half_fov(self) -> float:
        return self._half_fov

    def see_goal(self, goal_x: float) -> None:
        self._goal_heading = self._heading + self.bearing(goal_x)
        self._goal_age = 0

    def see_ball(self, ball_x: float) -> None:
        self._ball_heading = self._heading + self.bearing(ball_x)

    def on_move(self, move: Move) -> None:
        self._goal_age += 1
        if move.direction in ROTATIONS:
            turn = move.speed * move.ms * self._cfg.turn_rad_per_speed_ms
            self._heading += turn if move.direction == "rotate_left" else -turn

    @property
    def goal_age(self) -> int:
        """Moves issued since the goal was last seen."""
        return self._goal_age

    def knows_goal(self) -> bool:
        return self._goal_heading is not None and self._goal_age <= self._cfg.goal_memory_pulses

    def goal_relative_to(self, x: float) -> Optional[float]:
        """Remembered goal direction minus the direction of image column x, positive to the left."""
        if not self.knows_goal():
            return None
        return _wrap(self._goal_heading - (self._heading + self.bearing(x)))

    def ball_relative(self) -> Optional[float]:
        """Remembered ball direction relative to the camera axis, positive to the left."""
        if self._ball_heading is None:
            return None
        return _wrap(self._ball_heading - self._heading)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class BallGoalController:
    """Lining up alternates lateral strafes with rotations that keep the ball centred, so
    the robot orbits the ball. A strafe shifts the nearby ball across the image more
    than the distant goal, which changes the gap between them, while a rotation shifts
    both equally and leaves the gap alone, so the two corrections barely interfere.
    """

    def __init__(self, config: ControlConfig) -> None:
        self.cfg = config
        self.phase = Phase.SEARCH
        self.reason = ""
        self._memory = HeadingMemory(config)
        self._misses = 0
        self._last_ball_x = 0.5
        self._phase_pulses = 0
        self._search_pulses = 0
        self._goal_scan_failed = False
        self._return_turn = "rotate_right"  # the scan spins left, so the way back is usually right
        self._handlers = {
            Phase.SEARCH: self._search,
            Phase.APPROACH: self._approach,
            Phase.LINEUP: self._lineup,
            Phase.SCAN: self._scan,
            Phase.PUSH: self._push,
            Phase.RECOVER: self._recover,
        }

    def notify_boundary(self) -> None:
        """Call when a line sensor sees the field edge; the caller has already backed off."""
        self._enter(Phase.RECOVER, "boundary")

    def step(self, obs: Observation) -> Action:
        ball = obs.ball
        if ball is None:
            self._misses += 1
        else:
            self._misses = 0
            self._last_ball_x = ball.x
            self._memory.see_ball(ball.x)
        if obs.goal is not None:
            self._memory.see_goal(obs.goal.x)
            self._goal_scan_failed = False
        action = self._handlers[self.phase](ball, obs.goal)
        if isinstance(action, Halt):
            # Halt branches never set reason themselves, and the run log reads it after step().
            self.reason = action.reason
        elif isinstance(action, Move):
            self._memory.on_move(action)
        self._phase_pulses += 1
        return action

    def _search(self, ball: Optional[Detection], goal: Optional[Detection]) -> Action:
        if ball is not None:
            self._search_pulses = 0
            self._enter(Phase.APPROACH, "ball found")
            return self._approach(ball, goal)
        self._search_pulses += 1
        lap = self.cfg.pulses_per_lap
        if self._search_pulses > self.cfg.max_search_laps * lap:
            return Halt("ball not found after searching")
        if self._search_pulses > lap and (self._search_pulses - 1) % lap < self.cfg.search_back_pulses:
            self.reason = "back off after a fruitless lap"
            return Move("backward", self.cfg.drive_speed, self.cfg.max_approach_ms)
        self.reason = "spin toward the side the ball was last seen"
        direction = "rotate_left" if self._last_ball_x < 0.5 else "rotate_right"
        return Move(direction, self.cfg.turn_speed, self.cfg.max_turn_ms)

    def _approach(self, ball: Optional[Detection], goal: Optional[Detection]) -> Action:
        if ball is None:
            return self._on_missing()
        if abs(ball.x - 0.5) > self.cfg.center_band:
            return self._turn_toward(ball)
        if ball.y >= self.cfg.near_y:
            self._enter(Phase.LINEUP, "ball near")
            return self._lineup(ball, goal)
        self.reason = "drive toward the ball"
        gap = self.cfg.near_y - ball.y
        return Move("forward", self.cfg.drive_speed,
                    self._clamp_ms(gap * self.cfg.approach_ms_per_unit, self.cfg.max_approach_ms))

    def _lineup(self, ball: Optional[Detection], goal: Optional[Detection]) -> Action:
        if ball is None:
            return self._on_missing()
        if abs(ball.x - 0.5) > self.cfg.center_band:
            return self._turn_toward(ball)
        if ball.y < self.cfg.near_y - self.cfg.drift_margin:
            self._enter(Phase.APPROACH, "ball drifted away")
            return self._approach(ball, goal)
        if self._phase_pulses >= self.cfg.max_orbit_pulses:
            self._enter(Phase.PUSH, "orbit budget spent, pushing anyway")
            return self._push(ball, goal)

        if goal is not None:
            if abs(goal.x - ball.x) <= self.cfg.align_band:
                self._enter(Phase.PUSH, "ball and goal lined up")
                return self._push(ball, goal)
            goal_on_right = goal.x > ball.x
            self.reason = "orbit to line up"
        else:
            relative = self._memory.goal_relative_to(ball.x)
            if relative is None:
                if self._goal_scan_failed:
                    self._enter(Phase.PUSH, "goal never found, pushing straight")
                    return self._push(ball, goal)
                self._enter(Phase.SCAN, "goal direction unknown")
                return self._scan(ball, goal)
            # Dead-reckoned heading drifts, so only a recent sighting may trigger a push;
            # older memory just picks the orbit direction.
            recent = self._memory.goal_age <= self.cfg.trusted_goal_age
            if recent and abs(relative) <= math.radians(self.cfg.align_angle_deg):
                self._enter(Phase.PUSH, "goal seen moments ago lies behind the ball")
                return self._push(ball, goal)
            goal_on_right = relative < 0
            self.reason = "orbit toward the remembered goal"
        # Goal right of the ball means sliding left puts the ball between robot and goal.
        return Move("left" if goal_on_right else "right", self.cfg.strafe_speed, self.cfg.strafe_ms)

    def _scan(self, ball: Optional[Detection], goal: Optional[Detection]) -> Action:
        """Spin in place until the goal is seen, then turn back to the remembered ball."""
        lap = self.cfg.pulses_per_lap
        # Two laps of budget. The scan ends at the first sighting, so the second lap is only
        # spent when the goal never appears or an overestimated turn rate leaves one "lap" short.
        scan_over = self._memory.knows_goal() or self._phase_pulses >= 2 * lap
        if not scan_over:
            self.reason = "scan in place for the goal"
            return Move("rotate_left", self.cfg.turn_speed, self.cfg.max_turn_ms)
        if not self._memory.knows_goal():
            self._goal_scan_failed = True
        if ball is not None:
            self._enter(Phase.APPROACH, "back on the ball after scanning")
            return self._approach(ball, goal)
        relative = self._memory.ball_relative()
        if relative is None or self._phase_pulses >= 4 * lap:
            return self._lose_ball("lost the ball while scanning")
        if abs(relative) > self._memory.half_fov():
            self._return_turn = "rotate_left" if relative > 0 else "rotate_right"
            self.reason = "turn back to the remembered ball"
            ms = abs(relative) / (self.cfg.turn_speed * self.cfg.turn_rad_per_speed_ms)
            return Move(self._return_turn, self.cfg.turn_speed, self._clamp_ms(ms, self.cfg.max_turn_ms))
        # Memory says the ball should be in view but it is not, which usually means the
        # turn rate is overestimated and the robot under-rotated, so keep turning the same way.
        self.reason = "ball not where memory says, keep turning"
        return Move(self._return_turn, self.cfg.turn_speed, self.cfg.max_turn_ms)

    def _push(self, ball: Optional[Detection], goal: Optional[Detection]) -> Action:
        if self._phase_pulses >= self.cfg.max_push_pulses:
            self._enter(Phase.RECOVER, "push budget spent")
            return self._recover(ball, goal)
        if ball is None:
            goal_ahead = goal is not None and abs(goal.x - 0.5) <= self.cfg.push_band
            if goal_ahead:
                self.reason = "dribble toward the goal, ball below the camera"
                return Move("forward", self.cfg.push_speed, self.cfg.push_ms)
            if self._misses > self.cfg.blind_push_pulses:
                self._enter(Phase.RECOVER, "ball lost during push")
                return self._recover(ball, goal)
            self.reason = "blind push, ball likely below the camera"
            return Move("forward", self.cfg.push_speed, self.cfg.push_ms)
        misaligned = goal is not None and abs(goal.x - ball.x) > self.cfg.push_align_band
        if abs(ball.x - 0.5) > self.cfg.push_band or misaligned:
            self._enter(Phase.LINEUP, "ball slid off the line to the goal")
            return self._lineup(ball, goal)
        self.reason = "push"
        return Move("forward", self.cfg.push_speed, self.cfg.push_ms)

    def _recover(self, ball: Optional[Detection], goal: Optional[Detection]) -> Action:
        if self._phase_pulses < self.cfg.recover_back_pulses:
            self.reason = "back off to widen the view"
            return Move("backward", self.cfg.drive_speed, self.cfg.max_approach_ms)
        if ball is not None:
            self._enter(Phase.APPROACH, "ball found after recovery")
            return self._approach(ball, goal)
        return self._lose_ball("ball not visible after backing off")

    def _on_missing(self) -> Action:
        if self._misses <= self.cfg.ball_lost_grace:
            self.reason = "ball missing, waiting for a fresh frame"
            return Wait(self.cfg.wait_ms)
        return self._lose_ball("ball lost")

    def _lose_ball(self, reason: str) -> Action:
        """Every route that gives up on the ball comes here, so search_when_lost applies to all."""
        if not self.cfg.search_when_lost:
            return Halt(reason)
        self._enter(Phase.SEARCH, reason)
        return self._search(None, None)

    def _turn_toward(self, ball: Detection) -> Move:
        self.reason = "centre the ball"
        error = ball.x - 0.5
        ms = self._clamp_ms(abs(error) * self.cfg.turn_ms_per_unit, self.cfg.max_turn_ms)
        return Move("rotate_right" if error > 0 else "rotate_left", self.cfg.turn_speed, ms)

    def _clamp_ms(self, ms: float, upper: int) -> int:
        return int(max(self.cfg.min_pulse_ms, min(upper, ms)))

    def _enter(self, phase: Phase, reason: str) -> None:
        self.phase = phase
        self.reason = reason
        self._phase_pulses = 0
