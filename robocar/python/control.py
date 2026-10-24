import math
from collections import deque
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

    # The sketch gives each wheel half the PWM for a rotation that it gives for a translation,
    # so 180 drives the wheels about as hard as forward at drive_speed 130 (PWM 90 of 255).
    # At 120 they got PWM 60, near where cheap TT gear motors are reported to stall.
    turn_speed: int = 180
    # Slowest rotate speed at which a min_pulse_ms pulse still turns the robot. A centring
    # turn that needs a shorter pulse slows toward it, but only while overshoots have kept
    # the centring turns shrunk. Check on the field that the robot really turns at this speed.
    # 110 gives PWM 55, a little under the 65 of Hiwonder's own motor test for this chassis,
    # because once turns ran twice the assumed rate, 130 scored 149 of 200 in simulation and 110
    # scored 178, as slower turns are what lets centring settle.
    min_turn_speed: int = 110
    drive_speed: int = 130           # also the speed of backward pulses in search and recover
    strafe_speed: int = 140
    push_speed: int = 170
    min_pulse_ms: int = 80
    max_approach_ms: int = 300       # also the length of backward pulses in search and recover
    strafe_ms: int = 150
    push_ms: int = 250
    wait_ms: int = 60
    # Centring turns are angles, turned into pulse lengths through turn_rad_per_speed_ms like the
    # look step, so calibrating the turn rate also sizes them. Per unit of horizontal error the
    # turn is about 70% of the angle to the ball, and the cap applies to centring turns only.
    centre_turn_rad_per_unit: float = 0.864
    max_centre_turn_rad: float = 0.3168
    approach_ms_per_unit: float = 900.0  # forward pulse length per unit of gap between ball.y and near_y

    # Measure both on the robot. turn_rad_per_speed_ms = radians turned by a rotate pulse
    # (protractor) / (speed * ms) and assumes the angle is linear in both. MODE = "turn_check"
    # in main.py turns look steps for this, since short pulses are expected to lose more to
    # motor spin-up. Set turn_rad_per_speed_ms to about 0.75 of the measured value, which keeps
    # real turns faster than assumed. In simulation real turns 0.9 to 1.5 times the assumed rate
    # scored 196 to 199 of 200, while 0.8 times scored 182 and 2 times 163. Measure again after
    # changing turn_speed, hfov_deg or look_step_frac, which move look_step_ms.
    # hfov_deg comes from a ruler at a known distance.
    hfov_deg: float = 70.0
    turn_rad_per_speed_ms: float = 1.2e-5
    # Both come from turn_check through tools/turn_calc.py and change only the pulse sent, see
    # pulse_ms. pulse_dead_ms is the start of a pulse that moves nothing, from static friction
    # and spin-up. In simulation 60 ms of it cost 85 of 200 goals and 100 ms all of them.
    # Sending from the true value to twice it won them back, while three times it lost a quarter
    # again, because the shortest turn then overshoots the centre band. right_turn_scale
    # lengthens right turns for motors that turn right slower than left.
    pulse_dead_ms: int = 0
    right_turn_scale: float = 1.0
    align_angle_deg: float = 6.0     # remembered goal this close behind the ball counts as lined up
    goal_memory_pulses: int = 40     # forget the remembered goal after this many moves of drift
    trusted_goal_age: int = 4        # a goal seen within this many moves may justify a push unseen

    blind_push_pulses: int = 2       # the ball drops below the frame when touching the bumper
    max_push_pulses: int = 20
    recover_back_pulses: int = 2
    max_orbit_pulses: int = 24       # give up lining up and push anyway after this many pulses
    search_back_pulses: int = 2      # back off after each fruitless lap to uncover a ball at the bumper
    max_search_laps: int = 3
    # Search and scan turn this fraction of hfov between looks. Half leaves enough overlap
    # that a narrower lens or a faster turn than calibrated still leaves no blind gap.
    look_step_frac: float = 0.5
    # A ball last seen this close to an image edge rolled out of view, so the search starts
    # at once toward that edge and skips ball_lost_grace.
    edge_band: float = 0.15
    # Sightings within the last confirm_window frames that end a search; scan and recover
    # accept a single sighting. Raise to 2 if detect_only shows phantom balls flickering
    # in single frames. In simulation 2 gained a few goals against such phantoms, lost a few
    # when the detector missed more, and a phantom that persists passes it anyway.
    confirm_hits: int = 1
    confirm_window: int = 3

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
        # look_step_ms and the scan's return turn divide by turn_speed and turn_rad_per_speed_ms.
        # The pulse lengths feed Move, which rejects 0 because firmware treats it as non-stop.
        for name in ("turn_speed", "turn_rad_per_speed_ms", "centre_turn_rad_per_unit", "max_centre_turn_rad",
                     "min_pulse_ms", "max_approach_ms", "strafe_ms", "push_ms"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")
        # pulses_per_lap divides by the look step, and tan(hfov / 2) breaks at 180 degrees.
        if not 0 < self.hfov_deg < 180:
            raise ValueError("hfov_deg must be in (0, 180)")
        if not 0 < self.look_step_frac <= 1:
            raise ValueError("look_step_frac must be in (0, 1]")
        # At 0.5 or more every image column counts as an edge and the grace wait never runs.
        if not 0 <= self.edge_band < 0.5:
            raise ValueError("edge_band must be in [0, 0.5)")
        if not 1 <= self.confirm_hits <= self.confirm_window:
            raise ValueError("confirm_hits must be between 1 and confirm_window")
        if not 0 < self.min_turn_speed <= self.turn_speed:
            raise ValueError("min_turn_speed must be in (0, turn_speed]")
        if not 0 <= self.pulse_dead_ms <= 300:
            raise ValueError("pulse_dead_ms must be in [0, 300]")
        if not 0.5 <= self.right_turn_scale <= 2.0:
            raise ValueError("right_turn_scale must be in [0.5, 2.0]")
        # The firmware clamps pulses to 5000 ms while Python waits the full length it asked for.
        if max(self.pulse_ms(d, self.look_step_ms) for d in ROTATIONS) > 5000:
            raise ValueError("a look step pulse exceeds the firmware's 5000 ms limit")

    @property
    def look_step_rad(self) -> float:
        return self.look_step_frac * math.radians(self.hfov_deg)

    @property
    def look_step_ms(self) -> int:
        """Rotate pulse length that turns one look step.

        Search and scan both turn by this, so turn_rad_per_speed_ms is the first value
        to calibrate. In simulation, turns 40% slower than assumed cost about a fifth
        of the goals.
        """
        return max(1, round(self.look_step_rad / (self.turn_speed * self.turn_rad_per_speed_ms)))

    @property
    def turn_ms_per_unit(self) -> float:
        return self.centre_turn_rad_per_unit / (self.turn_speed * self.turn_rad_per_speed_ms)

    @property
    def max_turn_ms(self) -> int:
        return round(self.max_centre_turn_rad / (self.turn_speed * self.turn_rad_per_speed_ms))

    @property
    def pulses_per_lap(self) -> int:
        """Look steps for one full turn, with 10% to spare."""
        return math.ceil(1.1 * 2 * math.pi / self.look_step_rad)

    def pulse_ms(self, direction: str, ms: int) -> int:
        """Pulse to send so the motors turn or drive for ms, which is how every move is sized."""
        if direction == "rotate_right":
            ms = round(ms * self.right_turn_scale)
        return ms + self.pulse_dead_ms


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


class CentringGain:
    """Shrinks centring turns after one overshoots, because the true turn rate is only roughly known.

    At the calibrated rate even the shortest pulse lands inside the centre band. A turn rate
    well above it makes that pulse jump the ball across the band, and the robot rocks from
    side to side of the ball. A turn that falls short grows the next ones back, never past
    the calibrated length, so a shrink caused by detector jitter does not last the whole run.
    """

    _CALIBRATED = 1.0
    _SHRINK = 0.6
    _REGROW = 1.15
    _FLOOR = 0.2

    def __init__(self, center_band: float) -> None:
        self._center_band = center_band
        self._gain = self._CALIBRATED
        self._error_before: Optional[float] = None

    @property
    def gain(self) -> float:
        return self._gain

    @property
    def shrunk(self) -> bool:
        return self._gain < self._CALIBRATED

    def turned(self, error: float) -> None:
        self._error_before = error

    def observe(self, ball: Optional[Detection]) -> None:
        """Call with every frame; only the frame right after a centring turn is compared."""
        before, self._error_before = self._error_before, None
        if before is None or ball is None:
            return
        after = ball.x - 0.5
        if abs(after) <= self._center_band:
            return
        if before * after < 0:
            self._gain = max(self._FLOOR, self._gain * self._SHRINK)
        else:
            self._gain = min(self._CALIBRATED, self._gain * self._REGROW)


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
        self._centring = CentringGain(config.center_band)
        self._misses = 0
        self._last_ball_x = 0.5
        self._phase_pulses = 0
        self._search_pulses = 0
        self._goal_scan_failed = False
        self._return_turn: Optional[str] = None
        # -1 or +1 if the ball left over the left or right edge, read only while it is missing.
        self._exit_side = 0
        self._search_turn: Optional[str] = None
        self._sightings = deque(maxlen=config.confirm_window)
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
        # Before the handlers, because a centring turn they issue records the error that the
        # next frame is compared with, and this frame would otherwise be compared with itself.
        self._centring.observe(ball)
        self._sightings.append(ball is not None)
        if ball is None:
            if self._misses == 0:
                self._exit_side = self._edge_side(self._last_ball_x)
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
            if sum(self._sightings) >= self.cfg.confirm_hits:
                self._enter(Phase.APPROACH, "ball found")
                return self._approach(ball, goal)
            self.reason = "possible ball, holding still to confirm it"
            return Wait(self.cfg.wait_ms)
        if self._search_turn is None:
            # Chosen once per search: re-choosing every step would flip the turn as soon
            # as the remembered goal came into view, and the robot would rock in place.
            self._search_turn = self._choose_search_turn()
        self._search_pulses += 1
        lap = self.cfg.pulses_per_lap
        if self._search_pulses > self.cfg.max_search_laps * lap:
            return Halt("ball not found after searching")
        if self._search_pulses > lap and (self._search_pulses - 1) % lap < self.cfg.search_back_pulses:
            self.reason = "back off after a fruitless lap"
            return Move("backward", self.cfg.drive_speed, self.cfg.max_approach_ms)
        self.reason = "turn one look step and look again"
        return Move(self._search_turn, self.cfg.turn_speed, self.cfg.look_step_ms)

    def _choose_search_turn(self) -> str:
        """Edge the ball rolled over, else the remembered goal when it is out of view, else the last side."""
        if self._exit_side:
            right = self._exit_side > 0
        else:
            relative = self._memory.goal_relative_to(0.5)
            if relative is not None and abs(relative) > self._memory.half_fov():
                # A missed shot usually leaves the ball near the goal.
                right = relative < 0
            else:
                right = self._last_ball_x >= 0.5
        return "rotate_right" if right else "rotate_left"

    def _edge_side(self, x: float) -> int:
        if x < self.cfg.edge_band:
            return -1
        if x > 1 - self.cfg.edge_band:
            return 1
        return 0

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
            return Move("rotate_left", self.cfg.turn_speed, self.cfg.look_step_ms)
        if not self._memory.knows_goal():
            self._goal_scan_failed = True
        if ball is not None:
            self._enter(Phase.APPROACH, "back on the ball after scanning")
            return self._approach(ball, goal)
        relative = self._memory.ball_relative()
        if relative is None or self._phase_pulses >= 4 * lap:
            return self._lose_ball("lost the ball while scanning")
        if self._return_turn is None:
            # Chosen once per scan. With an overestimated turn rate the remembered ball passes
            # the view without the real one appearing, and re-choosing then reversed the turn
            # every few pulses, so the robot rocked over the same arc.
            self._return_turn = "rotate_left" if relative > 0 else "rotate_right"
        toward = (relative > 0) == (self._return_turn == "rotate_left")
        if toward and abs(relative) > self._memory.half_fov():
            self.reason = "turn back to the remembered ball"
            ms = abs(relative) / (self.cfg.turn_speed * self.cfg.turn_rad_per_speed_ms)
            return Move(self._return_turn, self.cfg.turn_speed, self._clamp_ms(ms, self.cfg.look_step_ms))
        # Memory says the ball is in view or already passed, but it is not seen, which usually
        # means the turn rate is overestimated and the robot under-rotated.
        self.reason = "ball not where memory says, keep turning"
        return Move(self._return_turn, self.cfg.turn_speed, self.cfg.look_step_ms)

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
        # Waiting covers missed detections; a ball that left over an edge will
        # not come back on its own, so that case searches at once.
        if not self._exit_side and self._misses <= self.cfg.ball_lost_grace:
            self.reason = "ball missing, waiting for a fresh frame"
            return Wait(self.cfg.wait_ms)
        return self._lose_ball("ball rolled out of view" if self._exit_side else "ball lost")

    def _lose_ball(self, reason: str) -> Action:
        """Every route that gives up on the ball comes here, so search_when_lost applies to all."""
        if not self.cfg.search_when_lost:
            return Halt(reason)
        self._enter(Phase.SEARCH, reason)
        return self._search(None, None)

    def _turn_toward(self, ball: Detection) -> Move:
        self.reason = "centre the ball"
        error = ball.x - 0.5
        ms = abs(error) * self.cfg.turn_ms_per_unit * self._centring.gain
        speed = self.cfg.turn_speed
        if ms < self.cfg.min_pulse_ms and self._centring.shrunk:
            # The pulse cannot get shorter, so turn slower, down to min_turn_speed. Never at the
            # calibrated gain, where the shortest pulse already lands inside the centre band.
            speed = max(self.cfg.min_turn_speed, int(speed * ms / self.cfg.min_pulse_ms))
        self._centring.turned(error)
        return Move("rotate_right" if error > 0 else "rotate_left", speed,
                    self._clamp_ms(ms, self.cfg.max_turn_ms))

    def _clamp_ms(self, ms: float, upper: int) -> int:
        return int(max(self.cfg.min_pulse_ms, min(upper, ms)))

    def _enter(self, phase: Phase, reason: str) -> None:
        if phase is Phase.SEARCH:
            # A search can also end through notify_boundary and recover, which never pass
            # the found-ball branch, so its lap budget, turn and sightings reset here.
            self._search_pulses = 0
            self._search_turn = None
            self._sightings.clear()
        elif phase is Phase.SCAN:
            self._return_turn = None
        self.phase = phase
        self.reason = reason
        self._phase_pulses = 0
