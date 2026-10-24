"""Turn calibration shared by MODE = "turn_check" on the robot and tools/turn_calc.py on a laptop."""
import math
from dataclasses import dataclass

from control import ControlConfig

TURN_CHECK_PULSES = 10
# The check pulses aim at one full turn when the current settings are right, so the operator reads
# how far the robot ends from its start rather than an arbitrary angle.
TURN_CHECK_DEG = 360.0
# Each slow turn reverses the last, as centring does after an overshoot, so gear backlash and
# break-away friction are part of what they show.
SLOW_TURNS = ("rotate_right", "rotate_left", "rotate_right")
# The shortest of these that visibly turns the robot is the pulse_dead_ms estimate. Reading one
# step long is safe, since center_band below widens to cover the surplus, and in a simulation of
# the whole procedure readings up to twice the true dead time lost no goals.
DEAD_LADDER_MS = (20, 40, 60, 80, 100, 130, 160, 200)
# Real turns about a third faster than assumed, inside the 0.9 to 1.5 times the assumed rate
# that scored alike in simulation, and a quarter slower still inside it.
SAFETY_FACTOR = 0.75


def check_pulse_ms(cfg: ControlConfig) -> int:
    """Length of each check pulse as the controller sizes it, before ControlConfig.pulse_ms."""
    real_rate = cfg.turn_rad_per_speed_ms / SAFETY_FACTOR
    return round(math.radians(TURN_CHECK_DEG / TURN_CHECK_PULSES) / (cfg.turn_speed * real_rate))


@dataclass(frozen=True)
class TurnCalibration:
    turn_rad_per_speed_ms: float
    right_turn_scale: float
    pulse_dead_ms: int
    # Centring settles only if its slowest turn fits inside the centre band. Otherwise it rocks
    # across the ball, as in simulation at twice the assumed rate. The slowest turn is reckoned
    # as min_pulse_ms plus all of pulse_dead_ms at min_turn_speed, because the ladder can read
    # the dead time long and the surplus then turns the robot. Widening the band to fit it
    # scored as well in simulation as slowing the turn, without the risk of a stall, so
    # center_band never narrows here and widens when the slowest turn needs it.
    slowest_turn_rad: float
    center_band: float


def calibrate(cfg: ControlConfig, left_deg: float, end_deg: float, dead_ms: int) -> TurnCalibration:
    """New turn settings from one turn_check run made with cfg.

    left_deg is how far the left check pulses turned. end_deg is where the right ones left the
    robot, measured from its starting direction and positive to the left. dead_ms is the
    shortest ladder pulse that visibly turned it.
    """
    right_deg = left_deg - end_deg
    turned = {"rotate_left": left_deg, "rotate_right": right_deg}
    rates = {}
    for direction, degrees in turned.items():
        moving_ms = cfg.pulse_ms(direction, check_pulse_ms(cfg)) - dead_ms
        if degrees <= 0 or moving_ms <= 0:
            raise ValueError(f"{direction} turned {degrees:.0f} degrees in {moving_ms} ms of motion, "
                             "which cannot be a measurement")
        rates[direction] = math.radians(degrees) / (TURN_CHECK_PULSES * cfg.turn_speed * moving_ms)
    real_rate = rates["rotate_left"]
    half_fov_tan = math.tan(math.radians(cfg.hfov_deg) / 2)
    slowest = (cfg.min_pulse_ms + dead_ms) * cfg.min_turn_speed * real_rate
    return TurnCalibration(
        turn_rad_per_speed_ms=real_rate * SAFETY_FACTOR,
        right_turn_scale=real_rate / rates["rotate_right"],
        pulse_dead_ms=dead_ms,
        slowest_turn_rad=slowest,
        # A tenth wider than the slowest turn, since the turn rate itself varies between pulses,
        # and rounded up to a value that reads well in ControlConfig.
        center_band=max(cfg.center_band, math.ceil(110 * math.tan(slowest / 2) / (2 * half_fov_tan)) / 100),
    )
