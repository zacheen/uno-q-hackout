"""Turn calibration shared by MODE = "turn_check" on the robot and tools/turn_calc.py on a laptop."""
import math
from dataclasses import dataclass

from control import ControlConfig

TURN_CHECK_PULSES = 10
SLOW_TURN_PULSES = 3
# The shortest of these that visibly turns the robot is the pulse_dead_ms estimate. Reading one
# step long is safe, since sending up to twice the true dead time cost nothing in simulation.
DEAD_LADDER_MS = (20, 40, 60, 80, 100, 130, 160, 200)
# Real turns about a third faster than assumed, inside the 0.9 to 1.5 times the assumed rate
# that scored alike in simulation, and a quarter slower still inside it.
SAFETY_FACTOR = 0.75


@dataclass(frozen=True)
class TurnCalibration:
    turn_rad_per_speed_ms: float
    right_turn_scale: float
    pulse_dead_ms: int
    # Centring settles only if its slowest turn fits inside the centre band. Otherwise it rocks
    # across the ball, as in simulation at twice the assumed rate. The slowest turn is reckoned
    # as min_pulse_ms plus all of pulse_dead_ms at min_turn_speed, because the ladder can read
    # the dead time long and the surplus then turns the robot.
    slowest_turn_rad: float
    centre_band_rad: float
    settling_min_turn_speed: int

    @property
    def centring_settles(self) -> bool:
        return self.slowest_turn_rad < self.centre_band_rad


def calibrate(cfg: ControlConfig, left_deg: float, end_deg: float, dead_ms: int) -> TurnCalibration:
    """New turn settings from one turn_check run made with cfg.

    left_deg is how far the left look steps turned. end_deg is where the right look steps left
    the robot, measured from its starting direction and positive to the left. dead_ms is the
    shortest ladder pulse that visibly turned it.
    """
    right_deg = left_deg - end_deg
    turned = {"rotate_left": left_deg, "rotate_right": right_deg}
    rates = {}
    for direction, degrees in turned.items():
        moving_ms = cfg.pulse_ms(direction, cfg.look_step_ms) - dead_ms
        if degrees <= 0 or moving_ms <= 0:
            raise ValueError(f"{direction} turned {degrees:.0f} degrees in {moving_ms} ms of motion, "
                             "which cannot be a measurement")
        rates[direction] = math.radians(degrees) / (TURN_CHECK_PULSES * cfg.turn_speed * moving_ms)
    real_rate = rates["rotate_left"]
    band = 2 * math.atan(2 * cfg.center_band * math.tan(math.radians(cfg.hfov_deg) / 2))
    return TurnCalibration(
        turn_rad_per_speed_ms=real_rate * SAFETY_FACTOR,
        right_turn_scale=real_rate / rates["rotate_right"],
        pulse_dead_ms=dead_ms,
        slowest_turn_rad=(cfg.min_pulse_ms + dead_ms) * cfg.min_turn_speed * real_rate,
        centre_band_rad=band,
        settling_min_turn_speed=int(band / ((cfg.min_pulse_ms + dead_ms) * real_rate)),
    )
