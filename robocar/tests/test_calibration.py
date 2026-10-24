import math

from calibration import SAFETY_FACTOR, TURN_CHECK_PULSES, calibrate
from control import ControlConfig


def _readings(cfg: ControlConfig, left_rate: float, right_rate: float, dead_ms: int) -> tuple:
    """What turn_check shows on a robot with these true rates and dead time, run with cfg."""
    def degrees(direction: str, rate: float) -> float:
        moving_ms = cfg.pulse_ms(direction, cfg.look_step_ms) - dead_ms
        return math.degrees(TURN_CHECK_PULSES * cfg.turn_speed * moving_ms * rate)
    left = degrees("rotate_left", left_rate)
    return left, left - degrees("rotate_right", right_rate)


def test_calibrate_recovers_the_true_robot_whatever_settings_the_check_ran_with():
    left_rate, right_rate, dead = 1.1e-5, 0.8e-5, 60
    for ran_with in (ControlConfig(), ControlConfig(pulse_dead_ms=30, right_turn_scale=1.2),
                     ControlConfig(turn_rad_per_speed_ms=7e-6, pulse_dead_ms=90, right_turn_scale=0.8)):
        left_deg, end_deg = _readings(ran_with, left_rate, right_rate, dead)
        result = calibrate(ran_with, left_deg, end_deg, dead)
        assert math.isclose(result.turn_rad_per_speed_ms, left_rate * SAFETY_FACTOR, rel_tol=1e-9)
        assert math.isclose(result.right_turn_scale, left_rate / right_rate, rel_tol=1e-9)
        assert result.pulse_dead_ms == dead


def test_calibrated_settings_make_left_and_right_look_steps_turn_alike():
    left_rate, right_rate, dead = 1.1e-5, 0.8e-5, 60
    ran_with = ControlConfig()
    result = calibrate(ran_with, *_readings(ran_with, left_rate, right_rate, dead), dead)
    cfg = ControlConfig(turn_rad_per_speed_ms=result.turn_rad_per_speed_ms, pulse_dead_ms=result.pulse_dead_ms,
                        right_turn_scale=result.right_turn_scale)
    left = (cfg.pulse_ms("rotate_left", cfg.look_step_ms) - dead) * left_rate
    right = (cfg.pulse_ms("rotate_right", cfg.look_step_ms) - dead) * right_rate
    # Rounding the right pulse to whole milliseconds is the only difference left.
    assert math.isclose(left, right, rel_tol=0.01)
    assert math.isclose(left * cfg.turn_speed, cfg.look_step_rad / SAFETY_FACTOR, rel_tol=0.01)


def test_calibrate_flags_a_robot_whose_slowest_centring_turn_overshoots_the_centre_band():
    cfg = ControlConfig()
    slow = calibrate(cfg, *_readings(cfg, 1.2e-5, 1.2e-5, 0), 0)
    fast = calibrate(cfg, *_readings(cfg, 3e-5, 3e-5, 0), 0)
    assert slow.centring_settles and not fast.centring_settles
    assert fast.settling_min_turn_speed < cfg.min_turn_speed
    assert cfg.min_pulse_ms * fast.settling_min_turn_speed * 3e-5 <= fast.centre_band_rad
    # Dead time counts against settling, since a long ladder reading turns the robot further.
    assert calibrate(cfg, *_readings(cfg, 3e-5, 3e-5, 40), 40).settling_min_turn_speed < fast.settling_min_turn_speed


def test_calibrate_rejects_readings_that_cannot_come_from_the_check():
    cfg = ControlConfig()
    for left_deg, end_deg, dead in ((0, 0, 0), (300, 300, 0), (300, 0, cfg.look_step_ms)):
        try:
            calibrate(cfg, left_deg, end_deg, dead)
        except ValueError:
            continue
        raise AssertionError(f"accepted left {left_deg}, end {end_deg}, dead {dead}")
