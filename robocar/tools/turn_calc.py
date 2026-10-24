"""Turn the readings of one MODE = "turn_check" run into the turn settings for ControlConfig.

Run it before editing ControlConfig, because it reads the settings the robot ran with.

    conda run -n ML python robocar/tools/turn_calc.py --left-deg 300 --end-deg 20 --dead-ms 60
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from calibration import calibrate  # noqa: E402
from control import ControlConfig  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--left-deg", type=float, required=True, help="angle the left look steps turned")
    parser.add_argument("--end-deg", type=float, required=True,
                        help="where the right look steps left the robot, from its start and positive to the left")
    parser.add_argument("--dead-ms", type=int, required=True, help="shortest ladder pulse that visibly turned it")
    args = parser.parse_args()
    result = calibrate(ControlConfig(), args.left_deg, args.end_deg, args.dead_ms)
    print("Set these in ControlConfig in python/control.py, then deploy again.")
    print(f"    turn_rad_per_speed_ms: float = {result.turn_rad_per_speed_ms:.3g}")
    print(f"    pulse_dead_ms: int = {result.pulse_dead_ms}")
    print(f"    right_turn_scale: float = {result.right_turn_scale:.2f}")
    try:
        ControlConfig(turn_rad_per_speed_ms=result.turn_rad_per_speed_ms, pulse_dead_ms=result.pulse_dead_ms,
                      right_turn_scale=result.right_turn_scale)
    except ValueError as exc:
        print(f"[WARN] ControlConfig rejects these values ({exc}). Check the readings and run turn_check again.")
    if not result.centring_settles:
        print(f"[WARN] the slowest centring turn is about {math.degrees(result.slowest_turn_rad):.1f} degrees, wider "
              f"than the {math.degrees(result.centre_band_rad):.1f} degree centre band, so centring can rock across "
              f"the ball. Lower min_turn_speed to {result.settling_min_turn_speed} or less if the slow turns of "
              f"turn_check still move the robot at that speed, or else raise center_band.")


if __name__ == "__main__":
    main()
