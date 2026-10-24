"""Turn the readings of one MODE = "turn_check" run into the turn settings for FIELD.

Run it before editing python/field_config.py, because it reads the settings the robot ran with.

    conda run -n ML python robocar/tools/turn_calc.py --left-deg 300 --end 20L --dead-ms 60
"""
import argparse
import dataclasses
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from calibration import calibrate  # noqa: E402
from field_config import FIELD  # noqa: E402


def parse_end(text: str) -> float:
    """Degrees left of the starting direction from a reading such as 20L, 15R or 0."""
    match = re.fullmatch(r"\s*(\d+(?:\.\d+)?)\s*([LlRr]?)\s*", text)
    if match is None or (not match.group(2) and float(match.group(1)) != 0):
        raise argparse.ArgumentTypeError(f"{text!r} is not a reading like 20L, 15R or 0")
    degrees = float(match.group(1))
    return -degrees if match.group(2) in ("R", "r") else degrees


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--left-deg", type=float, required=True, help="angle the left look steps turned")
    parser.add_argument("--end", type=parse_end, required=True,
                        help="where the right look steps left the robot against its starting direction, "
                             "for example 20L for 20 degrees to its left, 15R or 0")
    parser.add_argument("--dead-ms", type=int, required=True,
                        help="shortest ladder pulse after which the chassis had clearly turned")
    args = parser.parse_args()
    cfg = FIELD
    result = calibrate(cfg, args.left_deg, args.end, args.dead_ms)
    print("Set these in FIELD in python/field_config.py, then deploy again.")
    print(f"    turn_rad_per_speed_ms={result.turn_rad_per_speed_ms:.3g},")
    print(f"    pulse_dead_ms={result.pulse_dead_ms},")
    print(f"    right_turn_scale={result.right_turn_scale:.2f},")
    print(f"    center_band={result.center_band:.2f},")
    push_band = max(cfg.push_band, result.center_band)
    if push_band > cfg.push_band:
        print(f"    push_band={push_band:.2f},")
        print("Turns this fast make centring coarse. Lowering turn_speed and min_turn_speed by 30 each and "
              "running turn_check again would keep center_band narrower.")
    print(f"center_band fits the slowest centring turn, about {math.degrees(result.slowest_turn_rad):.1f} degrees, "
          f"so centring does not rock across the ball.")
    try:
        dataclasses.replace(cfg, turn_rad_per_speed_ms=result.turn_rad_per_speed_ms, pulse_dead_ms=result.pulse_dead_ms,
                            right_turn_scale=result.right_turn_scale, center_band=result.center_band,
                            push_band=push_band)
    except ValueError as exc:
        print(f"[WARN] ControlConfig rejects these values ({exc}). Check the readings and run turn_check again.")


if __name__ == "__main__":
    main()
