"""Score the unchanged controller when the simulated robot differs from what the controller assumes.

A large drop for one variant marks the calibration to do first on the real robot.

    conda run -n ML python robocar/sim/robustness.py --episodes 150
"""
import argparse
import dataclasses
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from control import ControlConfig  # noqa: E402
from run_sim import run_one  # noqa: E402
from sim2d import SimParams  # noqa: E402

BASE = SimParams()
VARIANTS = {
    "as assumed": {},
    "camera FOV 55 deg": {"hfov_deg": 55.0, "vfov_deg": 42.0},
    "camera FOV 90 deg": {"hfov_deg": 90.0, "vfov_deg": 68.0},
    "turns 40% slower": {"max_turn_rps": BASE.max_turn_rps * 0.6},
    "turns 40% faster": {"max_turn_rps": BASE.max_turn_rps * 1.4},
    "strafe half as effective": {"lateral_factor": BASE.lateral_factor * 0.5},
    "motion noise x2": {"motion_noise": BASE.motion_noise * 2},
    "pixel noise x3": {"pixel_noise": BASE.pixel_noise * 3},
    "detector misses more": {"detect_far_m": BASE.detect_far_m * 0.5},
    "camera tilted 10 deg less": {"cam_pitch_deg": BASE.cam_pitch_deg - 10},
    "FOMO 12x12 grid": {"fomo_cells": 12},
    "phantom balls 5%": {"false_ball_rate": 0.05},
    "60 ms dead per pulse": {"dead_ms": 60},
    "left turns 30% slow, right fast": {"turn_asymmetry": -0.3},
    "drives and strafes veer": {"forward_yaw": 1.0, "strafe_yaw": 2.0},
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=150)
    args = parser.parse_args()
    config = ControlConfig()
    for name, changes in VARIANTS.items():
        params = dataclasses.replace(BASE, **changes)
        outcomes = [run_one(seed, config, params=params) for seed in range(args.episodes)]
        times = [o.time_s for o in outcomes if o.scored]
        median = statistics.median(times) if times else float("nan")
        print(f"{name:28s} scored {len(times):3d}/{args.episodes}  median {median:5.1f}s")


if __name__ == "__main__":
    main()
