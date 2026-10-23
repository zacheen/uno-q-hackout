"""Run the real controller and run loop against the 2D simulator many times and summarise.

    conda run -n ML python robocar/sim/run_sim.py --episodes 200
    conda run -n ML python robocar/sim/run_sim.py --episodes 1 --seed 7 --trace
"""
import argparse
import statistics
import sys
from collections import Counter
from pathlib import Path
from typing import NamedTuple, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from control import BallGoalController, ControlConfig, Observation  # noqa: E402
from runtime import EpisodeConfig, RunLog, run_episode  # noqa: E402
from sim2d import BallOut, Scored, SimExecutor, SimLineSensor, SimParams, SimPerception, random_world  # noqa: E402


class Outcome(NamedTuple):
    scored: bool
    time_s: float
    reason: str


class TracingController(BallGoalController):
    def step(self, obs: Observation):
        action = super().step(obs)
        ball = f"{obs.ball.x:.2f},{obs.ball.y:.2f}" if obs.ball else "-"
        goal = f"{obs.goal.x:.2f},{obs.goal.y:.2f}" if obs.goal else "-"
        print(f"ball {ball:>9} goal {goal:>9} {self.phase.value:>8} {action}  {self.reason}")
        return action


def run_one(seed: int, config: ControlConfig, trace: bool = False, log_dir: Optional[Path] = None,
            params: Optional[SimParams] = None) -> Outcome:
    world = random_world(seed, params)
    controller = (TracingController if trace else BallGoalController)(config)
    log = RunLog(log_dir, clock=world.clock) if log_dir else None
    try:
        reason = run_episode(SimPerception(world), controller, SimExecutor(world), EpisodeConfig(),
                             boundary=SimLineSensor(world), log=log, clock=world.clock)
    except Scored:
        return Outcome(True, world.time_s, "scored")
    except BallOut:
        return Outcome(False, world.time_s, "ball left the field outside the goal")
    finally:
        if log is not None:
            log.close()
    return Outcome(False, world.time_s, reason)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--log-dir", type=Path, help="write one CSV per episode, readable by tools/plot_run.py")
    args = parser.parse_args()

    config = ControlConfig()
    results = [run_one(args.seed + i, config, args.trace, args.log_dir) for i in range(args.episodes)]
    times = [r.time_s for r in results if r.scored]
    print(f"scored {len(times)}/{len(results)}")
    if times:
        print(f"time to score: median {statistics.median(times):.1f}s, "
              f"p90 {sorted(times)[int(0.9 * (len(times) - 1))]:.1f}s")
    for reason, count in Counter(r.reason for r in results if not r.scored).most_common():
        print(f"  {count:4d}  {reason}")


if __name__ == "__main__":
    main()
