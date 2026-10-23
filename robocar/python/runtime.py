import csv
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol, TextIO, assert_never

from control import Action, BallGoalController, DetectorTimeout, Halt, Move, Observation, Wait


class Perception(Protocol):
    def observe(self, wait_s: float) -> Optional[Observation]: ...


class Executor(Protocol):
    def execute(self, move: Move) -> None: ...

    def wait(self, ms: int) -> None: ...


class BoundaryGuard(Protocol):
    def triggered(self) -> bool: ...


@dataclass(frozen=True)
class EpisodeConfig:
    """Safety limits shared by the real robot and the simulator, so the simulator tests the real values."""

    time_limit_s: float = 60.0
    max_frame_age_s: float = 0.5
    observe_wait_s: float = 1.0
    back_off: Move = Move("backward", 130, 250)
    # A guard that keeps firing after this many back-offs in a row has either the wrong
    # black_bit or a back_off that cannot clear the tape, as when the robot crossed the
    # edge sideways, and would otherwise reverse until the time limit.
    max_boundary_backoffs: int = 3


class RunLog:
    """One CSV row per decision, so a run can be replayed when tuning thresholds."""

    FIELDS = ("t", "seq", "frame_age_s", "ball_x", "ball_y", "ball_score",
              "goal_x", "goal_y", "goal_score", "phase", "action", "reason")

    def __init__(self, directory: Path, clock: Callable[[], float] = time.monotonic) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = directory / f"run-{stamp}.csv"
        suffix = 1
        while path.exists():
            path = directory / f"run-{stamp}-{suffix}.csv"
            suffix += 1
        self._file: TextIO = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self._clock = clock
        self._start = clock()
        self.path = path

    def record(self, obs: Optional[Observation], phase: str, action: str, reason: str) -> None:
        row = {"t": f"{self._clock() - self._start:.3f}", "phase": phase, "action": action, "reason": reason}
        if obs is not None:
            row.update(seq=obs.seq, frame_age_s=f"{obs.frame_age_s:.3f}")
            for name, det in (("ball", obs.ball), ("goal", obs.goal)):
                if det is not None:
                    row.update({f"{name}_x": f"{det.x:.3f}", f"{name}_y": f"{det.y:.3f}",
                                f"{name}_score": f"{det.score:.2f}"})
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def run_episode(perception: Perception, controller: BallGoalController, executor: Executor,
                cfg: EpisodeConfig = EpisodeConfig(), *, boundary: Optional[BoundaryGuard] = None,
                log: Optional[RunLog] = None, clock: Callable[[], float] = time.monotonic) -> str:
    """Run one attempt until the controller halts or a safety check fails; returns the reason.

    Every safety failure ends the attempt instead of retrying, so a restart is always
    a deliberate BOOT press.
    """
    start = clock()
    backoffs_in_a_row = 0
    while True:
        if clock() - start > cfg.time_limit_s:
            return _finish(log, None, controller, "time limit reached")
        if boundary is not None and boundary.triggered():
            backoffs_in_a_row += 1
            if backoffs_in_a_row > cfg.max_boundary_backoffs:
                return _finish(log, None, controller,
                               f"line sensor still on tape after {cfg.max_boundary_backoffs} back-offs")
            _record(log, None, controller, cfg.back_off, "line sensor saw the field edge")
            executor.execute(cfg.back_off)
            controller.notify_boundary()
            continue
        backoffs_in_a_row = 0
        try:
            obs = perception.observe(cfg.observe_wait_s)
        except DetectorTimeout as exc:
            return _finish(log, None, controller, f"detector timeout: {exc}")
        if obs is None:
            return _finish(log, None, controller, "no new camera frame")
        if obs.frame_age_s > cfg.max_frame_age_s:
            return _finish(log, obs, controller, f"stale frame, {obs.frame_age_s:.2f}s old")
        action = controller.step(obs)
        _record(log, obs, controller, action, controller.reason)
        if isinstance(action, Halt):
            return action.reason
        if isinstance(action, Wait):
            executor.wait(action.ms)
        else:
            executor.execute(action)


def _describe(action: Action) -> str:
    if isinstance(action, Move):
        return f"{action.direction} s{action.speed} {action.ms}ms"
    if isinstance(action, Wait):
        return f"wait {action.ms}ms"
    if isinstance(action, Halt):
        return "halt"
    assert_never(action)


def _record(log: Optional[RunLog], obs: Optional[Observation], controller: BallGoalController,
            action: Action, reason: str) -> None:
    if log is not None:
        log.record(obs, controller.phase.value, _describe(action), reason)


def _finish(log: Optional[RunLog], obs: Optional[Observation], controller: BallGoalController,
            reason: str) -> str:
    _record(log, obs, controller, Halt(reason), reason)
    return reason
