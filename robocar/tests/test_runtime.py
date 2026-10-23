import csv
import tempfile
from pathlib import Path

from control import BALL, BallGoalController, ControlConfig, Detection, DetectorTimeout, Move, Observation, Phase
from hardware import LineSensorGuard
from runtime import EpisodeConfig, RunLog, run_episode

# A back-off unlike any controller move, so counting it never picks up a recover pulse.
CFG = EpisodeConfig(time_limit_s=10.0, max_frame_age_s=0.5, observe_wait_s=0.1,
                    back_off=Move("backward", 99, 77))


class _Script:
    """Perception fake that replays a fixed list of observations or exceptions."""

    def __init__(self, *items):
        self._items = list(items)

    def observe(self, wait_s):
        item = self._items.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


class _Recorder:
    def __init__(self):
        self.calls = []

    def execute(self, move):
        self.calls.append(move)

    def wait(self, ms):
        self.calls.append(("wait", ms))


class _Clock:
    def __init__(self, step_s):
        self._t, self._step = 0.0, step_s

    def __call__(self):
        self._t += self._step
        return self._t


class _Edge:
    def __init__(self, fires):
        self._fires = list(fires)

    def triggered(self):
        return self._fires.pop(0) if self._fires else False


def _obs(age=0.05, ball=(0.5, 0.3)):
    return Observation(ball=Detection(BALL, *ball, 0.9) if ball else None, goal=None, frame_age_s=age, seq=1)


def _run(perception, executor=None, controller=None, **kwargs):
    return run_episode(perception, controller or BallGoalController(ControlConfig()),
                       executor or _Recorder(), CFG, **kwargs)


def test_ends_when_no_frame_arrives():
    assert _run(_Script(None)) == "no new camera frame"


def test_ends_on_a_stale_frame_without_moving():
    executor = _Recorder()
    assert _run(_Script(_obs(age=0.9)), executor).startswith("stale frame")
    assert executor.calls == []


def test_ends_on_detector_timeout():
    assert _run(_Script(DetectorTimeout("1.0s"))).startswith("detector timeout")


def test_ends_at_the_time_limit():
    assert _run(_Script(*[_obs()] * 50), clock=_Clock(step_s=3.0)) == "time limit reached"


def test_missing_ball_waits_instead_of_moving():
    executor = _Recorder()
    # The ball ends the opening search and is approached, then vanishes from the centre.
    _run(_Script(_obs(), _obs(), _obs(ball=None), None), executor)
    assert executor.calls[-1][0] == "wait"


def test_boundary_backs_off_and_puts_the_controller_into_recovery():
    executor = _Recorder()
    controller = BallGoalController(ControlConfig())
    reason = _run(_Script(None), executor, controller, boundary=_Edge([True]))
    assert executor.calls == [CFG.back_off] and reason == "no new camera frame"
    assert controller.phase is Phase.RECOVER


def test_a_guard_that_never_clears_ends_the_run_instead_of_reversing_forever():
    executor = _Recorder()
    reason = _run(_Script(), executor, boundary=_Edge([True] * 10))
    assert reason.startswith("line sensor still on tape")
    assert executor.calls == [CFG.back_off] * CFG.max_boundary_backoffs


def test_back_off_count_resets_once_the_robot_leaves_the_tape():
    executor = _Recorder()
    edge = _Edge([True] * CFG.max_boundary_backoffs + [False] + [True] * CFG.max_boundary_backoffs)
    reason = _run(_Script(_obs(), None), executor, boundary=edge)
    assert reason == "no new camera frame"
    assert executor.calls.count(CFG.back_off) == 2 * CFG.max_boundary_backoffs


def test_halt_ends_the_run_and_the_last_log_row_carries_its_reason():
    with tempfile.TemporaryDirectory() as tmp:
        log = RunLog(Path(tmp))
        controller = BallGoalController(ControlConfig(search_when_lost=False))
        reason = _run(_Script(_obs(), _obs(), *[_obs(ball=None)] * 5), controller=controller, log=log)
        log.close()
        with log.path.open(encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    assert reason == "ball lost"
    assert rows[-1]["action"] == "halt" and rows[-1]["reason"] == "ball lost"
    # Columns tools/plot_run.py reads by name; spelled out so renaming one breaks this test.
    assert {"t", "phase", "reason", "ball_x", "ball_y", "goal_x"} <= set(rows[0])


class _SensorRobot:
    def __init__(self, sensors):
        self._sensors = sensors

    def read_sensors(self):
        return self._sensors


def test_robot_executor_wait_notices_the_boot_stop():
    from arduino import app_utils
    from hardware import RobotExecutor
    from robot_client import MiniAutoRobot, ProgramStopped

    executor = RobotExecutor(MiniAutoRobot())
    app_utils.Bridge.sensors["program_enabled"] = False
    try:
        executor.wait(1)
    except ProgramStopped:
        pass
    else:
        raise AssertionError("wait() ignored a disabled program")
    app_utils.Bridge.sensors["program_enabled"] = True
    try:
        executor.wait(1)
        app_utils.calls.clear()
        executor.execute(Move("forward", 100, 1))
        assert ("drive", ("forward", 100, 1)) in app_utils.calls
    finally:
        app_utils.Bridge.sensors["program_enabled"] = False


def test_line_guard_reads_each_channel_and_ignores_failed_reads():
    def guard(bits, ok=True, black_bit=1):
        return LineSensorGuard(_SensorRobot({"line_ok": ok, "line_digital": bits}), black_bit=black_bit)

    assert guard([0, 0, 1, 0]).triggered()
    assert not guard([0, 0, 0, 0]).triggered()
    assert guard([1, 1, 1, 0], black_bit=0).triggered()
    assert not guard([1, 1, 1, 1], ok=False).triggered()


def test_line_guard_accepts_the_real_sensor_payload():
    from arduino import app_utils
    from robot_client import MiniAutoRobot

    app_utils.Bridge.sensors["line_digital"] = [0, 1, 0, 0]
    try:
        assert LineSensorGuard(MiniAutoRobot(), black_bit=1).triggered()
    finally:
        app_utils.Bridge.sensors["line_digital"] = [0, 0, 0, 0]
