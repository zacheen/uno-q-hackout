import math
import time
from pathlib import Path
from typing import Callable, Optional

from arduino.app_utils import App

from camera import LatestFrameReader
from control import BallGoalController, ControlConfig
from hardware import LineSensorGuard, RobotExecutor
from perception import CameraPerception, Detector, EimDetector, HsvBallDetector
from robot_client import MiniAutoRobot
from runtime import EpisodeConfig, RunLog, run_episode

# "motion_check" is the starter's single forward pulse, "turn_check" turns search look
# steps to check turn_rad_per_speed_ms and then a few slow ones to check min_turn_speed,
# "detect_only" prints detections with the motors idle, and "play" runs the full search,
# line-up and push routine.
MODE = "motion_check"

CAMERA_URL = "http://192.168.5.1:81/stream"
HERE = Path(__file__).resolve().parent
# make flash replaces python/ and sketch/ but keeps data/, so logs survive redeploys.
LOG_DIR = HERE.parent / "data" / "runs"

# Tune these in runtime.EpisodeConfig, not here, so the simulator keeps testing the same limits.
EPISODE = EpisodeConfig()
MODES = ("motion_check", "turn_check", "detect_only", "play")
TURN_CHECK_PULSES = 10
SLOW_TURN_PULSES = 3
DETECT_ONLY_S = 30.0
LINE_GUARD_ENABLED = False  # enable only after confirming LineSensorGuard.black_bit on the field
# Used only when no .eim model loads; the default range is a guess for an orange ball.
HSV_LOWER = (5, 120, 80)
HSV_UPPER = (25, 255, 255)

robot = MiniAutoRobot()
reader = LatestFrameReader(CAMERA_URL)


def make_detector() -> Detector:
    models = sorted(HERE.glob("*.eim"))
    if not models:
        print("[WARN] no .eim model in python/, falling back to HSV colour detection")
        return HsvBallDetector(HSV_LOWER, HSV_UPPER)
    try:
        return EimDetector(str(models[0]))
    except Exception as exc:
        print(f"[WARN] {models[0].name} failed to load ({exc}), falling back to HSV colour detection")
        return HsvBallDetector(HSV_LOWER, HSV_UPPER)


def motion_check() -> None:
    # Start with raised wheels; calibrate this short pulse on each chassis.
    robot.drive("forward", speed=100, ms=200)


def turn_check() -> None:
    # Run with the wheels on the floor. The pulses are the search's own look steps, so the
    # motor spin-up lost between looks is part of what gets measured.
    cfg = ControlConfig()
    expected = TURN_CHECK_PULSES * cfg.turn_speed * cfg.look_step_ms * cfg.turn_rad_per_speed_ms
    print(f"[INFO] turning left {TURN_CHECK_PULSES} x {cfg.look_step_ms} ms at speed {cfg.turn_speed}, "
          f"the controller expects {math.degrees(expected):.0f} degrees in total")
    for _ in range(TURN_CHECK_PULSES):
        robot.drive("rotate_left", speed=cfg.turn_speed, ms=cfg.look_step_ms)
    print(f"[INFO] read the angle now. In 3 s come {SLOW_TURN_PULSES} right turns of {cfg.min_pulse_ms} ms "
          f"at min_turn_speed {cfg.min_turn_speed}, and each one must visibly move the robot")
    time.sleep(3.0)
    for _ in range(SLOW_TURN_PULSES):
        robot.drive("rotate_right", speed=cfg.min_turn_speed, ms=cfg.min_pulse_ms)


def detect_only(perception: CameraPerception) -> None:
    end = time.monotonic() + DETECT_ONLY_S
    while time.monotonic() < end and robot.is_running():
        obs = perception.observe(EPISODE.observe_wait_s)
        if obs is None:
            print(f"[WARN] no camera frame, last error: {reader.last_error}")
            continue
        print(f"seq {obs.seq} age {obs.frame_age_s:.2f}s ball {obs.ball} goal {obs.goal}")


def open_log() -> Optional[RunLog]:
    # Whether the app container may write next to python/ is unverified; a run must not depend on it.
    try:
        return RunLog(LOG_DIR)
    except OSError as exc:
        print(f"[WARN] run log disabled, cannot write {LOG_DIR}: {exc}")
        return None


def play(perception: CameraPerception) -> None:
    controller = BallGoalController(ControlConfig())
    guard = LineSensorGuard(robot) if LINE_GUARD_ENABLED else None
    log = open_log()
    try:
        reason = run_episode(perception, controller, RobotExecutor(robot), EPISODE, boundary=guard, log=log)
        print(f"[INFO] attempt ended: {reason}" + (f", log at {log.path}" if log else ""))
    finally:
        if log is not None:
            log.close()


def build_routine() -> tuple[Callable[[], None], Optional[CameraPerception]]:
    """Returns the routine for MODE and the perception it owns, which must be closed on exit."""
    if MODE not in MODES:
        raise ValueError(f"unknown MODE {MODE!r}, expected one of {MODES}")
    if MODE == "motion_check":
        return motion_check, None
    if MODE == "turn_check":
        return turn_check, None
    camera_perception = CameraPerception(reader, make_detector())
    reader.start()
    if MODE == "detect_only":
        return (lambda: detect_only(camera_perception)), camera_perception
    if MODE == "play":
        return (lambda: play(camera_perception)), camera_perception
    camera_perception.close()
    raise AssertionError(f"MODES lists {MODE!r} but build_routine has no branch for it")


active_perception: Optional[CameraPerception] = None
try:
    robot.stop()
    print(f"health  : {robot.health()}")
    print(f"sensors : {robot.read_sensors()}")
    print(f"mode    : {MODE}")
    wheels = "Put the robot on the floor" if MODE == "turn_check" else "Raise the wheels"
    print(f"{wheels}, then short-press the camera BOOT button to start.")
    routine, active_perception = build_routine()
    App.run(user_loop=lambda: robot.run_program(routine))
finally:
    try:
        robot.stop()  # motors first, before anything else that could fail
    finally:
        reader.stop()
        if active_perception is not None:
            # Stops the .eim child process when this block runs; a hard kill of the app skips it.
            active_perception.close()
