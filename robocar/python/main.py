import time
from pathlib import Path
from typing import Callable, Optional

from arduino.app_utils import App

from calibration import DEAD_LADDER_MS, SLOW_TURNS, TURN_CHECK_DEG, TURN_CHECK_PULSES, check_pulse_ms
from camera import LatestFrameReader
from control import BallGoalController
from field_config import FIELD
from hardware import LineSensorGuard, RobotExecutor
from perception import CameraPerception, Detector, EimDetector, HsvBallDetector
from robot_client import MiniAutoRobot
from runtime import EpisodeConfig, RunLog, run_episode

# "motion_check" is the starter's single forward pulse, "turn_check" makes the turns that
# tools/turn_calc.py turns into the turn settings and checks min_turn_speed, "detect_only"
# prints detections with the motors idle, and "play" runs the full search, line-up and
# push routine.
MODE = "turn_check"

CAMERA_URL = "http://192.168.5.1:81/stream"
HERE = Path(__file__).resolve().parent
# make flash replaces python/ and sketch/ but keeps data/, so logs survive redeploys.
LOG_DIR = HERE.parent / "data" / "runs"

# Tune these in runtime.EpisodeConfig, not here, so the simulator keeps testing the same limits.
EPISODE = EpisodeConfig()
MODES = ("motion_check", "turn_check", "detect_only", "play")
DETECT_ONLY_S = 30.0
READ_PAUSE_S = 5.0
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
    # Run with the wheels on the floor. The pulses are sent the way the controller sends them, so
    # tools/turn_calc.py can undo whatever settings were in force.
    cfg = FIELD
    ms = check_pulse_ms(cfg)
    print(f"[INFO] {TURN_CHECK_PULSES} left turns of {ms} ms at speed {cfg.turn_speed}, which make exactly "
          f"{TURN_CHECK_DEG:.0f} degrees if the turn settings are right")
    for _ in range(TURN_CHECK_PULSES):
        robot.drive("rotate_left", speed=cfg.turn_speed, ms=cfg.pulse_ms("rotate_left", ms))
    print(f"[INFO] read the angle turned left. In {READ_PAUSE_S:.0f} s come {TURN_CHECK_PULSES} right turns")
    time.sleep(READ_PAUSE_S)
    for _ in range(TURN_CHECK_PULSES):
        robot.drive("rotate_right", speed=cfg.turn_speed, ms=cfg.pulse_ms("rotate_right", ms))
    print(f"[INFO] read where the robot ended against its starting direction, such as 20L, 15R or 0. "
          f"In {READ_PAUSE_S:.0f} s come {len(SLOW_TURNS)} turns back and forth at min_turn_speed "
          f"{cfg.min_turn_speed}, and each one must visibly move the robot")
    time.sleep(READ_PAUSE_S)
    for direction in SLOW_TURNS:
        robot.drive(direction, speed=cfg.min_turn_speed, ms=cfg.pulse_ms(direction, cfg.min_pulse_ms))
    print(f"[INFO] in {READ_PAUSE_S:.0f} s come left pulses of rising length. Note the shortest after which "
          f"the chassis has clearly turned, since a buzz or a twitch does not count")
    time.sleep(READ_PAUSE_S)
    for ms in DEAD_LADDER_MS:
        print(f"[INFO] {ms} ms")
        robot.drive("rotate_left", speed=cfg.turn_speed, ms=ms)
        time.sleep(1.5)
    print("[INFO] on the laptop run: conda run -n ML python robocar/tools/turn_calc.py "
          "--left-deg <angle> --end <end> --dead-ms <shortest>")


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
    controller = BallGoalController(FIELD)
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
