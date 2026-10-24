import cv2
import numpy as np

from camera import Frame
from control import BALL, GOAL, Detection
from perception import CameraPerception, HsvBallDetector, _canonical, _goal_mouth, _nearest_ball

ORANGE_BGR = (0, 128, 255)


def _scene(cx: int, cy: int, radius: int = 18) -> np.ndarray:
    img = np.full((240, 320, 3), (60, 90, 60), np.uint8)
    cv2.circle(img, (cx, cy), radius, ORANGE_BGR, -1)
    return img


def _jpeg(img: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


def _detector() -> HsvBallDetector:
    return HsvBallDetector((5, 120, 80), (25, 255, 255))


class _FakeReader:
    """Serves a fixed frame list the way LatestFrameReader would, without sockets or sleeps.

    Like the real reader, latest() only moves forward: it reports the newest frame handed out.
    """

    def __init__(self, frames, latest=None):
        self._frames = frames
        self._latest = latest

    def latest(self):
        return self._latest

    def wait_for_frame_after(self, seq, timeout_s):
        frame = next((f for f in self._frames if f.seq > seq), None)
        if frame is not None:
            self._latest = frame
        return frame


def _frames(*payloads):
    return [Frame(jpeg=p, received_at=0.0, seq=i + 1) for i, p in enumerate(payloads)]


def test_hsv_detector_reports_ball_centroid_as_fractions():
    found = _detector().detect(_scene(240, 180))
    assert len(found) == 1
    ball = found[0]
    assert ball.label == BALL
    assert abs(ball.x - 0.75) < 0.02 and abs(ball.y - 0.75) < 0.02
    assert ball.score > 0.8


def test_hsv_detector_ignores_specks_and_empty_scenes():
    assert _detector().detect(_scene(160, 120, radius=2)) == []
    assert _detector().detect(np.full((240, 320, 3), (60, 90, 60), np.uint8)) == []


def test_observe_skips_the_frame_that_may_predate_the_last_move():
    # Frame 1 is newest when observe() starts, frame 2 may still show motion, frame 3 is used.
    frames = _frames(_jpeg(_scene(40, 120)), _jpeg(_scene(60, 120)), _jpeg(_scene(280, 120)))
    reader = _FakeReader(frames, latest=frames[0])
    obs = CameraPerception(reader, _detector(), skip_frames=1).observe(wait_s=1.0)
    assert obs is not None and obs.seq == 3 and obs.ball.x > 0.5


def test_capture_tool_skips_frozen_and_corrupt_frames():
    import tempfile
    from pathlib import Path

    from capture_frames import capture

    a, b = _jpeg(_scene(40, 120)), _jpeg(_scene(280, 120))
    reader = _FakeReader(_frames(a, a, b"not a jpeg", b))
    reader.last_error = None
    with tempfile.TemporaryDirectory() as out:
        saved = capture(reader, Path(out), count=5, every_s=0.0, frame_timeout_s=0.1)
        assert saved == 2
        assert sorted(f.read_bytes() for f in Path(out).glob("*.jpg")) == sorted([a, b])


def test_frozen_or_corrupt_frames_count_as_no_new_frame():
    frame = _jpeg(_scene(160, 120))
    perception = CameraPerception(_FakeReader(_frames(frame, frame, b"not a jpeg")), _detector(), skip_frames=0)
    assert perception.observe(wait_s=1.0).seq == 1
    assert perception.observe(wait_s=1.0) is None


def test_model_labels_map_to_ball_and_goal_however_they_are_spelt():
    for label in ("soccerball", "soccer_ball", "Soccer Ball", "soccer-ball"):
        assert _canonical(label) == BALL
    assert _canonical("goal") == GOAL and _canonical("Goal") == GOAL
    assert _canonical("robot") == "robot"


def test_the_goal_is_the_point_between_the_two_posts():
    def post(x, y, score):
        return Detection(GOAL, x, y, score)

    ball = Detection(BALL, 0.5, 0.7, 0.9)
    mouth = _goal_mouth([post(0.3, 0.30, 0.9), ball, post(0.7, 0.34, 0.6)])
    assert abs(mouth.x - 0.5) < 1e-9 and abs(mouth.y - 0.32) < 1e-9 and mouth.score == 0.6
    # A single post, or two merged into one cell, is all the robot can go by.
    assert _goal_mouth([post(0.2, 0.3, 0.8), ball]) == post(0.2, 0.3, 0.8)
    assert _goal_mouth([ball]) is None


def test_the_nearest_ball_wins_over_a_far_one_with_a_higher_score():
    near = Detection(BALL, 0.3, 0.54, 0.7)
    far = Detection(BALL, 0.8, 0.12, 1.0)
    post = Detection(GOAL, 0.5, 0.9, 1.0)
    assert _nearest_ball([far, post, near]) == near
    assert _nearest_ball([Detection(BALL, 0.3, 0.54, 0.6), near]) == near
    assert _nearest_ball([post]) is None
