import os
import stat
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout
from typing import Optional, Protocol, Sequence

import cv2
import numpy as np

from camera import LatestFrameReader
from control import BALL, GOAL, Detection, DetectorTimeout, Observation


class Detector(Protocol):
    def detect(self, bgr: np.ndarray) -> Sequence[Detection]: ...

    def close(self) -> None: ...


class EimDetector:
    """Runs an Edge Impulse .eim object-detection model through the edge_impulse_linux SDK."""

    def __init__(self, model_path: str, min_score: float = 0.5, timeout_s: float = 1.0,
                 required_labels: Sequence[str] = (BALL,)) -> None:
        # Imported here so the OpenCV fallback still works when the SDK fails to install.
        from edge_impulse_linux.image import ImageImpulseRunner

        # Unzipping on the board (python3 -m zipfile) does not restore the executable bit,
        # and a ZIP built on Windows may not carry it either.
        os.chmod(model_path, os.stat(model_path).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        self._runner = ImageImpulseRunner(model_path)
        params = self._runner.init()["model_parameters"]
        labels = {_canonical(label) for label in params["labels"]}
        missing = [label for label in required_labels if label not in labels]
        if missing:
            self._runner.stop()
            raise ValueError(f"model {model_path} lacks labels {missing}, has {params['labels']}")
        # Box coordinates come back in model-input pixels, not camera pixels.
        self._input_w = float(params["image_input_width"])
        self._input_h = float(params["image_input_height"])
        self._min_score = min_score
        self._timeout_s = timeout_s
        # The SDK's socket has no timeout, so a stuck .eim would block classify() forever.
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="eim")

    def detect(self, bgr: np.ndarray) -> Sequence[Detection]:
        future = self._pool.submit(self._classify, bgr)
        try:
            return future.result(timeout=self._timeout_s)
        except FutureTimeout as exc:
            raise DetectorTimeout(f"inference exceeded {self._timeout_s}s") from exc

    def _classify(self, bgr: np.ndarray) -> Sequence[Detection]:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        features, _ = self._runner.get_features_from_image_auto_studio_settings(rgb)
        result = self._runner.classify(features)["result"]
        detections = []
        for box in result.get("bounding_boxes", []):
            if box["value"] < self._min_score:
                continue
            detections.append(Detection(
                label=_canonical(box["label"]),
                x=(box["x"] + box["width"] / 2) / self._input_w,
                y=(box["y"] + box["height"] / 2) / self._input_h,
                score=float(box["value"]),
            ))
        return detections

    def close(self) -> None:
        self._runner.stop()
        self._pool.shutdown(wait=False, cancel_futures=True)


class HsvBallDetector:
    """Colour-threshold fallback that finds only the ball; tune the HSV range under venue lighting."""

    def __init__(self, lower_hsv: Sequence[int], upper_hsv: Sequence[int],
                 min_area_frac: float = 0.0015, max_area_frac: float = 0.5) -> None:
        self._lower = np.array(lower_hsv, dtype=np.uint8)
        self._upper = np.array(upper_hsv, dtype=np.uint8)
        self._min_area_frac = min_area_frac
        self._max_area_frac = max_area_frac
        self._kernel = np.ones((3, 3), np.uint8)

    def detect(self, bgr: np.ndarray) -> Sequence[Detection]:
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if best_area < area <= self._max_area_frac * w * h:
                best, best_area = contour, area
        if best is None or best_area < self._min_area_frac * w * h:
            return []
        m = cv2.moments(best)
        # Fill ratio against the enclosing circle stands in for a confidence score.
        (_, _), radius = cv2.minEnclosingCircle(best)
        score = min(1.0, best_area / (np.pi * radius * radius)) if radius > 0 else 0.0
        return [Detection(label=BALL, x=m["m10"] / m["m00"] / w, y=m["m01"] / m["m00"] / h, score=score)]

    def close(self) -> None:
        pass


class CameraPerception:
    """Turns the newest camera frame into an Observation holding the best ball and goal detections."""

    def __init__(self, reader: LatestFrameReader, detector: Detector, skip_frames: int = 1) -> None:
        self._reader = reader
        self._detector = detector
        # The camera double-buffers, so the first frame after a move may have been captured mid-move.
        self._skip_frames = skip_frames
        self._last_jpeg = b""

    def observe(self, wait_s: float) -> Optional[Observation]:
        """Wait for a frame that arrived after this call plus skip_frames; None when none arrives in time."""
        current = self._reader.latest()
        after_seq = (current.seq if current is not None else 0) + self._skip_frames
        deadline = time.monotonic() + wait_s
        while True:
            frame = self._reader.wait_for_frame_after(after_seq, max(0.0, deadline - time.monotonic()))
            if frame is None:
                return None
            after_seq = frame.seq
            # A camera resending identical bytes has frozen, and a corrupt JPEG shows nothing;
            # counting either as a frame would read as "ball missing" instead of "no video".
            if frame.jpeg == self._last_jpeg:
                continue
            bgr = frame.decode()
            if bgr is not None:
                break
        self._last_jpeg = frame.jpeg
        detections = self._detector.detect(bgr)
        return Observation(
            ball=_best(detections, BALL),
            goal=_best(detections, GOAL),
            frame_age_s=time.monotonic() - frame.received_at,
            seq=frame.seq,
        )

    def close(self) -> None:
        """Closes the detector; the reader is shared with its creator, which stops it."""
        self._detector.close()


def _canonical(label: str) -> str:
    """BALL or GOAL for a model label however it is spelt, since the organisers' dataset says soccer_ball."""
    key = label.lower().replace("_", "").replace("-", "").replace(" ", "")
    return {"soccerball": BALL, "goal": GOAL}.get(key, label)


def _best(detections: Sequence[Detection], label: str) -> Optional[Detection]:
    return max((d for d in detections if d.label == label), key=lambda d: d.score, default=None)
