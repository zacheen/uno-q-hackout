import threading
import time
from dataclasses import dataclass
from typing import Optional
from urllib.request import urlopen

import cv2
import numpy as np


@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    received_at: float  # time.monotonic() when the last byte arrived
    seq: int

    def decode(self) -> Optional[np.ndarray]:
        """BGR image, or None when the JPEG is corrupt."""
        return cv2.imdecode(np.frombuffer(self.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)


class LatestFrameReader:
    """Reads the camera's MJPEG stream on a background thread and keeps only the newest frame.

    The camera serves one stream at a time, so close any browser preview first.
    The firmware sends no capture time, so frame age can only be measured from
    arrival; a frame that arrives late after a network stall still looks fresh.
    """

    def __init__(self, url: str, read_timeout_s: float = 2.0, reconnect_delay_s: float = 0.5) -> None:
        self._url = url
        self._read_timeout_s = read_timeout_s
        self._reconnect_delay_s = reconnect_delay_s
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._latest: Optional[Frame] = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.last_error: Optional[str] = None

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="mjpeg-reader", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def latest(self) -> Optional[Frame]:
        with self._lock:
            return self._latest

    def wait_for_frame_after(self, seq: int, timeout_s: float) -> Optional[Frame]:
        """Block until a frame with a sequence number above seq arrives, or return None on timeout."""
        deadline = time.monotonic() + timeout_s
        with self._new_frame:
            while self._latest is None or self._latest.seq <= seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._new_frame.wait(remaining)
            return self._latest

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with urlopen(self._url, timeout=self._read_timeout_s) as stream:
                    self._read_parts(stream)
            except Exception as exc:  # network errors must never kill the reader thread
                self.last_error = f"{type(exc).__name__}: {exc}"
            self._stop.wait(self._reconnect_delay_s)

    def _read_parts(self, stream) -> None:
        while not self._stop.is_set():
            length = self._read_part_header(stream)
            if length is None:
                return
            jpeg = stream.read(length)
            if len(jpeg) != length:
                return
            with self._new_frame:
                self._seq += 1
                self._latest = Frame(jpeg=jpeg, received_at=time.monotonic(), seq=self._seq)
                self._new_frame.notify_all()

    @staticmethod
    def _read_part_header(stream) -> Optional[int]:
        """Skip to the next part and return its Content-Length, or None at end of stream."""
        length = None
        while True:
            line = stream.readline()
            if not line:
                return None
            text = line.strip()
            if not text:
                # The blank line after the part headers; blank lines before the boundary are skipped.
                if length is not None:
                    return length
                continue
            if text.lower().startswith(b"content-length:"):
                length = int(text.split(b":", 1)[1])
