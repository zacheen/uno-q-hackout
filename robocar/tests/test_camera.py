import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from camera import LatestFrameReader
from capture_frames import capture

BOUNDARY = "123456789000000000000987654321"


def _jpeg(value: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((24, 32, 3), value, np.uint8))
    assert ok
    return buf.tobytes()


class _FirmwareLikeHandler(BaseHTTPRequestHandler):
    """Mimics the HiwonderCamStream framing: boundary, Content-Type, Content-Length, JPEG."""

    frames = 5
    delay_s = 0.02

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace;boundary={BOUNDARY}")
        self.end_headers()
        for i in range(self.frames):
            body = _jpeg(40 * (i + 1))
            self.wfile.write(f"\r\n--{BOUNDARY}\r\n".encode())
            self.wfile.write(f"Content-Type: image/jpeg\r\nContent-Length: {len(body)}\r\n\r\n".encode())
            self.wfile.write(body)
            self.wfile.flush()
            time.sleep(self.delay_s)

    def log_message(self, *args):
        pass

    def handle(self):
        # The reader hangs up mid-stream on purpose; that is not a test failure.
        try:
            super().handle()
        except ConnectionError:
            pass


def _serve():
    server = ThreadingHTTPServer(("127.0.0.1", 0), _FirmwareLikeHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://127.0.0.1:{server.server_address[1]}/stream"


def test_reader_parses_firmware_framing_and_keeps_newest():
    server, url = _serve()
    reader = LatestFrameReader(url, read_timeout_s=1.0, reconnect_delay_s=5.0)
    try:
        reader.start()
        first = reader.wait_for_frame_after(0, timeout_s=2.0)
        assert first is not None and first.seq >= 1
        assert first.decode() is not None
        time.sleep(0.3)
        latest = reader.latest()
        assert latest.seq == 5, latest.seq
        # Frame 5 was encoded with value 200, so the newest frame really is the last one sent.
        assert abs(int(latest.decode().mean()) - 200) <= 3
    finally:
        reader.stop()
        server.shutdown()


def test_wait_returns_none_when_no_newer_frame_arrives():
    server, url = _serve()
    reader = LatestFrameReader(url, read_timeout_s=1.0, reconnect_delay_s=5.0)
    try:
        reader.start()
        time.sleep(0.3)
        newest = reader.latest().seq
        assert reader.wait_for_frame_after(newest, timeout_s=0.2) is None
    finally:
        reader.stop()
        server.shutdown()


def test_capture_tool_saves_distinct_frames():
    server, url = _serve()
    reader = LatestFrameReader(url, read_timeout_s=1.0, reconnect_delay_s=5.0)
    try:
        reader.start()
        with tempfile.TemporaryDirectory() as out:
            saved = capture(reader, Path(out), count=3, every_s=0.0, frame_timeout_s=1.0)
            files = sorted(Path(out).glob("*.jpg"))
            assert saved == 3 and len(files) == 3
            assert len({f.read_bytes() for f in files}) == 3
    finally:
        reader.stop()
        server.shutdown()


def test_reader_survives_unreachable_camera():
    reader = LatestFrameReader("http://127.0.0.1:9/stream", read_timeout_s=0.2, reconnect_delay_s=0.1)
    reader.start()
    time.sleep(0.4)
    reader.stop()
    assert reader.latest() is None
    assert reader.last_error is not None
