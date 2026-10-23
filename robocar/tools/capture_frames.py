"""Save frames from the robot camera for Edge Impulse training.

Training images should come through the same camera and JPEG path the robot uses
while driving, since phone photos differ in colour and exposure. Join the camera's
Wi-Fi on the laptop, close any browser preview (the camera serves one stream), and
stop the robot app if it is reading the camera.

    conda run -n ML python robocar/tools/capture_frames.py --count 150 --every-s 0.5
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from camera import LatestFrameReader  # noqa: E402

DEFAULT_URL = "http://192.168.5.1:81/stream"
# robocar/captures/ is already in the repository's .gitignore.
DEFAULT_OUT = Path(__file__).resolve().parent.parent / "captures"
FROZEN_WARNING_FRAMES = 10


def capture(reader: LatestFrameReader, out_dir: Path, count: int, every_s: float,
            frame_timeout_s: float = 5.0) -> int:
    """Write up to count distinct, decodable frames at least every_s apart; returns how many were saved."""
    out_dir.mkdir(parents=True, exist_ok=True)
    saved, last_seq, last_jpeg, repeats = 0, 0, b"", 0
    while saved < count:
        frame = reader.wait_for_frame_after(last_seq, frame_timeout_s)
        if frame is None:
            print(f"\nno frame for {frame_timeout_s}s, last error: {reader.last_error}")
            break
        last_seq = frame.seq
        # Identical bytes mean a frozen camera and an undecodable JPEG is useless for training.
        if frame.jpeg == last_jpeg or frame.decode() is None:
            repeats += 1
            if repeats == FROZEN_WARNING_FRAMES:
                print(f"\nskipped {repeats} repeated or corrupt frames, the camera may be frozen")
            continue
        repeats, last_jpeg = 0, frame.jpeg
        (out_dir / f"frame-{saved:04d}.jpg").write_bytes(frame.jpeg)
        saved += 1
        print(f"\rsaved {saved}/{count}", end="", flush=True)
        time.sleep(every_s)
    print()
    return saved


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT / time.strftime("%Y%m%d-%H%M%S"))
    parser.add_argument("--count", type=int, default=150)
    parser.add_argument("--every-s", type=float, default=0.5, help="move the ball or robot between shots")
    args = parser.parse_args()
    reader = LatestFrameReader(args.url)
    reader.start()
    try:
        saved = capture(reader, args.out, args.count, args.every_s)
    finally:
        reader.stop()
    print(f"{saved} frames in {args.out}")


if __name__ == "__main__":
    main()
