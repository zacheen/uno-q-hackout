"""Runs every test_* function in this folder without pytest.

    conda run -n ML python robocar/tests/run_tests.py
"""
import importlib
import sys
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python"))
sys.path.insert(0, str(HERE.parent / "tools"))
sys.path.insert(0, str(HERE))
# Stand-in for the board-only arduino.app_utils, so main.py and hardware.py import on a PC.
sys.path.insert(0, str(HERE / "stubs"))


def main() -> int:
    failures = 0
    total = 0
    for path in sorted(HERE.glob("test_*.py")):
        try:
            module = importlib.import_module(path.stem)
        except Exception:
            total += 1
            failures += 1
            print(f"FAIL  {path.stem} (import)")
            traceback.print_exc()
            continue
        for name in sorted(n for n in dir(module) if n.startswith("test_")):
            total += 1
            try:
                getattr(module, name)()
                print(f"ok    {path.stem}.{name}")
            except Exception:
                failures += 1
                print(f"FAIL  {path.stem}.{name}")
                traceback.print_exc()
    print(f"{total - failures}/{total} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
