"""Stand-in for the board-only arduino.app_utils module so main.py can be imported on a PC."""
import json

calls = []


class Bridge:
    # Subset of sketch.ino's read_sensors JSON, with line_digital as a 4-element array like the real one.
    sensors = {"program_enabled": False, "line_ok": True, "line_digital": [0, 0, 0, 0], "ultrasonic_mm": -1}

    @staticmethod
    def call(name, *args):
        calls.append((name, args))
        if name == "read_sensors":
            return json.dumps(Bridge.sensors)
        if name == "health":
            return json.dumps({"firmware": "stub"})
        return True


class App:
    @staticmethod
    def run(user_loop):
        user_loop()
