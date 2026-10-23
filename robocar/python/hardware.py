import time

from control import Move
from robot_client import MiniAutoRobot, ProgramStopped


class RobotExecutor:
    """Sends moves to the real robot; drive() waits out the pulse and checks the BOOT stop."""

    def __init__(self, robot: MiniAutoRobot) -> None:
        self._robot = robot

    def execute(self, move: Move) -> None:
        self._robot.drive(move.direction, speed=move.speed, ms=move.ms)

    def wait(self, ms: int) -> None:
        time.sleep(ms / 1000.0)
        # drive() is where the BOOT stop is normally noticed, and waiting skips it.
        if not self._robot.is_running():
            raise ProgramStopped


class LineSensorGuard:
    """Reports the field edge when any line-sensor channel reads black tape.

    Which bit value means black is unverified on this robot, so check black_bit on
    the real field before enabling the guard.
    """

    def __init__(self, robot: MiniAutoRobot, black_bit: int = 1) -> None:
        self._robot = robot
        self._black_bit = black_bit

    def triggered(self) -> bool:
        sensors = self._robot.read_sensors()
        # line_digital is zeroed on a failed read, which would look like all-white or all-black.
        if not sensors.get("line_ok"):
            return False
        # The sketch sends one 0/1 value per channel as a JSON array, not a bitmask.
        return any(int(bit) == self._black_bit for bit in sensors.get("line_digital", []))
