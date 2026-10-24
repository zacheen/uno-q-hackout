"""Settings measured on the event robot. The robot runs with FIELD and tools/turn_calc.py reads it.

ControlConfig keeps its defaults for the simulator and the tests, whose numbers are quoted in the docs.
"""
from control import ControlConfig

# turn_check on 2026-10-24: ten left look steps turned about 240 degrees, the right ones came back
# to the start, and the eight ladder pulses together turned about 45 degrees, which fits a dead
# time of 45 to 50 ms. The ladder value one step longer, 60, is used.
FIELD = ControlConfig(
    turn_rad_per_speed_ms=7.83e-6,
    pulse_dead_ms=60,
    right_turn_scale=1.0,
    center_band=0.08,
)
