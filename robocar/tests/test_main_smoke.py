import importlib

from arduino import app_utils


def test_main_imports_and_every_mode_builds():
    app_utils.calls.clear()
    main = importlib.import_module("main")
    # Import ran the starter lifecycle: stop first, then a status read with the program disabled.
    names = [name for name, _ in app_utils.calls]
    assert names[0] == "stop" and "read_sensors" in names
    try:
        for mode in main.MODES:
            main.MODE = mode
            routine, perception = main.build_routine()
            assert callable(routine)
            assert (perception is None) == (mode in ("motion_check", "turn_check"))
            if perception is not None:
                perception.close()
    finally:
        main.reader.stop()


def test_turn_check_sends_look_steps_both_ways_the_slowest_centring_turns_then_the_dead_time_ladder():
    import robot_client
    from calibration import DEAD_LADDER_MS, SLOW_TURN_PULSES, TURN_CHECK_PULSES
    from control import ControlConfig

    main = importlib.import_module("main")
    cfg = ControlConfig()
    sleep = robot_client.time.sleep
    app_utils.Bridge.sensors["program_enabled"] = True
    app_utils.calls.clear()
    robot_client.time.sleep = lambda s: None
    try:
        main.turn_check()
    finally:
        robot_client.time.sleep = sleep
        app_utils.Bridge.sensors["program_enabled"] = False
    drives = [args for name, args in app_utils.calls if name == "drive"]
    assert drives == ([("rotate_left", cfg.turn_speed, cfg.pulse_ms("rotate_left", cfg.look_step_ms))] * TURN_CHECK_PULSES
                      + [("rotate_right", cfg.turn_speed, cfg.pulse_ms("rotate_right", cfg.look_step_ms))] * TURN_CHECK_PULSES
                      + [("rotate_right", cfg.min_turn_speed, cfg.pulse_ms("rotate_right", cfg.min_pulse_ms))] * SLOW_TURN_PULSES
                      + [("rotate_left", cfg.turn_speed, ms) for ms in DEAD_LADDER_MS])
