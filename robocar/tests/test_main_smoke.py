import importlib

from arduino import app_utils


def test_main_imports_and_every_mode_builds():
    app_utils.calls.clear()
    main = importlib.import_module("main")
    # Import ran the starter lifecycle: stop first, then a status read with the program disabled.
    names = [name for name, _ in app_utils.calls]
    assert names[0] == "stop" and "read_sensors" in names
    try:
        for mode in ("motion_check", "detect_only", "play"):
            main.MODE = mode
            routine, perception = main.build_routine()
            assert callable(routine)
            assert (perception is None) == (mode == "motion_check")
            if perception is not None:
                perception.close()
    finally:
        main.reader.stop()
