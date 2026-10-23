from control import BALL, GOAL, BallGoalController, ControlConfig, Detection, Halt, Move, Observation, Phase, Wait


def _obs(ball=None, goal=None) -> Observation:
    return Observation(
        ball=Detection(BALL, *ball, 0.9) if ball else None,
        goal=Detection(GOAL, *goal, 0.9) if goal else None,
        frame_age_s=0.05, seq=1,
    )


def _raises(exc_type, fn, *args, **kwargs) -> bool:
    try:
        fn(*args, **kwargs)
    except exc_type:
        return True
    return False


def test_config_rejects_values_that_recurse_or_divide_by_zero():
    for bad in (dict(push_band=0.05), dict(push_align_band=0.05), dict(drift_margin=-0.1),
                dict(max_push_pulses=0), dict(max_orbit_pulses=0), dict(turn_rad_per_speed_ms=0.0),
                dict(turn_speed=0), dict(max_turn_ms=0), dict(min_pulse_ms=0), dict(max_approach_ms=0),
                dict(strafe_ms=0), dict(push_ms=0)):
        assert _raises(ValueError, ControlConfig, **bad), f"accepted {bad}"


def test_halt_reason_reaches_the_controller_for_the_run_log():
    c = BallGoalController(ControlConfig(search_when_lost=False))
    c.step(_obs(ball=(0.5, 0.3)))
    for _ in range(ControlConfig().ball_lost_grace + 1):
        action = c.step(_obs())
    assert isinstance(action, Halt) and c.reason == action.reason


def _controller() -> BallGoalController:
    return BallGoalController(ControlConfig())


def test_move_rejects_continuous_and_unknown_commands():
    for bad in (dict(direction="forward", speed=100, ms=0), dict(direction="jump", speed=100, ms=100)):
        assert _raises(ValueError, Move, **bad), f"accepted {bad}"


def test_turns_toward_an_off_centre_ball():
    c = _controller()
    assert c.step(_obs(ball=(0.2, 0.4))).direction == "rotate_left"
    assert c.step(_obs(ball=(0.8, 0.4))).direction == "rotate_right"


def test_drives_at_a_centred_far_ball():
    c = _controller()
    action = c.step(_obs(ball=(0.5, 0.3)))
    assert action.direction == "forward" and c.phase is Phase.APPROACH


def test_slides_left_when_goal_is_right_of_a_near_ball():
    c = _controller()
    action = c.step(_obs(ball=(0.5, 0.7), goal=(0.8, 0.3)))
    assert c.phase is Phase.LINEUP and action.direction == "left"


def test_pushes_when_ball_and_goal_line_up():
    c = _controller()
    action = c.step(_obs(ball=(0.5, 0.7), goal=(0.52, 0.3)))
    assert c.phase is Phase.PUSH and action.direction == "forward"


def test_scans_for_the_goal_when_its_direction_is_unknown():
    c = _controller()
    action = c.step(_obs(ball=(0.5, 0.7)))
    assert c.phase is Phase.SCAN and action.direction == "rotate_left"


def test_waits_briefly_then_searches_when_the_ball_disappears():
    c = _controller()
    c.step(_obs(ball=(0.5, 0.3)))
    grace = ControlConfig().ball_lost_grace
    for _ in range(grace):
        assert isinstance(c.step(_obs()), Wait)
    action = c.step(_obs())
    assert c.phase is Phase.SEARCH and action.direction.startswith("rotate")


def test_keeps_dribbling_while_goal_is_ahead_and_ball_is_under_the_camera():
    c = _controller()
    c.step(_obs(ball=(0.5, 0.7), goal=(0.5, 0.3)))
    for _ in range(5):
        action = c.step(_obs(goal=(0.5, 0.35)))
        assert c.phase is Phase.PUSH and action.direction == "forward"


def test_backs_off_after_losing_the_ball_mid_push():
    c = _controller()
    c.step(_obs(ball=(0.5, 0.7), goal=(0.5, 0.3)))
    actions = [c.step(_obs()) for _ in range(ControlConfig().blind_push_pulses + 1)]
    assert c.phase is Phase.RECOVER and actions[-1].direction == "backward"


def test_halts_on_a_lost_ball_when_searching_is_disabled():
    c = BallGoalController(ControlConfig(search_when_lost=False))
    c.step(_obs(ball=(0.5, 0.3)))
    actions = [c.step(_obs()) for _ in range(ControlConfig().ball_lost_grace + 1)]
    assert isinstance(actions[-1], Halt)


def test_gives_up_after_searching_every_lap():
    c = _controller()
    cfg = ControlConfig()
    actions = [c.step(_obs()) for _ in range(cfg.max_search_laps * cfg.pulses_per_lap + 1)]
    assert isinstance(actions[-1], Halt)
