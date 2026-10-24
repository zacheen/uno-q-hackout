import math

from control import (BALL, GOAL, BallGoalController, ControlConfig, Detection, Halt, HeadingMemory, Move,
                     Observation, Phase, Wait)


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


def _controller(**config) -> BallGoalController:
    return BallGoalController(ControlConfig(**config))


# The centring tests count milliseconds at the turn speeds they were written for, where one unit
# of horizontal error is a 600 ms turn.
_SLOW_TURNS = dict(turn_speed=120, min_turn_speed=60)


def _confirmed(c: BallGoalController, obs: Observation):
    """Feed obs until the search accepts it; every sighting short of confirm_hits returns Wait."""
    for _ in range(c.cfg.confirm_hits - 1):
        assert isinstance(c.step(obs), Wait)
    return c.step(obs)


def test_config_rejects_values_that_recurse_or_divide_by_zero():
    for bad in (dict(push_band=0.05), dict(push_align_band=0.05), dict(drift_margin=-0.1),
                dict(max_push_pulses=0), dict(max_orbit_pulses=0), dict(turn_rad_per_speed_ms=0.0),
                dict(turn_speed=0), dict(max_centre_turn_rad=0.0), dict(centre_turn_rad_per_unit=0.0),
                dict(min_pulse_ms=0), dict(max_approach_ms=0),
                dict(strafe_ms=0), dict(push_ms=0), dict(look_step_frac=0.0), dict(confirm_hits=4),
                dict(turn_rad_per_speed_ms=1e-7), dict(hfov_deg=0.0), dict(hfov_deg=180.0),
                dict(edge_band=0.5), dict(min_turn_speed=0), dict(min_turn_speed=181),
                dict(pulse_dead_ms=-1), dict(pulse_dead_ms=301), dict(right_turn_scale=0.49), dict(sidestep_pulses=-1),
                dict(right_turn_scale=2.01)):
        assert _raises(ValueError, ControlConfig, **bad), f"accepted {bad}"


def test_config_rejects_a_look_step_the_firmware_would_cut_short_once_the_pulse_is_corrected():
    # Each look step alone fits under 5000 ms, and only the corrected pulse does not.
    assert ControlConfig(turn_rad_per_speed_ms=1.13e-6).look_step_ms < 5000
    assert _raises(ValueError, ControlConfig, turn_rad_per_speed_ms=1.13e-6, right_turn_scale=2.0)
    assert ControlConfig(turn_rad_per_speed_ms=6.87e-7).look_step_ms < 5000
    assert _raises(ValueError, ControlConfig, turn_rad_per_speed_ms=6.87e-7, pulse_dead_ms=100)


def test_pulse_ms_lengthens_right_turns_only_and_adds_the_dead_time_to_every_direction():
    cfg = ControlConfig(pulse_dead_ms=40, right_turn_scale=1.25)
    assert cfg.pulse_ms("rotate_right", 200) == 290
    for direction in ("rotate_left", "forward", "backward", "left", "right"):
        assert cfg.pulse_ms(direction, 200) == 240
    assert ControlConfig().pulse_ms("rotate_right", 200) == 200


def test_look_step_follows_the_camera_field_of_view():
    cfg = ControlConfig(hfov_deg=70.0, look_step_frac=0.7)
    assert math.isclose(math.degrees(cfg.look_step_rad), 49.0)
    turned = cfg.turn_speed * cfg.look_step_ms * cfg.turn_rad_per_speed_ms
    assert math.isclose(turned, cfg.look_step_rad, rel_tol=0.01)
    assert cfg.pulses_per_lap == 9


def test_halt_reason_reaches_the_controller_for_the_run_log():
    c = _controller(search_when_lost=False)
    _confirmed(c, _obs(ball=(0.5, 0.3)))
    for _ in range(ControlConfig().ball_lost_grace + 1):
        action = c.step(_obs())
    assert isinstance(action, Halt) and c.reason == action.reason


def test_move_rejects_continuous_and_unknown_commands():
    for bad in (dict(direction="forward", speed=100, ms=0), dict(direction="jump", speed=100, ms=100)):
        assert _raises(ValueError, Move, **bad), f"accepted {bad}"


def test_with_confirmation_a_single_sighting_does_not_end_the_search():
    c = _controller(confirm_hits=2)
    assert isinstance(c.step(_obs(ball=(0.5, 0.3))), Wait) and c.phase is Phase.SEARCH
    action = c.step(_obs())
    assert c.phase is Phase.SEARCH and action.direction.startswith("rotate")


def test_with_confirmation_a_second_sighting_ends_the_search():
    c = _controller(confirm_hits=2)
    action = _confirmed(c, _obs(ball=(0.5, 0.3)))
    assert c.phase is Phase.APPROACH and action.direction == "forward"


def test_turns_toward_an_off_centre_ball():
    c = _controller()
    assert _confirmed(c, _obs(ball=(0.2, 0.4))).direction == "rotate_left"
    assert c.step(_obs(ball=(0.8, 0.4))).direction == "rotate_right"


def test_a_short_centring_turn_keeps_full_speed_until_a_turn_overshoots():
    action = _confirmed(_controller(), _obs(ball=(0.6, 0.3)))
    assert action == Move("rotate_right", ControlConfig().turn_speed, ControlConfig().min_pulse_ms)


def test_a_centring_turn_that_overshoots_makes_the_next_ones_smaller():
    c = _controller(**_SLOW_TURNS)
    assert _confirmed(c, _obs(ball=(0.75, 0.3))).ms == 150
    # The ball lands as far past the centre as it started, so the turn rate is faster than assumed.
    assert c.step(_obs(ball=(0.25, 0.3))) == Move("rotate_left", 120, 90)
    # It overshoots again, and a correction shorter than the minimum pulse turns slower instead.
    assert c.step(_obs(ball=(0.6, 0.3))) == Move("rotate_right", 60, ControlConfig().min_pulse_ms)


def test_centring_turns_grow_back_after_falling_short_but_never_past_the_calibrated_length():
    c = _controller(**_SLOW_TURNS)
    _confirmed(c, _obs(ball=(0.75, 0.3)))
    shrunk = c.step(_obs(ball=(0.25, 0.3))).ms
    lengths = [c.step(_obs(ball=(0.25, 0.3))).ms for _ in range(8)]
    assert shrunk < lengths[0] and lengths[-1] == 150 and max(lengths) == 150


def test_a_turn_that_lands_across_the_centre_but_inside_the_band_keeps_the_gain():
    c = _controller(**_SLOW_TURNS)
    _confirmed(c, _obs(ball=(0.62, 0.3)))
    assert c.step(_obs(ball=(0.47, 0.3))).direction == "forward"
    assert c.step(_obs(ball=(0.75, 0.3))).ms == 150


def test_only_the_frame_right_after_a_centring_turn_is_compared():
    c = _controller(**_SLOW_TURNS)
    _confirmed(c, _obs(ball=(0.75, 0.3)))
    assert isinstance(c.step(_obs()), Wait)
    # A sample kept past the missed frame would read this as an overshoot and shrink the turn to 90 ms.
    assert c.step(_obs(ball=(0.25, 0.3))).ms == 150


def test_the_centring_gain_stops_shrinking_at_its_floor():
    c = _controller(**_SLOW_TURNS)
    _confirmed(c, _obs(ball=(0.9, 0.3)))
    for x in (0.1, 0.9, 0.1):
        c.step(_obs(ball=(x, 0.3)))
    # Four overshoots would leave a gain of 0.13, but the 0.2 floor gives 48 ms, stretched to 80 at speed 72.
    assert c.step(_obs(ball=(0.9, 0.3))) == Move("rotate_right", 72, ControlConfig().min_pulse_ms)


def test_drives_at_a_centred_far_ball():
    c = _controller()
    action = _confirmed(c, _obs(ball=(0.5, 0.3)))
    assert action.direction == "forward" and c.phase is Phase.APPROACH


def test_slides_left_when_goal_is_right_of_a_near_ball():
    c = _controller()
    action = _confirmed(c, _obs(ball=(0.5, 0.7), goal=(0.8, 0.3)))
    assert c.phase is Phase.LINEUP and action.direction == "left"


def test_pushes_when_ball_and_goal_line_up():
    c = _controller()
    action = _confirmed(c, _obs(ball=(0.5, 0.7), goal=(0.52, 0.3)))
    assert c.phase is Phase.PUSH and action.direction == "forward"


def test_scans_for_the_goal_when_its_direction_is_unknown():
    c = _controller()
    action = _confirmed(c, _obs(ball=(0.5, 0.7)))
    assert c.phase is Phase.SCAN and action.direction == "rotate_left"
    assert action.ms == ControlConfig().look_step_ms


def test_without_the_goal_a_near_ball_is_pushed_straight_whatever_the_detector_says():
    for goal in (None, (0.8, 0.3)):
        c = _controller(use_goal=False, sidestep_pulses=3)
        action = _confirmed(c, _obs(ball=(0.5, 0.7), goal=goal))
        assert c.phase is Phase.PUSH and action.direction == "forward"


def test_without_the_scan_a_near_ball_is_pushed_straight_unless_a_goal_shows():
    c = _controller(scan_for_goal=False, sidestep_pulses=3)
    action = _confirmed(c, _obs(ball=(0.5, 0.7)))
    assert c.phase is Phase.PUSH and action.direction == "forward"
    c = _controller(scan_for_goal=False)
    _confirmed(c, _obs(ball=(0.5, 0.7), goal=(0.8, 0.3)))
    assert c.phase is Phase.LINEUP and c.reason == "orbit to line up"


def test_a_lone_post_shifts_the_remembered_pair_instead_of_standing_for_the_goal():
    m = HeadingMemory(ControlConfig())
    m.see_posts([0.3, 0.7])
    assert abs(m.goal_column() - 0.5) < 1e-9
    # Only the right post shows, 0.1 left of where the pair put it.
    m.see_posts([0.6])
    assert abs(m.goal_relative_to(0.5) - (m.bearing(0.6) - m.bearing(0.7))) < 1e-9
    alone = HeadingMemory(ControlConfig())
    alone.see_posts([0.6])
    assert abs(alone.goal_column() - 0.6) < 1e-9


def test_a_push_keeps_going_when_the_ball_hides_one_post_of_a_lined_up_goal():
    c = _controller()
    posts = (Detection(GOAL, 0.2, 0.3, 0.9), Detection(GOAL, 0.8, 0.3, 0.9))
    ball = Detection(BALL, 0.5, 0.7, 0.9)
    _confirmed(c, Observation(ball=ball, goal=None, frame_age_s=0.05, seq=1, posts=posts))
    assert c.phase is Phase.PUSH
    action = c.step(Observation(ball=ball, goal=None, frame_age_s=0.05, seq=2, posts=posts[1:]))
    assert c.phase is Phase.PUSH and action.direction == "forward"


def test_a_lone_post_sends_lineup_aside_to_find_the_other_one_first():
    c = _controller(sidestep_pulses=1)
    lone = Observation(ball=Detection(BALL, 0.5, 0.7, 0.9), goal=None, frame_age_s=0.05, seq=1,
                       posts=(Detection(GOAL, 0.5, 0.3, 0.9),))
    assert _confirmed(c, lone) == Move("left", c.cfg.strafe_speed, c.cfg.strafe_ms)
    assert c.reason == "step aside to find the other post"
    # With the sidestep spent the lone post stands for the goal, which lies behind the ball.
    c.step(lone)
    assert c.phase is Phase.PUSH


def test_the_scan_turns_on_after_a_lone_post_and_ends_at_a_pair():
    c = _controller()
    _confirmed(c, _obs(ball=(0.5, 0.7)))
    assert c.phase is Phase.SCAN

    def frame(*xs):
        return Observation(ball=None, goal=None, frame_age_s=0.05, seq=2,
                           posts=tuple(Detection(GOAL, x, 0.3, 0.9) for x in xs))

    action = c.step(frame(0.1))
    assert c.reason == "turn on to find the other post" and action.direction == "rotate_left"
    c.step(frame(0.3, 0.6))
    assert c.reason == "turn back to the remembered ball"


def test_the_scan_gives_up_after_its_laps():
    c = _controller(scan_laps=1)
    _confirmed(c, _obs(ball=(0.5, 0.7)))
    for _ in range(c.cfg.pulses_per_lap - 1):
        c.step(_obs())
        assert c.reason == "scan in place for the goal"
    c.step(_obs())
    assert c.reason != "scan in place for the goal"


def test_steps_aside_before_scanning_and_lines_up_once_the_goal_shows():
    c = _controller(sidestep_pulses=2)
    assert _confirmed(c, _obs(ball=(0.5, 0.7))) == Move("left", c.cfg.strafe_speed, c.cfg.strafe_ms)
    assert c.reason == "step aside to look past the ball"
    c.step(_obs(ball=(0.5, 0.7), goal=(0.8, 0.3)))
    assert c.phase is Phase.LINEUP and c.reason == "orbit to line up"


def test_scans_once_the_sidesteps_run_out():
    c = _controller(sidestep_pulses=2)
    assert _confirmed(c, _obs(ball=(0.5, 0.7))).direction == "left"
    assert c.step(_obs(ball=(0.5, 0.7))).direction == "left"
    assert c.step(_obs(ball=(0.5, 0.7))).direction == "rotate_left" and c.phase is Phase.SCAN


def test_each_lineup_gets_its_own_sidesteps():
    c = _controller(sidestep_pulses=1)
    assert _confirmed(c, _obs(ball=(0.5, 0.7))).direction == "left"
    assert c.step(_obs(ball=(0.5, 0.4))).direction == "forward" and c.phase is Phase.APPROACH
    assert c.step(_obs(ball=(0.5, 0.7))).direction == "left"


def test_scan_return_keeps_turning_one_way_when_the_ball_is_not_where_memory_says():
    c = _controller()
    _confirmed(c, _obs(ball=(0.5, 0.7)))
    for _ in range(2):
        c.step(_obs())
    first = c.step(_obs(goal=(0.5, 0.3)))
    assert c.phase is Phase.SCAN and first.direction == "rotate_right"
    # The remembered ball passes the view and ends up on the left.
    turns = [c.step(_obs()).direction for _ in range(8)]
    assert c.phase is Phase.SCAN and turns == ["rotate_right"] * 8


def test_a_second_scan_chooses_its_return_turn_afresh():
    c = _controller(goal_memory_pulses=2)
    _confirmed(c, _obs(ball=(0.5, 0.7)))
    for _ in range(2):
        c.step(_obs())
    assert c.step(_obs(goal=(0.5, 0.3))).direction == "rotate_right"
    # Ball back in view, then the goal memory runs out and lineup starts a second scan.
    for _ in range(3):
        c.step(_obs(ball=(0.5, 0.7)))
    assert c.phase is Phase.SCAN
    for _ in range(5):
        c.step(_obs())
    # Six look steps left since the ball was last seen, one of them when this scan began, put
    # the remembered ball about 150 degrees to the left, so left is the short way.
    assert c.step(_obs(goal=(0.5, 0.3))).direction == "rotate_left"


def test_waits_briefly_then_searches_when_a_centred_ball_disappears():
    c = _controller()
    _confirmed(c, _obs(ball=(0.5, 0.3)))
    for _ in range(ControlConfig().ball_lost_grace):
        assert isinstance(c.step(_obs()), Wait)
    action = c.step(_obs())
    assert c.phase is Phase.SEARCH and action.direction.startswith("rotate")
    assert action.ms == ControlConfig().look_step_ms


def test_a_ball_that_rolls_over_an_edge_is_chased_at_once():
    c = _controller()
    _confirmed(c, _obs(ball=(0.95, 0.4)))
    action = c.step(_obs())
    assert c.phase is Phase.SEARCH and action.direction == "rotate_right"


def _lose_a_right_ball_with_the_goal_off_to_the_left(c: BallGoalController) -> None:
    _confirmed(c, _obs(ball=(0.75, 0.3), goal=(0.05, 0.3)))
    # Centring the ball on the right turns the robot right, pushing the goal out past the left edge.
    for _ in range(4):
        c.step(_obs(ball=(0.75, 0.3)))
    for _ in range(ControlConfig().ball_lost_grace):
        c.step(_obs())


def test_search_turns_toward_a_remembered_goal_that_is_out_of_view():
    c = _controller()
    _lose_a_right_ball_with_the_goal_off_to_the_left(c)
    action = c.step(_obs())
    assert c.phase is Phase.SEARCH and action.direction == "rotate_left"


def test_search_keeps_its_first_turn_direction():
    c = _controller()
    _lose_a_right_ball_with_the_goal_off_to_the_left(c)
    # Turning left brings the remembered goal into view after a couple of steps; a turn
    # chosen afresh each step would then switch to the ball's last side, which is right.
    turns = {c.step(_obs()).direction for _ in range(5)}
    assert c.phase is Phase.SEARCH and turns == {"rotate_left"}


def test_a_new_search_ignores_sightings_from_before_the_ball_was_lost():
    c = _controller(confirm_hits=2)
    _confirmed(c, _obs(ball=(0.95, 0.4)))
    assert c.step(_obs()).direction == "rotate_right"
    # Without clearing, the two sightings before the loss would confirm this one at once.
    assert isinstance(c.step(_obs(ball=(0.9, 0.4))), Wait)


def test_an_edge_exit_outranks_the_remembered_goal():
    c = _controller()
    _confirmed(c, _obs(ball=(0.95, 0.3), goal=(0.05, 0.3)))
    assert c.step(_obs()).direction == "rotate_right"


def test_a_search_cut_short_by_the_boundary_starts_fresh_next_time():
    c = _controller()
    assert c.step(_obs()).direction == "rotate_right"  # opening search, last ball side defaults right
    c.notify_boundary()
    c.step(_obs())
    c.step(_obs())
    c.step(_obs(ball=(0.1, 0.3)))
    assert c.phase is Phase.APPROACH
    action = c.step(_obs())
    assert c.phase is Phase.SEARCH and action.direction == "rotate_left"


def test_keeps_dribbling_while_goal_is_ahead_and_ball_is_under_the_camera():
    c = _controller()
    _confirmed(c, _obs(ball=(0.5, 0.7), goal=(0.5, 0.3)))
    for _ in range(5):
        action = c.step(_obs(goal=(0.5, 0.35)))
        assert c.phase is Phase.PUSH and action.direction == "forward"


def test_backs_off_after_losing_the_ball_mid_push():
    c = _controller()
    _confirmed(c, _obs(ball=(0.5, 0.7), goal=(0.5, 0.3)))
    actions = [c.step(_obs()) for _ in range(ControlConfig().blind_push_pulses + 1)]
    assert c.phase is Phase.RECOVER and actions[-1].direction == "backward"


def test_halts_on_a_lost_ball_when_searching_is_disabled():
    c = _controller(search_when_lost=False)
    _confirmed(c, _obs(ball=(0.5, 0.3)))
    actions = [c.step(_obs()) for _ in range(ControlConfig().ball_lost_grace + 1)]
    assert isinstance(actions[-1], Halt)


def test_gives_up_after_searching_every_lap():
    c = _controller()
    cfg = ControlConfig()
    actions = [c.step(_obs()) for _ in range(cfg.max_search_laps * cfg.pulses_per_lap + 1)]
    assert isinstance(actions[-1], Halt)
