# Robocar solution

This robot finds a ball with its camera, lines the ball up with the goal, pushes it across the goal line, and goes back for the ball when a shot misses. Everything runs on the Arduino UNO Q. The Linux side decodes camera frames, runs the detector and makes every decision, and the MCU side drives the motors with timed stops.

Every entry marked "to be measured", "to be filled" or "to be confirmed" is recorded on the event day.

## How it works

Each decision follows the same cycle.

1. A background thread reads the camera's MJPEG stream and keeps only the newest frame with the time it arrived.
2. After every move the robot waits for a frame that arrived after the move finished, so no decision is based on a frame captured while the robot was moving.
3. An Edge Impulse FOMO model finds the `soccerball` and the `goal` and reports their centres as fractions of the image. The event goal is two posts, each detected as `goal`, so the goal's centre is taken halfway between the outermost two. The robot remembers the direction of both posts, so when the ball or the edge of the view hides one of them, the post still in view moves the remembered pair and the centre stays between the posts instead of jumping onto the visible one. A single post stands for the goal only while the other has never been seen. When more than one ball is reported, the robot follows the one lowest in the image, which is the nearest, because following the highest score let a far detection win every other frame and the robot kept turning between them instead of driving. A ball reported in the top third of the image is ignored, because a ball on the floor always sits below the far edge of the floor, which the robot camera shows at 0.42 of the image height, while balls were reported above it, where only furniture shows, with scores up to 1.0. When no model loads, a colour threshold finds the ball.
4. The controller picks one short move, and the robot performs it as a timed pulse that the MCU stops on its own.

The controller is a state machine.

| Phase | What the robot does | Leaves when |
| --- | --- | --- |
| search | Turns half a field of view between looks, toward the edge the ball rolled over, else toward the remembered goal, else toward the side the ball was last seen, and backs off after each lap to uncover a ball sitting below the camera | The ball is seen, or three laps pass |
| approach | Turns to centre the ball, then drives toward it in pulses that shrink as it gets closer | The ball is close enough to line up |
| scan | Spins in place to find the goal while remembering where the ball was, turns one more look step when only one post has shown so the other comes into the same frame, then turns back to the ball in one direction chosen once, so a miscalibrated turn rate cannot leave it rocking over the same arc | The goal has been seen and the ball is back in view, or `scan_laps` laps pass and the ball is pushed straight |
| lineup | Slides sideways while turning to keep the ball centred, which orbits the ball until the goal sits directly behind it | The ball and goal line up, or the orbit budget runs out |
| push | Drives forward in short pulses, and keeps going while the goal stays ahead even after the ball drops below the camera | The ball and goal drift apart, the ball is lost, or the push budget runs out |
| recover | Backs off to widen the view | It goes back to approach if the ball is visible, and to search otherwise |

Lining up works because a sideways slide moves the nearby ball across the image more than the distant goal, which changes the gap between them, while a turn moves both by the same amount and leaves the gap alone. The two corrections therefore barely interfere, and neither needs a distance estimate.

The robot also keeps a rough heading by adding up the turns it has commanded. That lets it remember which way the goal lay after the goal leaves the image, so it orbits the short way round. Only a sighting from the last few moves can start a push, because the remembered heading drifts with every turn.

Near the ball, the ball itself hides a goal that lies right behind it, which is exactly where the goal is once the two line up. A goal seen within the last few moves and within 12 degrees of the ball therefore counts as lined up. When lineup has neither a goal in view nor one in memory, it can first strafe about one ball width to look past the ball, because spinning in place never sees behind it. That step is off by default and meant for starts that put robot, ball and goal in one line, where it was 2 to 12 seconds faster in simulation. When only one post is known, the same strafe looks for the other before lining up, because lining up with a lone post aims the ball at the post. With `scan_for_goal` off, lineup skips both the strafe and the scan and pushes a near ball straight whenever no goal is in view or memory, and still lines up with a goal it does see. The event robot scans for one lap, which with the turn rate set below the measured one turns more than a full circle.

Centring turns are sized as angles and turned into pulse lengths through the calibrated turn rate, the same way the search's look steps are. When one overshoots so far that the ball lands outside the centre band on the other side, the next ones shrink, and a turn that would fall below the minimum pulse length turns slower instead. A turn that falls short grows them back, never past the calibrated size, so a shrink caused by detector jitter wears off. In simulation a turn rate twice the calibrated one scored 142 of 200 with this and 7 without it, while the calibrated rate scored 199 and 196.

The controller sizes every move as the time the motors should run, and the run loop sends each pulse corrected for this robot's motors. Every pulse gets the start-up time measured on the field added, because cheap gear motors lose the start of each pulse to static friction and spin-up, and right turns are lengthened when the motors turn right slower than left. In simulation a 60 ms start-up loss cost 89 of 200 goals and 100 ms all of them, until the correction won them back. Emulating the whole field calibration in simulation, the robot scored 186 to 200 of 200 under every motor error tried, including those losses, a 30 percent difference between left and right turns, turns half or twice the assumed speed, and drift while driving and strafing.

## Safety

- Every move is a short timed pulse, and the MCU stops the motors when the pulse ends even if Python hangs.
- A run ends when no new frame arrives in time, when the newest frame is too old, when the camera keeps resending an identical frame, when inference takes too long, or when the run time limit passes.
- A restart always needs a press of the camera BOOT button.
- When the line sensor sees the taped boundary, the robot backs off before anything else happens. This guard stays off until the tape polarity has been checked on the field and `LINE_GUARD_ENABLED` in `python/main.py` is switched on. If the sensor still reads tape after the allowed number of back-offs in a row, the run ends, because a wrong polarity setting or a robot that crossed the edge sideways would otherwise keep it reversing.

## Hardware

- An Arduino UNO Q on a Hiwonder miniAuto mecanum chassis, with the organisers' sketch driving the four motors.
- The ESP32-S3 camera supplied with the robot. It serves 320×240 MJPEG frames over its own Wi-Fi access point and reports its BOOT button to the MCU over I2C.
- The four-channel line sensor on the chassis, used only for the taped boundary.
- The ball and goal from the challenge. Their colour, size and placement are to be filled.

## Source layout

| Path | What it holds |
| --- | --- |
| `python/main.py` | Mode selection, the BOOT-started run lifecycle and the calibration routines |
| `python/camera.py` | The background stream reader that keeps only the newest frame and its arrival time |
| `python/perception.py` | The `.eim` detector, the colour fallback, and the frozen and corrupt frame checks |
| `python/control.py` | The state machine and every tunable threshold in `ControlConfig` |
| `python/field_config.py` | The settings measured on the event robot, which override the defaults |
| `python/calibration.py` | The turn calibration shared by `turn_check` on the robot and `tools/turn_calc.py` |
| `python/runtime.py` | The run loop shared by the robot and the simulator, with every way a run can end |
| `python/hardware.py` | The motor executor and the line sensor guard |
| `python/robot_client.py`, `sketch/` | The organisers' robot driver and MCU firmware, unchanged |
| `python/edge_impulse_linux/` | The Edge Impulse Linux Python SDK, unchanged, with its license |
| `sim/` | The top-down simulator and the robustness sweep |
| `tests/` | Unit tests that run on an ordinary computer |
| `tools/` | Frame capture for training images, the turn calibration calculator and plotting of run logs |

## Model and dataset

The detector is an Edge Impulse FOMO model with two classes, `soccerball` and `goal`, trained with the impulse settings in the Edge Impulse section of the organisers' robocar guide.

| Item | Value |
| --- | --- |
| Edge Impulse project | A clone of the organisers' public project 1144501 made after they relabelled the goal as two posts, with the resize mode set to Squash. An earlier clone, from before the relabelling, had its 125 goal samples enabled by hand |
| Model file in `python/` | `uno-q-hackout-new-linux-aarch64-v1.eim`, built for Arduino UNO Q with the EON Compiler and int8 quantisation, and kept out of git by `.gitignore` because of its 13.6 MB |
| Training images | to be filled for the relabelled data. The earlier clone had 255, taken with the robot camera, giving 211 objects, 125 goal and 86 soccer_ball |
| Test images | to be filled for the relabelled data. The earlier clone had 66 |
| Training cycles and learning rate | 150 cycles at 0.001, with data augmentation |
| Test accuracy | to be filled for the relabelled data. The earlier model scored F1 86.4% on the validation set for int8, with 96% of goals and 93.3% of balls found and no background taken for either |

The camera's exposure and white balance cannot be changed through its driver, so robustness to the venue lighting has to come from the training images. `tools/capture_frames.py` saves frames straight from the robot camera while a laptop is on the camera's Wi-Fi, which keeps training images on the same sensor and JPEG path the robot sees while driving.

## Running it

1. Connect the UNO Q by USB for deploying. The prepared board already joins the camera's Wi-Fi, which has no Internet access, so the app installs nothing at start. The Edge Impulse SDK that runs the `.eim` model is copied into `python/edge_impulse_linux/`, and the numpy, OpenCV and pyaudio it needs come with the App Lab container.
2. Put the exported `.eim` model in `python/`. Without one the app falls back to a colour threshold, which finds the ball but never the goal.
3. Deploy from a terminal in this folder.

   ```sh
   make flash
   make logs
   ```

   On Windows run these in Git Bash. The `GNUmakefile` here stops Git Bash from rewriting the `/tmp` and `TMPDIR` arguments that the Makefile passes to adb.
4. The `MODE` constant at the top of `python/main.py` selects what runs, and every change needs another deploy. Calibrate in this order.
   - `motion_check` performs the starter's single forward pulse.
   - `turn_check` turns ten search look steps left and ten right, makes three short slow turns back and forth, then turns left by pulses of rising length. `tools/turn_calc.py` turns what it shows into the turn settings, see Calibration.
   - `detect_only` prints every detection while the motors stay idle.
   - `play` runs the full routine and writes one CSV row per decision under the app's `data/runs/` folder.
5. For a scored attempt set `MODE = "play"`, place the robot at the start position and short-press the camera BOOT button. After the red, yellow and green countdown the robot makes one attempt and stops, and the next attempt needs another press.

`make pull-logs` copies the run logs back for plotting with `tools/plot_run.py`.

## Calibration

Every controller threshold lives in `ControlConfig` in `python/control.py`, whose defaults the simulator and tests use. The values measured on the event robot override them in `FIELD` in `python/field_config.py`, which the robot runs with and `tools/turn_calc.py` reads. The line sensor polarity is the `black_bit` argument of `LineSensorGuard` in `python/hardware.py`.

| Setting | How to measure it | Measured value |
| --- | --- | --- |
| `turn_rad_per_speed_ms`, `pulse_dead_ms`, `right_turn_scale` and `center_band` | Set `MODE = "turn_check"` in `python/main.py`, put the robot on the field surface and press BOOT. Read three things against a tape line, the angle ten left turns made, which are sized to add up to one full turn when the current settings are right, which side of its starting direction ten right turns leave the robot and by how much, such as 20L or 15R, and the shortest of the final rising pulses after which the chassis has clearly turned, not just buzzed or twitched. Then run `python tools/turn_calc.py --left-deg <angle> --end <side reading> --dead-ms <shortest>` before editing anything, because it reads the settings the robot ran with. It sets the turn rate to 0.75 of the measured one, so real turns run faster than the controller assumes. In simulation real turns from 0.9 to 1.5 times the assumed rate scored 198 to 199 of 200 while 0.8 times scored 188, and a battery that slows the turns by a quarter during the day still leaves them inside that range. It also widens `center_band` until the slowest centring turn fits inside it, so centring does not rock across the ball, and never narrows it. Repeat the check after changing `turn_speed`, `hfov_deg` or `look_step_frac`, which change the pulse length. Calibrate these first, because search, scan and centring all depend on them | 7.83e-6, 60 ms, 1.0 and 0.08, from ten left look steps that turned about 240 degrees where 350 were expected, right look steps that came back to the start, and a ladder that turned about 45 degrees in all |
| `turn_speed` | If the look steps of `turn_check` barely turn the robot, raise this by 20. The sketch gives each wheel half the PWM for a turn that it gives for driving, so turns are the first move to stall | to be measured |
| `hfov_deg` | Read the angle between the left and right image edges with a ruler at a known distance | to be measured |
| `min_turn_speed` | `turn_check` makes three turns at this speed, right, left and right again so each one starts against the gear backlash of the last, and each as long as `min_pulse_ms`. If any of them leaves the robot still, raise this by 10 and run the check again. Do not lower it to make centring settle, since a stall costs more than a wider `center_band`. Centring slows toward this speed only while overshoots keep its turns shrunk | to be measured |
| `near_y` | Place the ball where lining up should start, about 25 cm ahead, and read `ball.y` in `detect_only` | 0.60, since a ball 25 cm ahead read 0.625 every frame, and FOMO reports y only at cell centres such as 0.542, 0.625 and 0.708 |
| `align_band` | Place the ball in front of the goal at several spots and read how far apart the two centres are | to be measured |
| `black_bit` in `python/hardware.py` | Start the app with the robot over tape and over the floor, and compare `line_digital` | to be measured |
| `MOTOR_POSITIVE_DIR` in `sketch/sketch.ino` | Use the motor calibration supplied with the robot, then check that every wheel turns both ways and that rotate pulses spin the robot on the spot | to be filled |
| `confirm_hits` | Keep 1 unless `detect_only` reports a ball where there is none, then set 2 | to be measured |
| `HSV_LOWER` and `HSV_UPPER` in `python/main.py` | Only needed without a model. Adjust until `detect_only` reports the ball in every position and nothing once it is removed | to be measured |
| `sidestep_pulses` | Ask where the robot, ball and goal start. If they start in one line, set 3 and check on the field that three strafe pulses move the robot about one ball width, adding or removing pulses to match | to be confirmed |
| `search_when_lost` | Ask the judges whether the robot may search for the ball after losing it, or must stop | to be confirmed |

The floor surface, the lighting and the battery voltage reported as `battery_mv` in the startup sensors line, all taken when these values were measured, are to be filled. Turning and detection both depend on them.

## Testing without the robot

All of these run on an ordinary computer.

```sh
python tests/run_tests.py
python sim/run_sim.py --episodes 300
python sim/robustness.py --episodes 150
```

The unit tests cover the stream parser against the firmware's exact framing, the frozen and corrupt frame checks, the colour detector, the controller's decisions and its rejection of threshold combinations that would loop forever, every way the run loop can end and the pulse correction it sends, the line sensor guard, the turn calibration, the frame capture tool, and an import of `main.py` against a stand-in for the board-only Arduino module.

The simulator runs the same controller and run loop against a top-down model of the robot, ball, taped field and camera. Its physical constants are guesses, so it shows that the decision logic is sound, not how the real robot will score. On 300 random starts with the default settings and no calibration it scored 299 times with a median of 23.6 seconds. The robustness sweep changes one assumption at a time and found that a start-up loss in every pulse and an overestimated turn rate hurt most, which is why both are calibrated first and the turn rate is set below the measured one.

## Results

Recorded on the event field. Each time runs from the BOOT press to the ball crossing the goal line.

| Attempt | Robot start | Ball position | Outcome | Time (s) | Notes |
| --- | --- | --- | --- | --- | --- |
| 1 | to be filled | to be filled | to be filled | to be filled | |

Goals out of attempts and the median time to a goal are to be filled.

## Credits

- The starter app, robot driver, firmware and Makefile come from the organisers' uno-q-hackout repository.
- The Edge Impulse Linux Python SDK 1.2.2 runs the `.eim` model. It is copied unchanged into `python/edge_impulse_linux/` with its Clear BSD license, because the board has no Internet access to install it.
- The sources of the training images and labels are to be filled. They are the organisers' Edge Impulse project, our own robot camera captures, or both.
- The orbit-to-align idea follows the shooting behaviour in the NAIST RoboCup-99 team description, and the reasoning about turns and slides follows the visual servoing tutorial by Chaumette and Hutchinson.
- Backing off after a lost ball and the line-sensor priority were informed by the 2024 RoboCup Junior Open soccer code published by chamburr.
