# Robocar solution

This robot finds a ball with its camera, lines the ball up with the goal, pushes it across the goal line, and goes back for the ball when a shot misses. Everything runs on the Arduino UNO Q. The Linux side decodes camera frames, runs the detector and makes every decision, and the MCU side drives the motors with timed stops.

The measured calibration and results sections are filled in on the event day.

## How it works

Each decision follows the same cycle.

1. A background thread reads the camera's MJPEG stream and keeps only the newest frame with the time it arrived.
2. After every move the robot waits for a frame that arrived after the move finished, so no decision is based on a frame captured while the robot was moving.
3. An Edge Impulse FOMO model finds the `soccerball` and the `goal` and reports their centres as fractions of the image. When no model loads, a colour threshold finds the ball.
4. The controller picks one short move, and the robot performs it as a timed pulse that the MCU stops on its own.

The controller is a state machine.

| Phase | What the robot does | Leaves when |
| --- | --- | --- |
| search | Turns half a field of view between looks, toward the edge the ball rolled over, else toward the remembered goal, else toward the side the ball was last seen, and backs off after each lap to uncover a ball sitting below the camera | The ball is seen, or three laps pass |
| approach | Turns to centre the ball, then drives toward it in pulses that shrink as it gets closer | The ball is close enough to line up |
| scan | Spins in place to find the goal while remembering where the ball was, then turns back to the ball in one direction chosen once, so a miscalibrated turn rate cannot leave it rocking over the same arc | The goal has been seen and the ball is back in view |
| lineup | Slides sideways while turning to keep the ball centred, which orbits the ball until the goal sits directly behind it | The ball and goal line up, or the orbit budget runs out |
| push | Drives forward in short pulses, and keeps going while the goal stays ahead even after the ball drops below the camera | The ball and goal drift apart, the ball is lost, or the push budget runs out |
| recover | Backs off to widen the view | It goes back to approach if the ball is visible, and to search otherwise |

Lining up works because a sideways slide moves the nearby ball across the image more than the distant goal, which changes the gap between them, while a turn moves both by the same amount and leaves the gap alone. The two corrections therefore barely interfere, and neither needs a distance estimate.

The robot also keeps a rough heading by adding up the turns it has commanded. That lets it remember which way the goal lay after the goal leaves the image, so it orbits the short way round. Only a sighting from the last few moves can start a push, because the remembered heading drifts with every turn.

## Safety

- Every move is a short timed pulse, and the MCU stops the motors when the pulse ends even if Python hangs.
- A run ends when no new frame arrives in time, when the newest frame is too old, when the camera keeps resending an identical frame, when inference takes too long, or when the run time limit passes.
- A restart always needs a press of the camera BOOT button.
- When the line sensor sees the taped boundary, the robot backs off before anything else happens. This guard stays off until the tape polarity has been checked on the field and `LINE_GUARD_ENABLED` in `python/main.py` is switched on. If the sensor still reads tape after the allowed number of back-offs in a row, the run ends, because a wrong polarity setting or a robot that crossed the edge sideways would otherwise keep it reversing.

## Training data

The camera's exposure and white balance cannot be changed through its driver, so robustness to the venue lighting has to come from the training images. `tools/capture_frames.py` saves frames straight from the robot camera while a laptop is on the camera's Wi-Fi, which keeps training images on the same sensor and JPEG path the robot sees while driving.

## Running it

The `MODE` constant at the top of `python/main.py` selects what runs.

- `motion_check` performs the starter's single forward pulse.
- `detect_only` prints every detection while the motors stay idle.
- `play` runs the full routine and writes one CSV row per decision under the app's `data/runs/` folder.

Deploy from a terminal in this folder.

```sh
make flash
make logs
```

On Windows run these in Git Bash. The `GNUmakefile` here stops Git Bash from rewriting the `/tmp` and `TMPDIR` arguments that the Makefile passes to adb. `make pull-logs` copies the run logs back for plotting with `tools/plot_run.py`.

## Calibration

Every controller threshold lives in `ControlConfig` in `python/control.py`. The line sensor polarity is the `black_bit` argument of `LineSensorGuard` in `python/hardware.py`.

| Setting | How to measure it | Measured value |
| --- | --- | --- |
| `turn_rad_per_speed_ms` | Turn ten 420 ms pulses at speed 120, measure the total angle in radians, and divide by 10, by 120 and by 420. The pulse length matches one search look step, since short pulses are expected to lose a larger share of their angle to motor spin-up. The look step is computed from this value, so if the result moves it far from 420 ms, measure again at the new length. Calibrate this first, because the search and scan both depend on it, and again after changing `turn_speed`, `hfov_deg` or `look_step_frac` | to be measured |
| `hfov_deg` | Read the angle between the left and right image edges with a ruler at a known distance | to be measured |
| `near_y` | Place the ball where lining up should start, about 25 cm ahead, and read `ball.y` in `detect_only` | to be measured |
| `align_band` | Place the ball in front of the goal at several spots and read how far apart the two centres are | to be measured |
| `black_bit` in `python/hardware.py` | Start the app with the robot over tape and over the floor, and compare `line_digital` | to be measured |
| `search_when_lost` | Ask the judges whether the robot may search for the ball after losing it, or must stop | to be confirmed |

## Testing without the robot

All of these run on an ordinary computer.

```sh
python tests/run_tests.py
python sim/run_sim.py --episodes 300
python sim/robustness.py --episodes 150
```

The unit tests cover the stream parser against the firmware's exact framing, the frozen and corrupt frame checks, the colour detector, the controller's decisions and its rejection of threshold combinations that would loop forever, every way the run loop can end, the line sensor guard, the frame capture tool, and an import of `main.py` against a stand-in for the board-only Arduino module.

The simulator runs the same controller and run loop against a top-down model of the robot, ball, taped field and camera. Its physical constants are guesses, so it shows that the decision logic is sound, not how the real robot will score. On 300 random starts it scored 299 times with a median of 23.3 seconds. The robustness sweep changes one assumption at a time and found that an overestimated turn rate hurts most, which is why that value is calibrated first.

## Results

To be filled in on the event day with the number of attempts, goals, and the time of each goal on the real field.

## Credits

- The starter app, robot driver, firmware and Makefile come from the organisers' uno-q-hackout repository.
- The Edge Impulse Linux Python SDK runs the `.eim` model.
- The orbit-to-align idea follows the shooting behaviour in the NAIST RoboCup-99 team description, and the reasoning about turns and slides follows the visual servoing tutorial by Chaumette and Hutchinson.
- Backing off after a lost ball and the line-sensor priority were informed by the 2024 RoboCup Junior Open soccer code published by chamburr.
