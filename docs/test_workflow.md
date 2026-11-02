# Standard test workflow (draft)

How to run the bench tests for Model 1.1 so every session produces comparable data. Draft for the team to review; change it as the setup changes.

## First time on new wiring: bring-up check

After wiring and flashing `firmware/testbench`:

```bash
python datalog/bringup.py               # --skip estop loadcell for parts not wired yet
```

It walks through telemetry, emergency stop, load cell, pump, actuator A, actuator B, steering and the spare analog inputs. Each move is short and stops by itself. It tells you what to fix: wrong actuator direction, a floating emergency stop pin, a load cell that does not respond, and so on. The report is saved next to the data as `data/<date>_bringup.txt`.

## Before every session

1. Flash `firmware/testbench` if the board has other code on it. `python datalog/log_data.py --duration 3` should print `Connected ... fams-sr-testbench 0.2.0`.
2. Check the emergency stop: press it and confirm the pump stops.
3. Keep water away from the Arduino and drivers; route tubing below the electronics.
4. Load cell: `TARE` with nothing on it, then `CALIB <grams>` with a known mass (a 20 g or 50 g weight). Re-check with a second mass; it should read within about 2%.
5. Steering: set the syringes to their middle so the tip is straight, then `ZERO`. Check the travel limit (`INFO` shows `limit_ms`) suits the syringes.
6. Camera: fixed overhead, frame parallel to the table, even lighting, a ruler in the frame at the height of the tip. Put a colored marker on the tip (and a second color at the base of the bending segment for steering tests). Keep the board's status LED (pin 13) in view, or an external LED on pin 13, for syncing.
7. Write down the tip version, tubing (ID, OD, length) and anything unusual. These go in `--meta`.

## Test A: pump flow calibration

Gives PWM → flow rate. Needed before any thrust number means anything.

```bash
python datalog/sweep.py flow --pwm 80 120 160 200 240 255 --hold 30 --repeat 3 \
  --meta tubing=0.8mmID_60cm tip=V21
```

At each step: outlet into an empty cup, press Enter, the pump runs 30 s, then weigh the water and type the mass. Run it with the real tether and tip attached; flow through the 0.8 mm tubing is much lower than the pump's rated 520 mL/min.

## Test B: thrust

Tip fixed to the load cell with the jets free to discharge (into a tank, not against a wall).

```bash
python datalog/sweep.py thrust --repeat 3 --meta tip=V21 tubing=0.8mmID_60cm
```

Report thrust in mN with the standard deviation over the 3 sweeps, next to the ideal jet thrust at the measured flow (see Figures). Use the measured flow from Test A, not the pump's rated flow.

## Test C: forward movement

1. Start logging so the pump setting is recorded with a timestamp:
   ```bash
   python datalog/log_data.py --label forward --meta tip=V21 setup=open_water trial=1
   ```
2. Start the video, type `MARK` (the LED flashes), then `PUMP <pwm>`; type `STOP` when the tip reaches the end.
3. Track the video:
   ```bash
   python tracking/track_tip.py forward_1.mov --hsv <range> --px-per-mm <scale> --plot
   ```
4. Do 3 trials per PWM setting. Report mean speed ± SD.

## Test D: bidirectional steering

1. Tip straight, syringes at their middle. Start the video, then:
   ```bash
   python datalog/sweep.py steer --pwm 100 150 200 --hold 1 --repeat 3 --meta tip=V21
   ```
   It sends `ZERO` and a `MARK`, then at each PWM steers one way for 1 s, holds, comes back, holds, and does the other side; a second `MARK` at the end. Every move is timed by the firmware. It refuses moves that would pass the travel limit.
2. Track with both markers and the LED box:
   ```bash
   python tracking/track_tip.py steer_1.mov --select-led            # gives --led X Y W H
   python tracking/track_tip.py steer_1.mov --hsv <tip range> --hsv2 <base range> \
     --led X Y W H --px-per-mm <scale> --plot
   ```
3. Line the video up with the log and merge them:
   ```bash
   python analysis/merge_video_log.py steer_1_track.csv data/<date>_steer.csv --plot
   ```
   It matches the first LED flash with the first `MARK` and checks the match with the last one (it warns if they disagree by more than 0.1 s).
4. Report the maximum bend each way, and bend angle vs. actuator travel. Do 3 repeats to show hysteresis and repeatability. `pos_a` is the travel estimate; if a position sensor is wired to an1, use `--x an1` instead.

## Figures

```bash
python analysis/plot_results.py flow data/*_flow_summary.csv
python analysis/plot_results.py thrust data/*_thrust_summary.csv --flow data/*_flow_summary.csv
python analysis/plot_results.py steer steer_1_merged.csv steer_2_merged.csv
python analysis/plot_results.py speed 120:forward_1_track.csv 120:forward_2_track.csv 180:forward_3_track.csv
```

Each prints the numbers behind the figure and saves a PNG. `thrust --flow` adds thrust vs. measured flow next to the ideal jet thrust F = ρQ²cosθ/A (2 jets of 0.8 mm at 4° by default; change with `--jets`, `--jet-d-mm`, `--jet-angle-deg`). For reference, the paper's 112.7 mN is this ideal value at 640 mL/min.

## Data

- Files go in `data/` named `<date>_<test>.csv` (the scripts do this by default). Keep videos out of git (they are ignored); store them in the SR Drive with the same name.
- Every CSV starts with `# key: value` metadata lines; load it with `pandas.read_csv(path, comment="#")`.
- Note anything that went wrong with `!` while logging, e.g. `!tubing kinked at 20 s`.
