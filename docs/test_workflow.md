# Standard test workflow (draft)

How to run the bench tests for Model 1.1 so every session produces comparable data. Draft for the team to review; change it as the setup changes.

## Before every session

1. Flash `firmware/testbench` if the board has other code on it. `python datalog/log_data.py --duration 3` should print `Connected ... fams-sr-testbench`.
2. Check the emergency stop: press it and confirm the pump stops.
3. Keep water away from the Arduino and drivers; route tubing below the electronics.
4. Load cell: `TARE` with nothing on it, then `CALIB <grams>` with a known mass (a 20 g or 50 g weight). Re-check with a second mass; it should read within about 2%.
5. Camera: fixed overhead, frame parallel to the table, even lighting, a ruler in the frame at the height of the tip. Put a colored marker on the tip (and a second color at the base of the bending segment for steering tests).
6. Write down the tip version, tubing (ID, OD, length) and anything unusual. These go in `--meta`.

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

Report thrust in mN with the standard deviation over the 3 sweeps. Compare with David's propulsion calculator at the measured flow from Test A, not the pump's rated flow.

## Test C: forward movement

1. Start logging so the pump setting is recorded with a timestamp:
   ```bash
   python datalog/log_data.py --label forward --meta tip=V21 setup=open_water trial=1
   ```
2. Start the video, then type `PUMP <pwm>`; type `STOP` when the tip reaches the end.
3. Track the video:
   ```bash
   python tracking/track_tip.py forward_1.mov --hsv <range> --px-per-mm <scale> --plot
   ```
4. Do 3 trials per PWM setting. Report mean speed ± SD.

## Test D: bidirectional steering

1. Log with `log_data.py --label steer`. `an1`/`an2` record actuator position or syringe pressure if those sensors are wired.
2. From straight, type `STEER 150` for a fixed time (e.g. 2 s), then `STOP`. Return to straight with `STEER -150` for the same time. Repeat for the other side, then at 2-3 speeds.
3. Track with both markers to get the bending angle:
   ```bash
   python tracking/track_tip.py steer_1.mov --hsv <tip range> --hsv2 <base range> --px-per-mm <scale> --plot
   ```
4. Report the maximum bend each way, and bend angle vs. actuator stroke (or syringe pressure). Do 3 repeats to show hysteresis and repeatability.

## Data

- Files go in `data/` named `<date>_<test>.csv` (the scripts do this by default). Keep videos out of git (they are ignored); store them in the SR Drive with the same name.
- Every CSV starts with `# key: value` metadata lines; load it with `pandas.read_csv(path, comment="#")`.
- Note anything that went wrong with `!` while logging, e.g. `!tubing kinked at 20 s`.
