# Test bench firmware

One Arduino runs the jet pump, the two steering actuators and the load cell, takes commands over USB serial, and streams timestamped data. The Python scripts in `datalog/` talk to it.

`firmware/pump` (Aiden's knob-only pump sketch) still works on its own; this project is the one to use for testing.

## Flash it

```bash
cd firmware/testbench
pio run -t upload          # Leonardo (the lab board)
pio run -e uno -t upload   # if it is an Uno/RedBoard instead (e.g. Taeheon's Uno R3)
pio device monitor         # type commands by hand, 115200 baud
```

Then run `python datalog/bringup.py` to check each part (see `docs/test_workflow.md`).

## Test without a board

```bash
pio test -e native -e native_pwm_dir
```

This builds `src/main.cpp` for this computer against a fake Arduino (`test/shim/`) and runs the tests in `test/test_firmware/`: commands, timed moves, the travel limit, emergency stop, timeout, manual mode, MARK, load cell calibration and EEPROM. It checks the logic, not the electronics. Run it after changing the firmware.

## Wiring

All pins are set in `include/config.h`. Change them there if the wiring differs. The actuator pins match Aiden's wiring of the lab setup (11/4).

| Part | Signal | Pin |
|---|---|---|
| Pump driver (BTS7960) | R_EN / RPWM / L_EN / LPWM | 8 / 9 / 12 / 11 |
| Actuator A (L298N channel A) | ENA / IN1 / IN2 | 6 / 7 / 2 |
| Actuator B (L298N channel B) | ENB / IN3 / IN4 | 5 / 3 / 4 |
| Load cell (HX711) | DOUT / SCK | A1 / A2 |
| Pump knob | wiper | A0 |
| Joystick | X axis | A3 |
| Emergency stop | button to GND | 10 |
| Status LED | onboard LED, or an external LED + resistor | 13 |
| Spare analog inputs | actuator position or pressure sensor | A4, A5 |

- Power: the pump has its own 24 V supply; the actuators share one 12 V supply (6 A or more) into the L298N. Connect the pump driver, L298N and Arduino grounds together.
- Remove the ENA/ENB jumpers on the L298N, or the actuators run at full speed whatever the PWM.
- The L298N is rated about 2 A per channel and drops about 2 V. Start at low PWM and feel whether it gets hot; if it does, use a bigger driver (below).
- If no emergency stop button is wired, set `USE_ESTOP 0`, or a floating pin 10 can trip it.
- Pin 13 is an output (status LED). Do not connect it to GND or 5V. Pin 10 is only for the emergency stop: connecting it to GND stops everything. There are no actuator buttons; drive the actuators from the computer (`ACT`, `STEER`) or the joystick in manual mode.

### Other actuator drivers

Set `ACT_DRIVER` in `config.h`:

- `ACT_DRIVER_EN_IN` (default): L298N as above. A BTS7960 also works this way: R_EN and L_EN together on the EN pin (6 or 5), RPWM on IN1, LPWM on IN2.
- `ACT_DRIVER_PWM_DIR`: drivers with one PWM pin and one direction pin (e.g. Cytron MD10C, MDD10A). PWM on the EN pin, DIR on IN1; IN2 is not used.

## Travel limit

There is no sensor on the actuators, so the firmware estimates each one's travel as (PWM / 255) × run time, in milliseconds at full speed, counted from power-up or the last `ZERO`. It reports this as `pos_a` / `pos_b`. When the estimate reaches ±`ACT_TRAVEL_LIMIT_MS` (default 2000), that actuator stops and will only move back. This keeps a syringe from being pushed to the end (a pressure spike in the bellows) or pulled out.

The actuator is slower than full speed under load and at part PWM, so the real travel is never more than the estimate and the limit errs on the safe side. It is still an estimate: set the syringes to their middle (tip straight) and send `ZERO` before steering. To choose the limit, time how long a full-speed move takes to use the safe part of the syringe stroke, then set it with `LIMIT <ms>` (or in `config.h` for a permanent default).

## Modes

- **SERIAL** (default): the computer sends commands.
- **MANUAL**: the knob sets pump speed and the joystick steers (left and right drive the actuators in opposite directions). It only starts once the knob is at zero and the joystick is centered. The travel limit applies here too.

The emergency stop turns everything off in both modes. After it is released, outputs stay off until a new command arrives (SERIAL) or the knob and joystick are back at zero (MANUAL).

## Commands

One command per line, not case sensitive. The board replies `OK ...` or `ERR ...`. `[ms]` means optional: with it, the output turns itself off after that many milliseconds and the board prints `# done ...`.

| Command | What it does |
|---|---|
| `PING` | Replies `OK PONG` |
| `INFO` | Prints firmware info and the column header |
| `HELP` | Lists the commands |
| `MODE SERIAL` / `MODE MANUAL` | Switch modes (everything stops) |
| `PUMP <-255..255> [ms]` | Pump PWM. Negative runs it backwards |
| `ACT A <-255..255> [ms]` / `ACT B ...` | One actuator. Positive extends |
| `STEER <-255..255> [ms]` | A extends and B retracts at that speed (negative is the other way) |
| `STOP` | Everything off |
| `ZERO` | Set both travel estimates to 0 at the current position |
| `LIMIT <ms>` | Travel limit in ms at full speed (0 = no limit) |
| `MARK` | Turn the status LED on for 200 ms and send a sample with `mark` = 1 (for lining up video) |
| `TARE` | Zero the load cell (takes about 1 s) |
| `CALIB <grams>` | After `TARE`, put a known mass on the load cell and send its mass. The factor is saved in EEPROM |
| `SCALE <counts_per_gram>` | Set the load cell factor directly (also saved) |
| `RATE <0..100>` | Telemetry rate in Hz (default 20, 0 = off) |
| `TIMEOUT <ms>` | Stop everything if no command arrives for this long (0 = off, the default). It also cuts a timed move longer than this |

A new command for an output replaces a timed one that is still running.

## Data format

```
I,fams-sr-testbench,0.2.1,scale=455.000,hx711=ok,limit_ms=2000,driver=en_in
H,t_ms,mode,pump,act_a,act_b,pos_a,pos_b,force_g,an1,an2,pot,joy,estop,mark
D,10234,S,0,150,-150,412,-412,3.21,512,498,0,511,0,0
```

- `t_ms` is the board's clock in milliseconds; use it for timing.
- `pump`, `act_a`, `act_b` are the outputs actually applied (0 while the emergency stop is pressed).
- `pos_a`, `pos_b` are the travel estimates in ms at full speed (see Travel limit).
- `force_g` is in grams-force (1 g = 9.81 mN); it is empty if no HX711 is found.
- `an1`, `an2`, `pot`, `joy` are raw ADC readings (0-1023).
- `mark` is 1 while the MARK flash is on.
- Lines starting with `#` are status messages: `# done ...`, travel limit, emergency stop, timeout.

## Notes

- The HX711 gives 10 readings per second unless its RATE pin is tied high (80 per second), so `force_g` repeats between updates at 20 Hz telemetry.
- `PUMP_PWM_20KHZ 1` in `config.h` moves the pump PWM from about 490 Hz to 20 kHz to stop motor whine. It compiles for both boards but has not been tried on the pump yet.
- Actuator direction depends on wiring; flip `ACT_A_REVERSED` / `ACT_B_REVERSED` if "extend" retracts. `bringup.py` tells you which.
- Version 0.2.0 also fixes a bug in 0.1.0 where, with `TIMEOUT` set, a command could be stopped right away if the clock ticked while it was being read.
