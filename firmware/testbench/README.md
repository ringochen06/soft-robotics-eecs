# Test bench firmware

One Arduino runs the jet pump, the two steering actuators and the load cell, takes commands over USB serial, and streams timestamped data. The Python scripts in `datalog/` talk to it.

`firmware/pump` (Aiden's knob-only pump sketch) still works on its own; this project is the one to use for testing.

## Flash it

```bash
cd firmware/testbench
pio run -t upload          # Leonardo (the lab board)
pio run -e uno -t upload   # if it is a RedBoard/Uno instead
pio device monitor         # type commands by hand, 115200 baud
```

## Wiring

All pins are set in `include/config.h`. Change them there if the wiring differs.

| Part | Signal | Pin |
|---|---|---|
| Pump driver (BTS7960) | R_EN / RPWM / L_EN / LPWM | 8 / 9 / 12 / 11 |
| Actuator A (L298N channel A) | ENA / IN1 / IN2 | 5 / 4 / 7 |
| Actuator B (L298N channel B) | ENB / IN3 / IN4 | 6 / A1 / A2 |
| Load cell (HX711) | DOUT / SCK | 2 / 3 |
| Pump knob | wiper | A0 |
| Joystick | X axis | A3 |
| Emergency stop | button to GND | 10 |
| Spare analog inputs | actuator position or pressure sensor | A4, A5 |

The pump, L298N and Arduino grounds must be connected together. If no emergency stop button is wired, set `USE_ESTOP 0`, or a floating pin 10 can trip it.

## Modes

- **SERIAL** (default): the computer sends commands.
- **MANUAL**: the knob sets pump speed and the joystick steers (left and right drive the actuators in opposite directions). It only starts once the knob is at zero and the joystick is centered.

The emergency stop turns everything off in both modes. After it is released, outputs stay off until a new command arrives (SERIAL) or the knob and joystick are back at zero (MANUAL).

## Commands

One command per line. The board replies `OK ...` or `ERR ...`.

| Command | What it does |
|---|---|
| `PING` | Replies `OK PONG` |
| `INFO` | Prints firmware info and the column header |
| `HELP` | Lists the commands |
| `MODE SERIAL` / `MODE MANUAL` | Switch modes (everything stops) |
| `PUMP <-255..255>` | Pump PWM. Negative runs it backwards |
| `ACT A <-255..255>` / `ACT B ...` | One actuator. Positive extends |
| `STEER <-255..255>` | A extends and B retracts at that speed (negative is the other way) |
| `STOP` | Everything off |
| `TARE` | Zero the load cell (takes about 1 s) |
| `CALIB <grams>` | After `TARE`, put a known mass on the load cell and send its mass. The factor is saved in EEPROM |
| `SCALE <counts_per_gram>` | Set the load cell factor directly (also saved) |
| `RATE <0..100>` | Telemetry rate in Hz (default 20, 0 = off) |
| `TIMEOUT <ms>` | Stop everything if no command arrives for this long (0 = off, the default) |

## Data format

```
I,fams-sr-testbench,0.1.0,scale=455.000,hx711=ok
H,t_ms,mode,pump,act_a,act_b,force_g,an1,an2,pot,joy,estop
D,10234,S,120,0,0,3.21,512,498,0,511,0
```

- `t_ms` is the board's clock in milliseconds; use it for timing.
- `pump`, `act_a`, `act_b` are the outputs actually applied (0 while the emergency stop is pressed).
- `force_g` is in grams-force (1 g = 9.81 mN); it is empty if no HX711 is found.
- `an1`, `an2`, `pot`, `joy` are raw ADC readings (0-1023).
- Lines starting with `#` are status messages.

## Notes

- The HX711 gives 10 readings per second unless its RATE pin is tied high (80 per second), so `force_g` repeats between updates at 20 Hz telemetry.
- `PUMP_PWM_20KHZ 1` in `config.h` moves the pump PWM from about 490 Hz to 20 kHz to stop motor whine. It compiles for both boards but has not been tried on the pump yet.
- Actuator direction depends on wiring; flip `ACT_A_REVERSED` / `ACT_B_REVERSED` if "extend" retracts.
