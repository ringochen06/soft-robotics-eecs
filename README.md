# Soft Robotics EE/CS

Code for the FAMS (NYU) water-jet ureteroscopy robot, starting with Model 1.1 (due Nov 10, 2026).

## Layout

- `firmware/testbench/`: Arduino firmware for the bench: pump, the two steering actuators and the load cell on one board, controlled over serial. See its [README](firmware/testbench/README.md) for wiring and commands.
- `firmware/pump/`: the original knob-controlled pump sketch.
- `datalog/`: Python scripts that talk to the board and save CSV data.
  - `log_data.py`: record telemetry while typing commands.
  - `sweep.py`: step the pump PWM and measure thrust or flow at each step.
  - `sim_device.py`: a fake board, for trying the scripts without hardware.
- `tracking/track_tip.py`: video tracking of tip speed and bending angle.
- `docs/test_workflow.md`: the standard test procedures.
- `data/`: test data (CSV). Videos are git-ignored; keep them in the SR Drive.

## Setup

```bash
pip install -r requirements.txt   # Python 3.9+
pip install platformio            # only needed to build and flash firmware
```

## Quick start

Flash the firmware (Leonardo):

```bash
cd firmware/testbench && pio run -t upload
```

Record data while sending commands:

```bash
python datalog/log_data.py --meta tip=V21 tubing=0.8mm
# then type: PUMP 120, STEER 100, STOP, !a note, quit
```

No board at hand? Start the simulator in one terminal and pass its port:

```bash
python datalog/sim_device.py                       # prints e.g. /dev/ttys012
python datalog/log_data.py --port /dev/ttys012
```

Track a video:

```bash
python tracking/track_tip.py run.mov --pick         # get the marker's HSV range
python tracking/track_tip.py run.mov --hsv 40 80 80 80 255 255 --px-per-mm 4.2 --plot
```

## Hardware notes

Pump: Kamoer KCPA600 (24 V, brushed DC), driven by a BTS7960 motor driver.

| Signal | Arduino pin |
|---|---|
| R_EN | 8 |
| RPWM | 9 |
| L_EN | 12 |
| LPWM | 11 |
| Potentiometer | A0 |

Steering: two linear actuators pushing two syringes (hydraulic bidirectional steering), on one L298N driver, using the same Arduino.
