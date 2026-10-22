# Soft Robotics EE/CS

Code for the FAMS (NYU) water-jet ureteroscopy robot, starting with Model 1.1 (due Nov 10, 2026).

## Layout

- `firmware/`: Arduino (PlatformIO) code for the pump, the dual linear actuator steering, and the load cell
- `logging/`: Python scripts that record serial data to CSV
- `tracking/`: video tracking of tip speed and bending angle
- `data/`: test data (CSV)

## Hardware notes

Pump: Kamoer KCPA600 (24 V, brushed DC), driven by a BTS7960 motor driver.

| Signal | Arduino pin |
|---|---|
| R_EN | 8 |
| RPWM | 9 |
| L_EN | 12 |
| LPWM | 11 |
| Potentiometer | A0 |

Steering: two linear actuators on one L298N driver, using the same Arduino.
