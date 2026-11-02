// Pin map and options for the test bench firmware.
// Change pins here after wiring; nothing else should need editing.
// Pins avoid 0/1 so the same map works on a Leonardo and an Uno/RedBoard.
#pragma once

// ---- Pump: Kamoer KCPA600 (brushed DC) on a BTS7960 driver ----
// Matches Aiden's wiring (firmware/pump).
#define PIN_PUMP_R_EN 8
#define PIN_PUMP_RPWM 9   // forward PWM (Timer1 / OC1A on both boards)
#define PIN_PUMP_L_EN 12
#define PIN_PUMP_LPWM 11  // reverse PWM

// 1 = run the forward PWM at ~20 kHz to remove motor whine (pin 9 only).
// Leave at 0 until it has been tried on the bench; 0 keeps analogWrite (~490 Hz),
// which is what the pump has been running on.
#define PUMP_PWM_20KHZ 0

// ---- Steering: two linear actuators ----
// Driver type, for both actuators:
//   ACT_DRIVER_EN_IN    L298N: PWM on EN, direction on IN1/IN2 (the default).
//                       Also fits a BTS7960 wired with R_EN + L_EN together on the
//                       EN pin and RPWM / LPWM on IN1 / IN2.
//   ACT_DRIVER_PWM_DIR  one PWM pin and one direction pin (e.g. Cytron MD10C, MDD10A):
//                       PWM on the EN pin, DIR on IN1. IN2 is not used.
#define ACT_DRIVER_EN_IN 0
#define ACT_DRIVER_PWM_DIR 1
#ifndef ACT_DRIVER
#define ACT_DRIVER ACT_DRIVER_EN_IN
#endif

#define PIN_ACT_A_EN 5    // ENA (PWM)
#define PIN_ACT_A_IN1 4
#define PIN_ACT_A_IN2 7
#define PIN_ACT_B_EN 6    // ENB (PWM)
#define PIN_ACT_B_IN1 A1  // IN3
#define PIN_ACT_B_IN2 A2  // IN4

// Flip if an actuator retracts when it should extend.
#define ACT_A_REVERSED 0
#define ACT_B_REVERSED 0

// Travel limit, to keep the syringes from being pushed to the end (pressure spike in
// the bellows) or pulled out. Travel has no sensor, so it is estimated as
// (duty / 255) x run time, i.e. milliseconds at full speed, counted from power-up or
// the last ZERO. The actuator is slower than that under load and at part duty, so the
// real travel is never more than the estimate. An actuator stops when its estimate
// reaches +/- this many ms and then only moves back. 0 = no limit. Change it at
// runtime with LIMIT <ms>; time a full-speed syringe stroke to pick the value.
#define ACT_TRAVEL_LIMIT_MS 2000

// ---- Load cell: HX711 ----
#define PIN_HX711_DOUT 2
#define PIN_HX711_SCK 3
// Default calibration factor (raw counts per gram). Overwritten by CALIB and
// stored in EEPROM. 455 is the value the summer team used for their load cell.
#define HX711_DEFAULT_SCALE 455.0f

// ---- Manual controls ----
#define PIN_POT A0        // pump speed knob (manual mode)
#define PIN_JOY_X A3      // joystick X axis (manual steering)
#define PIN_ESTOP 10      // emergency stop button to GND (INPUT_PULLUP)
#define USE_ESTOP 1       // set 0 if no button is wired, or the floating pin may trip it
#define JOY_DEADBAND 60   // ADC counts around center ignored

// ---- Status LED ----
// On while the emergency stop is pressed, and for MARK_MS after a MARK command.
// Put this LED (or an external one on the same pin) in the camera view to line up
// video with the data log.
#define PIN_STATUS_LED LED_BUILTIN  // pin 13 on both boards
#define MARK_MS 200

// ---- Extra analog inputs (actuator position feedback or pressure sensors) ----
#define PIN_AN_1 A4
#define PIN_AN_2 A5

// ---- Serial ----
#define SERIAL_BAUD 115200
#define DEFAULT_STREAM_HZ 20
