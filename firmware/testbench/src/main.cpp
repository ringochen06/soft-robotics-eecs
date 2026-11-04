// Test bench firmware for the water-jet ureteroscopy robot (Model 1.1).
//
// One Arduino drives the jet pump (BTS7960), the two steering actuators
// (L298N by default) and reads the load cell (HX711). It takes line-based
// commands over USB serial and streams timestamped CSV telemetry. See
// ../README.md for the command list and data format.

#include <Arduino.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <string.h>

#include "HX711.h"
#include "config.h"

static const char FW_NAME[] = "fams-sr-testbench";
static const char FW_VERSION[] = "0.2.1";

static const uint16_t EEPROM_MAGIC = 0xFA55;
static const int EEPROM_ADDR_MAGIC = 0;
static const int EEPROM_ADDR_SCALE = 2;

static const unsigned long MAX_DURATION_MS = 600000UL;

enum Mode { MODE_SERIAL, MODE_MANUAL };

// An output command that can switch itself off: cmd is -255..255, and when
// dur > 0 the output turns off dur ms after start.
struct Timed {
  int cmd;
  unsigned long start;
  unsigned long dur;  // 0 = until the next command
};

struct Actuator {
  char name;
  uint8_t en, in1, in2;
  bool reversed;
  Timed out;      // positive = extend
  int applied;    // what was actually driven last loop
  long travel;    // estimated travel since ZERO, in duty x ms (255 = 1 ms at full speed)
  bool limitHit;  // the travel limit message has been printed
};

static Mode mode = MODE_SERIAL;
static Timed pump = {0, 0, 0};  // positive = forward (jet)
static Actuator act[2] = {
    {'A', PIN_ACT_A_EN, PIN_ACT_A_IN1, PIN_ACT_A_IN2, ACT_A_REVERSED, {0, 0, 0}, 0, 0, false},
    {'B', PIN_ACT_B_EN, PIN_ACT_B_IN1, PIN_ACT_B_IN2, ACT_B_REVERSED, {0, 0, 0}, 0, 0, false},
};
static bool estop = false;
static bool manualArmed = false;

static unsigned long travelLimitMs = ACT_TRAVEL_LIMIT_MS;
static unsigned long lastTravelMs = 0;

static bool marking = false;
static unsigned long markStart = 0;

static uint16_t streamHz = DEFAULT_STREAM_HZ;
static unsigned long lastStreamMs = 0;
static bool streamNow = false;
static unsigned long timeoutMs = 0;  // 0 = no command timeout
static unsigned long lastCmdMs = 0;

static HX711 scale;
static bool scaleOk = false;
static float scaleFactor = HX711_DEFAULT_SCALE;
static float forceG = 0.0f;
static bool forceValid = false;

static char lineBuf[64];
static uint8_t lineLen = 0;

// ---------------------------------------------------------------- outputs

#if PUMP_PWM_20KHZ
static const uint16_t PUMP_PWM_TOP = 400;  // 16 MHz / (2 * 400) = 20 kHz

static void pumpPwmInit() {
  // Timer1 mode 10: phase-correct PWM, TOP = ICR1, no prescaler, OC1A (pin 9).
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = PUMP_PWM_TOP;
  OCR1A = 0;
}

static void pumpForwardDuty(uint8_t duty) {
  OCR1A = (uint32_t)duty * PUMP_PWM_TOP / 255;
}
#else
static void pumpPwmInit() {}

static void pumpForwardDuty(uint8_t duty) { analogWrite(PIN_PUMP_RPWM, duty); }
#endif

static void applyPump(int cmd) {
  cmd = constrain(cmd, -255, 255);
  if (cmd >= 0) {
    analogWrite(PIN_PUMP_LPWM, 0);
    pumpForwardDuty(cmd);
  } else {
    pumpForwardDuty(0);
    analogWrite(PIN_PUMP_LPWM, -cmd);
  }
}

static void driveActuator(const Actuator &a, int cmd) {
  cmd = constrain(cmd, -255, 255);
  if (a.reversed) cmd = -cmd;
#if ACT_DRIVER == ACT_DRIVER_PWM_DIR
  digitalWrite(a.in1, cmd < 0 ? HIGH : LOW);
  analogWrite(a.en, cmd < 0 ? -cmd : cmd);
#else
  if (cmd > 0) {
    digitalWrite(a.in1, HIGH);
    digitalWrite(a.in2, LOW);
    analogWrite(a.en, cmd);
  } else if (cmd < 0) {
    digitalWrite(a.in1, LOW);
    digitalWrite(a.in2, HIGH);
    analogWrite(a.en, -cmd);
  } else {
    digitalWrite(a.in1, LOW);
    digitalWrite(a.in2, LOW);
    analogWrite(a.en, 0);
  }
#endif
}

static bool outputsOff() { return estop || (mode == MODE_MANUAL && !manualArmed); }

static void applyOutputs() {
  bool off = outputsOff();
  applyPump(off ? 0 : pump.cmd);
  for (uint8_t i = 0; i < 2; i++) {
    act[i].applied = off ? 0 : act[i].out.cmd;
    driveActuator(act[i], act[i].applied);
  }
  digitalWrite(PIN_STATUS_LED, (estop || marking) ? HIGH : LOW);
}

static void setTimed(Timed &t, int cmd, unsigned long dur) {
  t.cmd = cmd;
  t.start = millis();
  t.dur = cmd == 0 ? 0 : dur;
}

static void stopAll() {
  setTimed(pump, 0, 0);
  setTimed(act[0].out, 0, 0);
  setTimed(act[1].out, 0, 0);
}

// Returns true (and turns the output off) once a timed command has run its time.
static bool expired(Timed &t, unsigned long now) {
  if (t.dur == 0 || now - t.start < t.dur) return false;
  t.cmd = 0;
  t.dur = 0;
  return true;
}

// ---------------------------------------------------------------- travel limit

// Dead reckoning from the duty actually applied since the last call.
static void updateTravel(unsigned long now) {
  unsigned long dt = now - lastTravelMs;
  if (dt == 0) return;
  lastTravelMs = now;
  for (uint8_t i = 0; i < 2; i++) act[i].travel += (long)act[i].applied * (long)dt;
}

static bool pastLimit(const Actuator &a, int cmd) {
  if (travelLimitMs == 0) return false;
  long limit = (long)travelLimitMs * 255L;
  return (cmd > 0 && a.travel >= limit) || (cmd < 0 && a.travel <= -limit);
}

// Returns cmd, or 0 if it would push the actuator further past its travel limit.
static int limitCmd(Actuator &a, int cmd) {
  if (!pastLimit(a, cmd)) {
    a.limitHit = false;
    return cmd;
  }
  if (!a.limitHit) {
    a.limitHit = true;
    Serial.print(F("# act "));
    Serial.print(a.name);
    Serial.println(F(" reached its travel limit and stopped; it can only move back (or send ZERO / LIMIT)"));
  }
  return 0;
}

static void enforceLimits() {
  for (uint8_t i = 0; i < 2; i++) {
    int cmd = limitCmd(act[i], act[i].out.cmd);
    if (cmd != act[i].out.cmd) setTimed(act[i].out, 0, 0);
  }
}

// ---------------------------------------------------------------- inputs

static int joystickSpeed(int raw) {
  int dx = raw - 512;
  if (abs(dx) < JOY_DEADBAND) return 0;
  int mag = map(abs(dx) - JOY_DEADBAND, 0, 511 - JOY_DEADBAND, 0, 255);  // full right (dx = 511) = 255
  mag = constrain(mag, 0, 255);
  return dx > 0 ? mag : -mag;
}

static void readEstop() {
#if USE_ESTOP
  bool pressed = digitalRead(PIN_ESTOP) == LOW;
  if (pressed && !estop) {
    stopAll();
    manualArmed = false;
    Serial.println(F("# estop pressed: all outputs off"));
  } else if (!pressed && estop) {
    Serial.println(F("# estop released"));
  }
  estop = pressed;
#endif
}

static void updateManual() {
  int pot = analogRead(PIN_POT);
  int steer = joystickSpeed(analogRead(PIN_JOY_X));
  if (!manualArmed) {
    // Only start once the knob is at zero and the joystick is centered,
    // so the pump and actuators never jump on at switch-on or after an estop.
    if (pot < 20 && steer == 0) {
      manualArmed = true;
      Serial.println(F("# manual mode armed"));
    }
    return;
  }
  setTimed(pump, map(pot, 0, 1023, 0, 255), 0);
  setTimed(act[0].out, steer, 0);
  setTimed(act[1].out, -steer, 0);
}

static void updateForce() {
  if (scaleOk && scale.is_ready()) {
    forceG = scale.get_units(1);
    forceValid = true;
  }
}

// ---------------------------------------------------------------- scale storage

static void loadScaleFactor() {
  uint16_t magic;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC) {
    float f;
    EEPROM.get(EEPROM_ADDR_SCALE, f);
    if (isfinite(f) && f != 0.0f) scaleFactor = f;
  }
}

static void saveScaleFactor() {
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR_SCALE, scaleFactor);
}

// ---------------------------------------------------------------- telemetry

static void printHeader() {
  Serial.println(F("H,t_ms,mode,pump,act_a,act_b,pos_a,pos_b,force_g,an1,an2,pot,joy,estop,mark"));
}

static void printInfo() {
  Serial.print(F("I,"));
  Serial.print(FW_NAME);
  Serial.print(',');
  Serial.print(FW_VERSION);
  Serial.print(F(",scale="));
  Serial.print(scaleFactor, 3);
  Serial.print(F(",hx711="));
  Serial.print(scaleOk ? F("ok") : F("missing"));
  Serial.print(F(",limit_ms="));
  Serial.print(travelLimitMs);
  Serial.print(F(",driver="));
  Serial.println(ACT_DRIVER == ACT_DRIVER_PWM_DIR ? F("pwm_dir") : F("en_in"));
  printHeader();
}

static void printTelemetry(unsigned long now) {
  bool off = outputsOff();
  Serial.print(F("D,"));
  Serial.print(now);
  Serial.print(',');
  Serial.print(mode == MODE_SERIAL ? 'S' : 'M');
  Serial.print(',');
  Serial.print(off ? 0 : pump.cmd);
  Serial.print(',');
  Serial.print(off ? 0 : act[0].out.cmd);
  Serial.print(',');
  Serial.print(off ? 0 : act[1].out.cmd);
  Serial.print(',');
  Serial.print(act[0].travel / 255L);
  Serial.print(',');
  Serial.print(act[1].travel / 255L);
  Serial.print(',');
  if (forceValid) Serial.print(forceG, 2);
  Serial.print(',');
  Serial.print(analogRead(PIN_AN_1));
  Serial.print(',');
  Serial.print(analogRead(PIN_AN_2));
  Serial.print(',');
  Serial.print(analogRead(PIN_POT));
  Serial.print(',');
  Serial.print(analogRead(PIN_JOY_X));
  Serial.print(',');
  Serial.print(estop ? 1 : 0);
  Serial.print(',');
  Serial.println(marking ? 1 : 0);
}

// ---------------------------------------------------------------- commands

static bool parseInt(const char *s, long lo, long hi, long *out) {
  if (s == NULL) return false;
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0' || v < lo || v > hi) return false;
  *out = v;
  return true;
}

static bool parseFloat(const char *s, float *out) {
  if (s == NULL) return false;
  char *end;
  double v = strtod(s, &end);
  if (end == s || *end != '\0') return false;
  *out = (float)v;
  return true;
}

// Optional duration argument: missing = 0 (run until the next command).
static bool parseDuration(const char *s, unsigned long *out) {
  long v = 0;
  if (s != NULL && !parseInt(s, 1, (long)MAX_DURATION_MS, &v)) return false;
  *out = (unsigned long)v;
  return true;
}

static void ok(const __FlashStringHelper *what) {
  Serial.print(F("OK "));
  Serial.println(what);
}

static void err(const __FlashStringHelper *why) {
  Serial.print(F("ERR "));
  Serial.println(why);
}

static bool canDrive() {
  if (estop) {
    err(F("estop is pressed"));
    return false;
  }
  if (mode != MODE_SERIAL) {
    err(F("in manual mode; send MODE SERIAL first"));
    return false;
  }
  return true;
}

static bool canMove(const Actuator &a, int cmd) {
  if (!pastLimit(a, cmd)) return true;
  Serial.print(F("ERR act "));
  Serial.print(a.name);
  Serial.println(F(" is at its travel limit; it can only move back (or send ZERO / LIMIT)"));
  return false;
}

static void printHelp() {
  Serial.println(F("# PING | INFO | HELP"));
  Serial.println(F("# MODE SERIAL|MANUAL"));
  Serial.println(F("# PUMP <-255..255> [ms]      jet pump PWM (negative = reverse)"));
  Serial.println(F("# ACT A|B <-255..255> [ms]   one actuator (positive = extend)"));
  Serial.println(F("# STEER <-255..255> [ms]     A extends and B retracts (negative = opposite)"));
  Serial.println(F("#   [ms] = stop by itself after this long"));
  Serial.println(F("# STOP                       everything off"));
  Serial.println(F("# ZERO                       set the actuator travel estimate to 0 here"));
  Serial.println(F("# LIMIT <ms>                 travel limit in ms at full speed (0 = off)"));
  Serial.println(F("# MARK                       flash the status LED (video sync)"));
  Serial.println(F("# TARE | CALIB <grams> | SCALE <counts_per_gram>"));
  Serial.println(F("# RATE <0..100>              telemetry Hz (0 = off)"));
  Serial.println(F("# TIMEOUT <ms>               stop if no command for this long (0 = off)"));
}

static void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (cmd == NULL) return;
  char *a1 = strtok(NULL, " \t");
  char *a2 = strtok(NULL, " \t");
  char *a3 = strtok(NULL, " \t");
  long v;
  float f;
  unsigned long dur;

  lastCmdMs = millis();

  if (!strcasecmp(cmd, "PING")) {
    ok(F("PONG"));
  } else if (!strcasecmp(cmd, "INFO")) {
    printInfo();
  } else if (!strcasecmp(cmd, "HELP")) {
    printHelp();
  } else if (!strcasecmp(cmd, "MODE")) {
    if (a1 && !strcasecmp(a1, "SERIAL")) {
      mode = MODE_SERIAL;
      stopAll();
      ok(F("MODE SERIAL"));
    } else if (a1 && !strcasecmp(a1, "MANUAL")) {
      mode = MODE_MANUAL;
      stopAll();
      manualArmed = false;
      ok(F("MODE MANUAL (turn the knob to zero and center the joystick to arm)"));
    } else {
      err(F("usage: MODE SERIAL|MANUAL"));
    }
  } else if (!strcasecmp(cmd, "PUMP")) {
    if (!parseInt(a1, -255, 255, &v) || !parseDuration(a2, &dur)) return err(F("usage: PUMP <-255..255> [ms]"));
    if (!canDrive()) return;
    setTimed(pump, (int)v, dur);
    ok(F("PUMP"));
  } else if (!strcasecmp(cmd, "ACT")) {
    if (!a1 || !parseInt(a2, -255, 255, &v) || !parseDuration(a3, &dur)) return err(F("usage: ACT A|B <-255..255> [ms]"));
    Actuator *a = NULL;
    if (!strcasecmp(a1, "A")) a = &act[0];
    if (!strcasecmp(a1, "B")) a = &act[1];
    if (a == NULL) return err(F("usage: ACT A|B <-255..255> [ms]"));
    if (!canDrive() || !canMove(*a, (int)v)) return;
    setTimed(a->out, (int)v, dur);
    ok(F("ACT"));
  } else if (!strcasecmp(cmd, "STEER")) {
    if (!parseInt(a1, -255, 255, &v) || !parseDuration(a2, &dur)) return err(F("usage: STEER <-255..255> [ms]"));
    if (!canDrive() || !canMove(act[0], (int)v) || !canMove(act[1], (int)-v)) return;
    setTimed(act[0].out, (int)v, dur);
    setTimed(act[1].out, (int)-v, dur);
    ok(F("STEER"));
  } else if (!strcasecmp(cmd, "STOP")) {
    stopAll();
    ok(F("STOP"));
  } else if (!strcasecmp(cmd, "ZERO")) {
    for (uint8_t i = 0; i < 2; i++) {
      act[i].travel = 0;
      act[i].limitHit = false;
    }
    ok(F("ZERO"));
  } else if (!strcasecmp(cmd, "LIMIT")) {
    if (!parseInt(a1, 0, (long)MAX_DURATION_MS, &v)) return err(F("usage: LIMIT <ms> (0 = off)"));
    travelLimitMs = (unsigned long)v;
    act[0].limitHit = act[1].limitHit = false;
    Serial.print(F("OK LIMIT "));
    Serial.println(travelLimitMs);
  } else if (!strcasecmp(cmd, "MARK")) {
    marking = true;
    markStart = millis();
    streamNow = true;  // a sample right at the LED edge
    ok(F("MARK"));
  } else if (!strcasecmp(cmd, "TARE")) {
    if (!scaleOk) return err(F("hx711 missing"));
    scale.tare(10);
    ok(F("TARE"));
  } else if (!strcasecmp(cmd, "CALIB")) {
    if (!scaleOk) return err(F("hx711 missing"));
    if (!parseFloat(a1, &f) || f <= 0.0f) return err(F("usage: CALIB <grams> (TARE first, then put the mass on)"));
    double counts = scale.get_value(10);
    if (counts == 0.0) return err(F("no change from tare; is the mass on the load cell?"));
    scaleFactor = (float)(counts / f);
    scale.set_scale(scaleFactor);
    saveScaleFactor();
    Serial.print(F("OK CALIB scale="));
    Serial.println(scaleFactor, 3);
  } else if (!strcasecmp(cmd, "SCALE")) {
    if (!parseFloat(a1, &f) || f == 0.0f) return err(F("usage: SCALE <counts_per_gram>"));
    scaleFactor = f;
    scale.set_scale(scaleFactor);
    saveScaleFactor();
    ok(F("SCALE"));
  } else if (!strcasecmp(cmd, "RATE")) {
    if (!parseInt(a1, 0, 100, &v)) return err(F("usage: RATE <0..100>"));
    streamHz = (uint16_t)v;
    ok(F("RATE"));
  } else if (!strcasecmp(cmd, "TIMEOUT")) {
    if (!parseInt(a1, 0, (long)MAX_DURATION_MS, &v)) return err(F("usage: TIMEOUT <ms>"));
    timeoutMs = (unsigned long)v;
    ok(F("TIMEOUT"));
  } else {
    err(F("unknown command; send HELP"));
  }
}

static void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---------------------------------------------------------------- main

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_PUMP_R_EN, OUTPUT);
  pinMode(PIN_PUMP_L_EN, OUTPUT);
  pinMode(PIN_PUMP_RPWM, OUTPUT);
  pinMode(PIN_PUMP_LPWM, OUTPUT);
  pumpPwmInit();
  applyPump(0);
  digitalWrite(PIN_PUMP_R_EN, HIGH);
  digitalWrite(PIN_PUMP_L_EN, HIGH);

  for (uint8_t i = 0; i < 2; i++) {
    pinMode(act[i].en, OUTPUT);
    pinMode(act[i].in1, OUTPUT);
    pinMode(act[i].in2, OUTPUT);
  }
  pinMode(PIN_STATUS_LED, OUTPUT);
#if USE_ESTOP
  pinMode(PIN_ESTOP, INPUT_PULLUP);
#endif
  applyOutputs();

  loadScaleFactor();
  scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
  scaleOk = scale.wait_ready_timeout(1000);
  if (scaleOk) {
    scale.set_scale(scaleFactor);
    scale.tare(10);
  }

  printInfo();
  if (!scaleOk) Serial.println(F("# hx711 not found; force column will be empty"));
  lastCmdMs = millis();
  lastTravelMs = millis();
}

void loop() {
  updateTravel(millis());  // credit the outputs driven since the last loop

  readSerial();
  readEstop();
  if (mode == MODE_MANUAL && !estop) updateManual();

  // Read the clock after handling commands, so it is never behind a start time
  // (or lastCmdMs) they just set; otherwise now - start would wrap around.
  unsigned long now = millis();
  if (expired(pump, now)) Serial.println(F("# done PUMP"));
  for (uint8_t i = 0; i < 2; i++) {
    if (expired(act[i].out, now)) {
      Serial.print(F("# done ACT "));
      Serial.println(act[i].name);
    }
  }

  if (mode == MODE_SERIAL && timeoutMs > 0 && now - lastCmdMs > timeoutMs &&
      (pump.cmd != 0 || act[0].out.cmd != 0 || act[1].out.cmd != 0)) {
    stopAll();
    Serial.println(F("# command timeout: all outputs off"));
  }

  enforceLimits();
  if (marking && now - markStart >= MARK_MS) marking = false;
  applyOutputs();
  updateForce();

  if (streamHz > 0 && (streamNow || now - lastStreamMs >= 1000UL / streamHz)) {
    streamNow = false;
    lastStreamMs = now;
    printTelemetry(now);
  }
}
