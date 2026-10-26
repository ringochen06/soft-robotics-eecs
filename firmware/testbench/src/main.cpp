// Test bench firmware for the water-jet ureteroscopy robot (Model 1.1).
//
// One Arduino drives the jet pump (BTS7960), the two steering actuators
// (L298N) and reads the load cell (HX711). It takes line-based commands over
// USB serial and streams timestamped CSV telemetry. See ../README.md for the
// command list and data format.

#include <Arduino.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <string.h>

#include "HX711.h"
#include "config.h"

static const char FW_NAME[] = "fams-sr-testbench";
static const char FW_VERSION[] = "0.1.0";

static const uint16_t EEPROM_MAGIC = 0xFA55;
static const int EEPROM_ADDR_MAGIC = 0;
static const int EEPROM_ADDR_SCALE = 2;

enum Mode { MODE_SERIAL, MODE_MANUAL };

static Mode mode = MODE_SERIAL;
static int pumpCmd = 0;  // -255..255, positive = forward (jet)
static int actACmd = 0;  // -255..255, positive = extend
static int actBCmd = 0;
static bool estop = false;
static bool manualArmed = false;

static uint16_t streamHz = DEFAULT_STREAM_HZ;
static unsigned long lastStreamMs = 0;
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

static void driveActuator(uint8_t en, uint8_t in1, uint8_t in2, int cmd, bool reversed) {
  cmd = constrain(cmd, -255, 255);
  if (reversed) cmd = -cmd;
  if (cmd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, cmd);
  } else if (cmd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -cmd);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  }
}

static void applyOutputs() {
  bool off = estop || (mode == MODE_MANUAL && !manualArmed);
  applyPump(off ? 0 : pumpCmd);
  driveActuator(PIN_ACT_A_EN, PIN_ACT_A_IN1, PIN_ACT_A_IN2, off ? 0 : actACmd, ACT_A_REVERSED);
  driveActuator(PIN_ACT_B_EN, PIN_ACT_B_IN1, PIN_ACT_B_IN2, off ? 0 : actBCmd, ACT_B_REVERSED);
  digitalWrite(LED_BUILTIN, estop ? HIGH : LOW);
}

static void stopAll() {
  pumpCmd = 0;
  actACmd = 0;
  actBCmd = 0;
}

// ---------------------------------------------------------------- inputs

static int joystickSpeed(int raw) {
  int dx = raw - 512;
  if (abs(dx) < JOY_DEADBAND) return 0;
  int mag = map(abs(dx) - JOY_DEADBAND, 0, 512 - JOY_DEADBAND, 0, 255);
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
  pumpCmd = map(pot, 0, 1023, 0, 255);
  actACmd = steer;
  actBCmd = -steer;
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
  Serial.println(F("H,t_ms,mode,pump,act_a,act_b,force_g,an1,an2,pot,joy,estop"));
}

static void printInfo() {
  Serial.print(F("I,"));
  Serial.print(FW_NAME);
  Serial.print(',');
  Serial.print(FW_VERSION);
  Serial.print(F(",scale="));
  Serial.print(scaleFactor, 3);
  Serial.print(F(",hx711="));
  Serial.println(scaleOk ? F("ok") : F("missing"));
  printHeader();
}

static void printTelemetry(unsigned long now) {
  bool off = estop || (mode == MODE_MANUAL && !manualArmed);
  Serial.print(F("D,"));
  Serial.print(now);
  Serial.print(',');
  Serial.print(mode == MODE_SERIAL ? 'S' : 'M');
  Serial.print(',');
  Serial.print(off ? 0 : pumpCmd);
  Serial.print(',');
  Serial.print(off ? 0 : actACmd);
  Serial.print(',');
  Serial.print(off ? 0 : actBCmd);
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
  Serial.println(estop ? 1 : 0);
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

static void printHelp() {
  Serial.println(F("# PING | INFO | HELP"));
  Serial.println(F("# MODE SERIAL|MANUAL"));
  Serial.println(F("# PUMP <-255..255>        jet pump PWM (negative = reverse)"));
  Serial.println(F("# ACT A|B <-255..255>     one actuator (positive = extend)"));
  Serial.println(F("# STEER <-255..255>       A extends and B retracts (negative = opposite)"));
  Serial.println(F("# STOP                    everything off"));
  Serial.println(F("# TARE | CALIB <grams> | SCALE <counts_per_gram>"));
  Serial.println(F("# RATE <0..100>           telemetry Hz (0 = off)"));
  Serial.println(F("# TIMEOUT <ms>            stop if no command for this long (0 = off)"));
}

static void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (cmd == NULL) return;
  char *a1 = strtok(NULL, " \t");
  char *a2 = strtok(NULL, " \t");
  long v;
  float f;

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
    if (!parseInt(a1, -255, 255, &v)) return err(F("usage: PUMP <-255..255>"));
    if (!canDrive()) return;
    pumpCmd = (int)v;
    ok(F("PUMP"));
  } else if (!strcasecmp(cmd, "ACT")) {
    if (!a1 || !parseInt(a2, -255, 255, &v)) return err(F("usage: ACT A|B <-255..255>"));
    if (!canDrive()) return;
    if (!strcasecmp(a1, "A")) {
      actACmd = (int)v;
    } else if (!strcasecmp(a1, "B")) {
      actBCmd = (int)v;
    } else {
      return err(F("usage: ACT A|B <-255..255>"));
    }
    ok(F("ACT"));
  } else if (!strcasecmp(cmd, "STEER")) {
    if (!parseInt(a1, -255, 255, &v)) return err(F("usage: STEER <-255..255>"));
    if (!canDrive()) return;
    actACmd = (int)v;
    actBCmd = (int)-v;
    ok(F("STEER"));
  } else if (!strcasecmp(cmd, "STOP")) {
    stopAll();
    ok(F("STOP"));
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
    if (!parseInt(a1, 0, 600000L, &v)) return err(F("usage: TIMEOUT <ms>"));
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

  pinMode(PIN_ACT_A_EN, OUTPUT);
  pinMode(PIN_ACT_A_IN1, OUTPUT);
  pinMode(PIN_ACT_A_IN2, OUTPUT);
  pinMode(PIN_ACT_B_EN, OUTPUT);
  pinMode(PIN_ACT_B_IN1, OUTPUT);
  pinMode(PIN_ACT_B_IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
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
}

void loop() {
  unsigned long now = millis();

  readSerial();
  readEstop();
  if (mode == MODE_MANUAL && !estop) updateManual();

  if (mode == MODE_SERIAL && timeoutMs > 0 && now - lastCmdMs > timeoutMs &&
      (pumpCmd != 0 || actACmd != 0 || actBCmd != 0)) {
    stopAll();
    Serial.println(F("# command timeout: all outputs off"));
  }

  applyOutputs();
  updateForce();

  if (streamHz > 0 && now - lastStreamMs >= 1000UL / streamHz) {
    lastStreamMs = now;
    printTelemetry(now);
  }
}
