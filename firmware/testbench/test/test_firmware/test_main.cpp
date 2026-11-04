// Runs src/main.cpp on this computer against the fake Arduino in test/shim.
//   pio test -e native            (L298N wiring)
//   pio test -e native_pwm_dir    (PWM + DIR driver wiring)
// The clock only moves when a test advances it; run(ms) calls loop() once per ms.

#include <Arduino.h>
#include <EEPROM.h>
#include <HX711.h>
#include <unity.h>

#include <sstream>
#include <vector>

#include "config.h"

void setup();
void loop();

// ---------------------------------------------------------------- helpers

static void run(unsigned long ms) {
  for (unsigned long i = 0; i < ms; i++) {
    sim::now_ms++;
    loop();
  }
}

static std::string send(const char *line) {
  sim::tx.clear();
  sim::rx += line;
  sim::rx += "\n";
  loop();
  return sim::tx;
}

static bool has(const std::string &text, const char *part) { return text.find(part) != std::string::npos; }

static int count(const std::string &text, const char *part) {
  int n = 0;
  for (size_t at = text.find(part); at != std::string::npos; at = text.find(part, at + 1)) n++;
  return n;
}

static std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) out.push_back(item);
  return out;
}

static std::vector<std::string> header;

// Field `name` of the last telemetry line printed since tx was cleared.
static std::string sample(const char *name) {
  size_t at = sim::tx.rfind("D,");
  TEST_ASSERT_TRUE_MESSAGE(at != std::string::npos, "no telemetry line");
  std::string line = sim::tx.substr(at, sim::tx.find('\r', at) - at);
  std::vector<std::string> fields = split(line);
  for (size_t i = 1; i < header.size(); i++) {
    if (header[i] == name) return i < fields.size() ? fields[i] : std::string();
  }
  TEST_FAIL_MESSAGE("no such column");
  return "";
}

static long sampleLong(const char *name) { return atol(sample(name).c_str()); }

// Power-up state for every test. Firmware statics persist between tests, so
// commands put everything back to the defaults; the clock keeps running.
static void boot() {
  sim::reset_io();
  sim::hx711_present = true;
  sim::hx711_grams = 0.0;
  setup();
  send("MODE SERIAL");
  send("STOP");
  send("TIMEOUT 0");
  send("LIMIT 2000");
  send("ZERO");
  send("RATE 0");
  run(300);  // let any MARK pulse from a previous test finish
  sim::tx.clear();
}

void setUp() { boot(); }
void tearDown() {}

// ---------------------------------------------------------------- basics

void test_ping_and_info() {
  TEST_ASSERT_TRUE(has(send("PING"), "OK PONG"));
  std::string info = send("INFO");
  TEST_ASSERT_TRUE(has(info, "I,fams-sr-testbench,0.2.1,scale=455.000,hx711=ok,limit_ms=2000,driver="));
  size_t h = info.find("H,");
  TEST_ASSERT_TRUE(h != std::string::npos);
  header = split(info.substr(h, info.find('\r', h) - h));
  TEST_ASSERT_EQUAL_STRING("pos_a", header[6].c_str());
  TEST_ASSERT_EQUAL_STRING("mark", header.back().c_str());
}

void test_bad_commands_are_rejected() {
  TEST_ASSERT_TRUE(has(send("PUMP 300"), "ERR usage: PUMP"));
  TEST_ASSERT_TRUE(has(send("PUMP 10 abc"), "ERR usage: PUMP"));
  TEST_ASSERT_TRUE(has(send("STEER 10 0"), "ERR usage: STEER"));
  TEST_ASSERT_TRUE(has(send("ACT C 10"), "ERR usage: ACT"));
  TEST_ASSERT_TRUE(has(send("ACT A"), "ERR usage: ACT"));
  TEST_ASSERT_TRUE(has(send("LIMIT -5"), "ERR usage: LIMIT"));
  TEST_ASSERT_TRUE(has(send("FOO"), "ERR unknown command"));
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
}

void test_commands_are_case_insensitive() {
  TEST_ASSERT_TRUE(has(send("pump 40"), "OK PUMP"));
  TEST_ASSERT_EQUAL(40, sim::pwm_out[PIN_PUMP_RPWM]);
}

// ---------------------------------------------------------------- pump

void test_pump_forward_and_reverse() {
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_PUMP_R_EN]);
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_PUMP_L_EN]);
  send("PUMP 120");
  TEST_ASSERT_EQUAL(120, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_LPWM]);
  send("PUMP -80");
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_EQUAL(80, sim::pwm_out[PIN_PUMP_LPWM]);
  send("STOP");
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_LPWM]);
}

void test_timed_pump_stops_by_itself() {
  TEST_ASSERT_TRUE(has(send("PUMP 100 1000"), "OK PUMP"));
  run(999);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_PUMP_RPWM]);
  run(2);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_TRUE(has(sim::tx, "# done PUMP"));
}

// ---------------------------------------------------------------- actuators

#if ACT_DRIVER == ACT_DRIVER_EN_IN
void test_actuator_pins_l298n() {
  send("ACT A 150");
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_ACT_A_IN1]);
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_ACT_A_IN2]);
  TEST_ASSERT_EQUAL(150, sim::pwm_out[PIN_ACT_A_EN]);
  send("ACT A -100");
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_ACT_A_IN1]);
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_ACT_A_IN2]);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_ACT_A_EN]);
  send("STEER 90");
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_ACT_A_IN1]);
  TEST_ASSERT_EQUAL(90, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_ACT_B_IN1]);
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_ACT_B_IN2]);
  TEST_ASSERT_EQUAL(90, sim::pwm_out[PIN_ACT_B_EN]);
  send("STOP");
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_ACT_A_IN1]);
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_ACT_A_IN2]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_B_EN]);
}
#else
void test_actuator_pins_pwm_dir() {
  send("ACT A 150");
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_ACT_A_IN1]);
  TEST_ASSERT_EQUAL(150, sim::pwm_out[PIN_ACT_A_EN]);
  send("ACT A -100");
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_ACT_A_IN1]);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_ACT_A_EN]);
  send("STEER 90");
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_ACT_A_IN1]);
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_ACT_B_IN1]);
  TEST_ASSERT_EQUAL(90, sim::pwm_out[PIN_ACT_B_EN]);
  send("STOP");
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_B_EN]);
}
#endif

void test_timed_steer_stops_both_actuators() {
  TEST_ASSERT_TRUE(has(send("STEER 150 500"), "OK STEER"));
  run(499);
  TEST_ASSERT_EQUAL(150, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(150, sim::pwm_out[PIN_ACT_B_EN]);
  run(2);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_B_EN]);
  TEST_ASSERT_TRUE(has(sim::tx, "# done ACT A"));
  TEST_ASSERT_TRUE(has(sim::tx, "# done ACT B"));
}

void test_new_command_replaces_a_timed_one() {
  send("ACT A 100 300");
  send("ACT A 50");
  run(1000);
  TEST_ASSERT_EQUAL(50, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_FALSE(has(sim::tx, "# done"));
}

void test_timed_move_across_millis_rollover() {
  sim::now_ms = 0xFFFFFFFFUL - 100;
  send("ZERO");
  send("STEER 100 500");
  run(499);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_ACT_A_EN]);
  run(2);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
}

// The clock can tick while a command is handled. Before 0.2.0 this made
// now - lastCmdMs wrap around, so TIMEOUT stopped everything at once.
void test_clock_ticking_during_commands() {
  send("TIMEOUT 1000");
  sim::tick_per_call = true;
  send("PUMP 100");
  send("STEER 80 2000");
  for (int i = 0; i < 20; i++) loop();
  sim::tick_per_call = false;
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_EQUAL(80, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_FALSE(has(sim::tx, "timeout"));
  TEST_ASSERT_FALSE(has(sim::tx, "# done"));
}

// ---------------------------------------------------------------- travel estimate and limit

void test_travel_estimate() {
  send("RATE 100");
  send("ACT A 255 1000");
  run(1100);
  TEST_ASSERT_INT_WITHIN(2, 1000, sampleLong("pos_a"));
  TEST_ASSERT_EQUAL(0, sampleLong("pos_b"));
  send("ACT A -128 1000");  // half duty for 1 s = 0.5 s at full speed
  run(1100);
  TEST_ASSERT_INT_WITHIN(3, 498, sampleLong("pos_a"));
  send("STEER 255 400");
  run(500);
  TEST_ASSERT_INT_WITHIN(3, 898, sampleLong("pos_a"));
  TEST_ASSERT_INT_WITHIN(2, -400, sampleLong("pos_b"));
  send("ZERO");
  run(20);
  TEST_ASSERT_EQUAL(0, sampleLong("pos_a"));
  TEST_ASSERT_EQUAL(0, sampleLong("pos_b"));
}

void test_travel_limit_stops_then_allows_moving_back() {
  send("LIMIT 500");
  send("ACT A 255");
  run(498);
  TEST_ASSERT_EQUAL(255, sim::pwm_out[PIN_ACT_A_EN]);
  run(5);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(1, count(sim::tx, "act A reached its travel limit"));
  run(200);
  TEST_ASSERT_EQUAL(1, count(sim::tx, "travel limit"));  // printed once
  TEST_ASSERT_TRUE(has(send("ACT A 100"), "ERR act A is at its travel limit"));
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_TRUE(has(send("ACT A -100"), "OK ACT"));
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_ACT_A_EN]);
}

void test_steer_stops_at_limit_both_ways() {
  send("LIMIT 300");
  send("STEER 255");
  run(310);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_B_EN]);
  TEST_ASSERT_TRUE(has(send("STEER 100"), "ERR act A"));
  TEST_ASSERT_TRUE(has(send("STEER -100"), "OK STEER"));
  run(10);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_ACT_B_EN]);
}

void test_timed_move_cut_short_by_limit() {
  send("LIMIT 200");
  send("ACT B -255 1000");
  run(210);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_B_EN]);
  run(1000);
  TEST_ASSERT_FALSE(has(sim::tx, "# done ACT B"));  // it was stopped by the limit instead
}

void test_zero_and_limit_0_release_the_limit() {
  send("LIMIT 100");
  send("ACT A 255");
  run(120);
  TEST_ASSERT_TRUE(has(send("ACT A 255"), "ERR"));
  send("ZERO");
  TEST_ASSERT_TRUE(has(send("ACT A 255"), "OK ACT"));
  TEST_ASSERT_TRUE(has(send("LIMIT 0"), "OK LIMIT 0"));
  run(5000);
  TEST_ASSERT_EQUAL(255, sim::pwm_out[PIN_ACT_A_EN]);
}

// ---------------------------------------------------------------- safety

void test_estop_stops_everything_and_blocks_commands() {
  send("PUMP 100");
  send("STEER 120");
  sim::digital_in[PIN_ESTOP] = LOW;
  loop();
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_B_EN]);
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_STATUS_LED]);
  TEST_ASSERT_TRUE(has(sim::tx, "# estop pressed"));
  TEST_ASSERT_TRUE(has(send("PUMP 50"), "ERR estop is pressed"));
  sim::digital_in[PIN_ESTOP] = HIGH;
  run(5);
  TEST_ASSERT_TRUE(has(sim::tx, "# estop released"));
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);  // stays off until a new command
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_STATUS_LED]);
  TEST_ASSERT_TRUE(has(send("PUMP 50"), "OK PUMP"));
  TEST_ASSERT_EQUAL(50, sim::pwm_out[PIN_PUMP_RPWM]);
}

void test_command_timeout() {
  send("TIMEOUT 200");
  send("PUMP 100");
  run(150);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_PUMP_RPWM]);
  send("PING");  // any command resets the timer
  run(150);
  TEST_ASSERT_EQUAL(100, sim::pwm_out[PIN_PUMP_RPWM]);
  run(60);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_TRUE(has(sim::tx, "# command timeout"));
}

// ---------------------------------------------------------------- manual mode

void test_manual_mode_arms_only_from_zero() {
  sim::analog_in[PIN_POT] = 600;  // knob left turned up
  TEST_ASSERT_TRUE(has(send("MODE MANUAL"), "OK MODE MANUAL"));
  run(10);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_FALSE(has(sim::tx, "armed"));
  sim::analog_in[PIN_POT] = 0;
  run(10);
  TEST_ASSERT_TRUE(has(sim::tx, "# manual mode armed"));
  sim::analog_in[PIN_POT] = 1023;
  sim::analog_in[PIN_JOY_X] = 1023;
  run(10);
  TEST_ASSERT_EQUAL(255, sim::pwm_out[PIN_PUMP_RPWM]);
  TEST_ASSERT_EQUAL(255, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(255, sim::pwm_out[PIN_ACT_B_EN]);
  TEST_ASSERT_TRUE(has(send("PUMP 10"), "ERR in manual mode"));
  sim::analog_in[PIN_JOY_X] = 512 + JOY_DEADBAND - 1;  // inside the deadband
  run(10);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  send("MODE SERIAL");
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_PUMP_RPWM]);
}

void test_manual_steering_respects_the_limit() {
  send("LIMIT 200");
  send("MODE MANUAL");
  run(5);  // arms (knob 0, joystick centered)
  sim::analog_in[PIN_JOY_X] = 1023;
  run(250);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_A_EN]);
  TEST_ASSERT_EQUAL(0, sim::pwm_out[PIN_ACT_B_EN]);
  TEST_ASSERT_EQUAL(1, count(sim::tx, "act A reached"));
  sim::analog_in[PIN_JOY_X] = 0;  // the other way is still allowed
  run(10);
  TEST_ASSERT_EQUAL(255, sim::pwm_out[PIN_ACT_A_EN]);
  send("MODE SERIAL");
}

// ---------------------------------------------------------------- mark, load cell, telemetry

void test_mark_flashes_led_and_sends_a_sample_at_once() {
  send("RATE 1");
  run(1500);
  sim::tx.clear();
  std::string out = send("MARK");
  TEST_ASSERT_TRUE(has(out, "OK MARK"));
  TEST_ASSERT_EQUAL(HIGH, sim::digital_out[PIN_STATUS_LED]);
  TEST_ASSERT_EQUAL_STRING("1", sample("mark").c_str());
  run(MARK_MS + 1);
  TEST_ASSERT_EQUAL(LOW, sim::digital_out[PIN_STATUS_LED]);
}

void test_telemetry_rate() {
  send("RATE 20");
  run(5);
  sim::tx.clear();
  run(1000);
  TEST_ASSERT_INT_WITHIN(1, 20, count(sim::tx, "D,"));
  send("RATE 0");
  sim::tx.clear();
  run(1000);
  TEST_ASSERT_EQUAL(0, count(sim::tx, "D,"));
}

void test_load_cell_tare_and_calib() {
  send("RATE 50");
  sim::hx711_grams = 12.0;
  TEST_ASSERT_TRUE(has(send("TARE"), "OK TARE"));
  sim::hx711_grams = 62.0;  // put a 50 g mass on
  std::string out = send("CALIB 50");
  TEST_ASSERT_TRUE(has(out, "OK CALIB scale=455.000"));
  run(40);
  TEST_ASSERT_EQUAL_STRING("50.00", sample("force_g").c_str());
  sim::hx711_grams = 12.0;
  run(40);
  TEST_ASSERT_EQUAL_STRING("0.00", sample("force_g").c_str());
}

void test_scale_is_saved_in_eeprom() {
  TEST_ASSERT_TRUE(has(send("SCALE 300.5"), "OK SCALE"));
  sim::reset_io();
  setup();  // power cycle
  TEST_ASSERT_TRUE(has(sim::tx, "scale=300.500"));
  send("SCALE 455");
}

void test_missing_load_cell() {
  sim::hx711_present = false;
  sim::reset_io();
  setup();
  TEST_ASSERT_TRUE(has(sim::tx, "hx711=missing"));
  TEST_ASSERT_TRUE(has(sim::tx, "# hx711 not found"));
  TEST_ASSERT_TRUE(has(send("TARE"), "ERR hx711 missing"));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_ping_and_info);  // first: reads the column header
  RUN_TEST(test_bad_commands_are_rejected);
  RUN_TEST(test_commands_are_case_insensitive);
  RUN_TEST(test_pump_forward_and_reverse);
  RUN_TEST(test_timed_pump_stops_by_itself);
#if ACT_DRIVER == ACT_DRIVER_EN_IN
  RUN_TEST(test_actuator_pins_l298n);
#else
  RUN_TEST(test_actuator_pins_pwm_dir);
#endif
  RUN_TEST(test_timed_steer_stops_both_actuators);
  RUN_TEST(test_new_command_replaces_a_timed_one);
  RUN_TEST(test_timed_move_across_millis_rollover);
  RUN_TEST(test_clock_ticking_during_commands);
  RUN_TEST(test_travel_estimate);
  RUN_TEST(test_travel_limit_stops_then_allows_moving_back);
  RUN_TEST(test_steer_stops_at_limit_both_ways);
  RUN_TEST(test_timed_move_cut_short_by_limit);
  RUN_TEST(test_zero_and_limit_0_release_the_limit);
  RUN_TEST(test_estop_stops_everything_and_blocks_commands);
  RUN_TEST(test_command_timeout);
  RUN_TEST(test_manual_mode_arms_only_from_zero);
  RUN_TEST(test_manual_steering_respects_the_limit);
  RUN_TEST(test_mark_flashes_led_and_sends_a_sample_at_once);
  RUN_TEST(test_telemetry_rate);
  RUN_TEST(test_load_cell_tare_and_calib);
  RUN_TEST(test_scale_is_saved_in_eeprom);
  RUN_TEST(test_missing_load_cell);
  return UNITY_END();
}
