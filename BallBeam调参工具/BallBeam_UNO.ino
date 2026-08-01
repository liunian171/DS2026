#include <Arduino.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// USER/TUNING PARAMETERS -- only KP/KI/KD are entered at runtime.
// The fixed execution values below were already used during bounded hardware tests.
// ============================================================================

constexpr float STARTUP_KP = 0.0F;                 // pulse/s per mm
constexpr float STARTUP_KI = 0.0F;                 // pulse/s per (mm*s)
constexpr float STARTUP_KD = 0.0F;                 // pulse/s per (mm/s)
constexpr float TARGET_POSITION_MM = 0.0F;         // beam center
constexpr float STARTUP_DEADBAND_MM = 5.0F;
constexpr float DERIVATIVE_FILTER_ALPHA = 0.20F;   // 0..1

constexpr float STARTUP_MAX_RATE_HZ = 2000.0F;
constexpr float STARTUP_MAX_ACCEL_HZ_S = 8000.0F;
constexpr float STARTUP_RETURN_RATE_HZ = 1000.0F;
constexpr int8_t STARTUP_CONTROL_SIGN = 1;          // +error -> clockwise

constexpr unsigned long STATUS_PERIOD_MS = 250UL;
constexpr long MAX_ABS_VISION_POSITION_MM = 150L;  // parser plausibility guard

// User-confirmed hardware and mechanical limits.
constexpr uint8_t PIN_X42S_DIR = 2U;
constexpr uint8_t PIN_X42S_STEP = 3U;
constexpr uint16_t PULSES_PER_REVOLUTION = 3200U;
constexpr uint8_t DIR_LEVEL_CW = HIGH;              // verified: 1 = clockwise
constexpr uint8_t DIR_LEVEL_CCW = LOW;              // verified: 0 = counterclockwise

// Software guards, not X42S specifications.
constexpr float MAX_ALLOWED_RATE_HZ = 12000.0F;
constexpr float MAX_ALLOWED_ACCEL_HZ_S = 50000.0F;
constexpr float MAX_ALLOWED_GAIN = 10000.0F;
constexpr float MAX_ALLOWED_DEADBAND_MM = 50.0F;
// Software angle limit: the beam is mechanically limited to about +/-30 deg,
// but normal control only needs a few degrees of tilt to drive the ball.
// Motor 1 turn (360 deg) moves the rack end up 5 cm over a 25 cm arm,
// so the ratio is 360 / asin(5/25) = 31.2:1 and 0.1125 deg/pulse means:
//   +/-30 deg of beam = about 8300 pulses (mechanical hard limit),
//   +/-8 deg of beam  = about 2200 pulses (control limit).
// The control limit is kept small so the ball is never launched at high
// speed; the mechanical limit is never reached in normal operation.
constexpr long MAX_ABS_COMMAND_PULSES = 2200L;
// Position-mode servo: the PID output accumulates into a target command
// pulse position, and the pulse generator follows that target with a
// limited rate (servo gain times remaining distance, clamped).
constexpr float SERVO_GAIN = 3.0F;
constexpr float MIN_SERVO_RATE_HZ = 60.0F;
constexpr unsigned long DIR_SETUP_US = 10000UL;
constexpr unsigned long USB_BAUD = 115200UL;

// ============================================================================
// Runtime state
// ============================================================================

namespace {

constexpr size_t LINE_CAPACITY = 80U;
constexpr unsigned long MAX_FRAME_ID = 2147483647UL;

enum class RunMode : uint8_t {
  LOCKED = 0U,
  ARMED = 1U,
  RETURNING = 2U,
};

char line_buffer[LINE_CAPACITY] = {};
size_t line_length = 0U;
bool discarding_overlong_line = false;

float kp = STARTUP_KP;
float ki = STARTUP_KI;
float kd = STARTUP_KD;
float deadband_mm = STARTUP_DEADBAND_MM;
float max_rate_hz = STARTUP_MAX_RATE_HZ;
float max_accel_hz_s = STARTUP_MAX_ACCEL_HZ_S;
float return_rate_hz = STARTUP_RETURN_RATE_HZ;
int8_t control_sign = STARTUP_CONTROL_SIGN;

RunMode run_mode = RunMode::LOCKED;
bool zero_confirmed = true;
bool auto_control_enabled = false;
bool pins_active = false;
bool step_high = false;
int8_t active_step_direction = 0; // +1=CW, -1=CCW
long command_position_pulses = 0L;
unsigned long next_edge_us = 0UL;

unsigned long latest_frame_id = 0UL;
long latest_position_mm = 0L;
uint8_t latest_vision_state = 0U;
unsigned long last_vision_ms = 0UL;
unsigned long last_track_ms = 0UL;
bool have_vision = false;
bool have_track = false;

float integral_error = 0.0F;
float filtered_derivative = 0.0F;
float previous_position_mm = 0.0F;
unsigned long previous_pid_ms = 0UL;
bool have_pid_sample = false;
float requested_rate_hz = 0.0F; // physical sign: +CW, -CCW
float target_command_pulse = 0.0F; // PID output integrated to a target pulse position
float current_rate_hz = 0.0F;
unsigned long previous_ramp_us = 0UL;

unsigned long last_status_ms = 0UL;

const __FlashStringHelper* modeName() {
  switch (run_mode) {
    case RunMode::ARMED:
      return F("ARMED");
    case RunMode::RETURNING:
      return F("RETURNING");
    default:
      return F("LOCKED");
  }
}

float clampFloat(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

float moveToward(float current, float target, float maximum_change) {
  if (current < target) {
    return min(current + maximum_change, target);
  }
  if (current > target) {
    return max(current - maximum_change, target);
  }
  return current;
}

void setPinsHighImpedance() {
  if (step_high) {
    digitalWrite(PIN_X42S_STEP, LOW);
  }
  pinMode(PIN_X42S_STEP, INPUT);
  pinMode(PIN_X42S_DIR, INPUT);
  pins_active = false;
  step_high = false;
  active_step_direction = 0;
}

void resetPidState() {
  integral_error = 0.0F;
  filtered_derivative = 0.0F;
  previous_position_mm = static_cast<float>(latest_position_mm);
  previous_pid_ms = 0UL;
  have_pid_sample = false;
  requested_rate_hz = 0.0F;
  target_command_pulse = static_cast<float>(command_position_pulses);
}

void stopAndInvalidateZero(const __FlashStringHelper* reason) {
  requested_rate_hz = 0.0F;
  current_rate_hz = 0.0F;
  run_mode = RunMode::LOCKED;
  zero_confirmed = false;
  auto_control_enabled = false;
  resetPidState();
  setPinsHighImpedance();
  Serial.print(F("DISARM reason="));
  Serial.println(reason);
}

void completeReturn() {
  requested_rate_hz = 0.0F;
  current_rate_hz = 0.0F;
  run_mode = RunMode::LOCKED;
  resetPidState();
  setPinsHighImpedance();
  Serial.println(F("RETURN complete; output_locked; zero_retained=1"));
}

void beginReturn(const __FlashStringHelper* reason) {
  if (!zero_confirmed) {
    stopAndInvalidateZero(F("return_without_zero"));
    return;
  }

  resetPidState();
  run_mode = RunMode::RETURNING;
  Serial.print(F("RETURN begin reason="));
  Serial.print(reason);
  Serial.print(F(" from_pulse="));
  Serial.println(command_position_pulses);

  if (command_position_pulses == 0L) {
    completeReturn();
  }
}

bool parseLongStrict(const char* text, long& value_out) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  const long value = strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  value_out = value;
  return true;
}

bool parseUnsignedLongStrict(const char* text, unsigned long& value_out) {
  if (text == nullptr || *text == '\0' || *text == '-') {
    return false;
  }
  char* end = nullptr;
  const unsigned long value = strtoul(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  value_out = value;
  return true;
}

bool parseFloatStrict(const char* text, float& value_out) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  const double value = strtod(text, &end);
  if (end == text || *end != '\0' || isnan(value) || isinf(value)) {
    return false;
  }
  value_out = static_cast<float>(value);
  return true;
}

void updatePidFromTrack(unsigned long now_ms) {
  if (run_mode != RunMode::ARMED) {
    return;
  }

  const float measured_position = static_cast<float>(latest_position_mm);
  if (!have_pid_sample) {
    previous_position_mm = measured_position;
    previous_pid_ms = now_ms;
    have_pid_sample = true;
    requested_rate_hz = 0.0F;
    return;
  }

  const unsigned long elapsed_ms = now_ms - previous_pid_ms;
  if (elapsed_ms < 5UL) {
    return;
  }

  float dt_s = static_cast<float>(elapsed_ms) * 0.001F;
  if (dt_s > 0.25F) {
    dt_s = 0.25F;
    integral_error = 0.0F;
    filtered_derivative = 0.0F;
  }

  float error_mm = TARGET_POSITION_MM - measured_position;
  if (fabs(error_mm) <= deadband_mm) {
    error_mm = 0.0F;
  }

  const float derivative_measurement =
      -(measured_position - previous_position_mm) / dt_s;
  filtered_derivative += DERIVATIVE_FILTER_ALPHA *
                         (derivative_measurement - filtered_derivative);

  const float proposed_integral = integral_error + error_mm * dt_s;
  const float p_term = kp * error_mm;
  const float d_term = kd * filtered_derivative;
  const float proposed_output = p_term + ki * proposed_integral + d_term;
  const float saturated_output =
      clampFloat(proposed_output, -max_rate_hz, max_rate_hz);

  const bool saturated_high = proposed_output > max_rate_hz;
  const bool saturated_low = proposed_output < -max_rate_hz;
  const bool drives_further_high = saturated_high && error_mm > 0.0F;
  const bool drives_further_low = saturated_low && error_mm < 0.0F;
  if (!drives_further_high && !drives_further_low) {
    integral_error = proposed_integral;
  }

  float effort_hz = saturated_output;
  if (error_mm == 0.0F && ki == 0.0F) {
    effort_hz = 0.0F;
  }
  // Position-mode control: integrate the PID rate into a target pulse
  // position, clamped to the software angle limit. The pulse generator
  // then follows this target with a limited rate instead of commanding
  // an unbounded rate directly.
  float effort_signed = effort_hz * static_cast<float>(control_sign);
  target_command_pulse += effort_signed * dt_s;
  if (target_command_pulse > MAX_ABS_COMMAND_PULSES) {
    target_command_pulse = MAX_ABS_COMMAND_PULSES;
  }
  if (target_command_pulse < -MAX_ABS_COMMAND_PULSES) {
    target_command_pulse = -MAX_ABS_COMMAND_PULSES;
  }
  requested_rate_hz = effort_signed;

  previous_position_mm = measured_position;
  previous_pid_ms = now_ms;
}

bool parseVisionLine(char* line) {
  char* save = nullptr;
  char* prefix = strtok_r(line, ",", &save);
  char* frame_text = strtok_r(nullptr, ",", &save);
  char* position_text = strtok_r(nullptr, ",", &save);
  char* state_text = strtok_r(nullptr, ",", &save);
  char* extra = strtok_r(nullptr, ",", &save);

  if (prefix == nullptr || strcmp(prefix, "B") != 0 ||
      frame_text == nullptr || position_text == nullptr ||
      state_text == nullptr || extra != nullptr) {
    return false;
  }

  unsigned long frame_id = 0UL;
  long position_mm = 0L;
  long state = 0L;
  if (!parseUnsignedLongStrict(frame_text, frame_id) ||
      !parseLongStrict(position_text, position_mm) ||
      !parseLongStrict(state_text, state) ||
      frame_id > MAX_FRAME_ID ||
      position_mm > MAX_ABS_VISION_POSITION_MM ||
      position_mm < -MAX_ABS_VISION_POSITION_MM || state < 0L ||
      state > 3L) {
    return false;
  }

  const unsigned long now_ms = millis();
  latest_frame_id = frame_id;
  latest_position_mm = position_mm;
  latest_vision_state = static_cast<uint8_t>(state);
  last_vision_ms = now_ms;
  have_vision = true;

  if (latest_vision_state >= 1U) {
    last_track_ms = now_ms;
    have_track = true;
    updatePidFromTrack(now_ms);
  }
  return true;
}

void printStatus() {
  const unsigned long now_ms = millis();
  Serial.print(F("STATUS mode="));
  Serial.print(modeName());
  Serial.print(F(" zero="));
  Serial.print(zero_confirmed ? 1 : 0);
  Serial.print(F(" auto="));
  Serial.print(auto_control_enabled ? 1 : 0);
  Serial.print(F(" frame="));
  Serial.print(latest_frame_id);
  Serial.print(F(" pos_mm="));
  Serial.print(latest_position_mm);
  Serial.print(F(" vision_state="));
  Serial.print(latest_vision_state);
  Serial.print(F(" track_age_ms="));
  Serial.print(have_track ? now_ms - last_track_ms : 0UL);
  Serial.print(F(" req_hz="));
  Serial.print(requested_rate_hz, 2);
  Serial.print(F(" current_hz="));
  Serial.print(current_rate_hz, 2);
  Serial.print(F(" pulse="));
  Serial.print(command_position_pulses);
  Serial.print(F(" target="));
  Serial.print(target_command_pulse, 0);
  Serial.print(F(" kp="));
  Serial.print(kp, 4);
  Serial.print(F(" ki="));
  Serial.print(ki, 4);
  Serial.print(F(" kd="));
  Serial.print(kd, 4);
  Serial.print(F(" sign="));
  Serial.print(static_cast<int>(control_sign));
  Serial.print(F(" rate="));
  Serial.print(max_rate_hz, 1);
  Serial.print(F(" accel="));
  Serial.print(max_accel_hz_s, 1);
  Serial.print(F(" return_rate="));
  Serial.println(return_rate_hz, 1);
}

void printHelp() {
  Serial.println(F("PID <kp> <ki> <kd> (enables automatic closed loop)"));
  Serial.println(F("RETURN | DISARM | STATUS | HELP"));
  Serial.println(F("Advanced: ZERO | ARM | KP/KI/KD | SIGN | RATE/ACCEL/RETURNRATE"));
  Serial.println(F("Vision input: B,<frame_id>,<position_mm>,<state>"));
}

void setFloatParameter(const char* name, const char* value_text) {
  float value = 0.0F;
  if (!parseFloatStrict(value_text, value)) {
    Serial.println(F("ERR invalid_number"));
    return;
  }

  if (strcmp(name, "KP") == 0 || strcmp(name, "KI") == 0 ||
      strcmp(name, "KD") == 0) {
    if (value < 0.0F || value > MAX_ALLOWED_GAIN) {
      Serial.println(F("ERR gain_range=0..10000"));
      return;
    }
    if (strcmp(name, "KP") == 0) {
      kp = value;
    } else if (strcmp(name, "KI") == 0) {
      ki = value;
    } else {
      kd = value;
    }
    resetPidState();
  } else if (strcmp(name, "RATE") == 0) {
    if (value < 0.0F || value > MAX_ALLOWED_RATE_HZ) {
      Serial.println(F("ERR rate_range=0..1000"));
      return;
    }
    max_rate_hz = value;
  } else if (strcmp(name, "ACCEL") == 0) {
    if (value < 0.0F || value > MAX_ALLOWED_ACCEL_HZ_S) {
      Serial.println(F("ERR accel_range=0..10000"));
      return;
    }
    max_accel_hz_s = value;
  } else if (strcmp(name, "RETURNRATE") == 0) {
    if (value < 0.0F || value > MAX_ALLOWED_RATE_HZ) {
      Serial.println(F("ERR return_rate_range=0..1000"));
      return;
    }
    return_rate_hz = value;
  } else if (strcmp(name, "DEADBAND") == 0) {
    if (value < 0.0F || value > MAX_ALLOWED_DEADBAND_MM) {
      Serial.println(F("ERR deadband_range=0..50"));
      return;
    }
    deadband_mm = value;
  } else {
    Serial.println(F("ERR unknown_parameter"));
    return;
  }

  Serial.print(F("OK "));
  Serial.print(name);
  Serial.print('=');
  Serial.println(value, 4);
}

void executeCommand(char* line) {
  char* save = nullptr;
  char* command = strtok_r(line, " \t", &save);
  char* value_1 = strtok_r(nullptr, " \t", &save);
  char* value_2 = strtok_r(nullptr, " \t", &save);
  char* value_3 = strtok_r(nullptr, " \t", &save);
  char* extra = strtok_r(nullptr, " \t", &save);
  if (command == nullptr) {
    return;
  }

  for (char* cursor = command; *cursor != '\0'; ++cursor) {
    if (*cursor >= 'a' && *cursor <= 'z') {
      *cursor = static_cast<char>(*cursor - 'a' + 'A');
    }
  }

  if (strcmp(command, "PID") == 0 && value_1 != nullptr &&
      value_2 != nullptr && value_3 != nullptr && extra == nullptr) {
    float new_kp = 0.0F;
    float new_ki = 0.0F;
    float new_kd = 0.0F;
    if (!parseFloatStrict(value_1, new_kp) ||
        !parseFloatStrict(value_2, new_ki) ||
        !parseFloatStrict(value_3, new_kd)) {
      Serial.println(F("ERR PID invalid_number"));
    } else if (new_kp <= 0.0F || new_kp > MAX_ALLOWED_GAIN ||
               new_ki < 0.0F || new_ki > MAX_ALLOWED_GAIN ||
               new_kd < 0.0F || new_kd > MAX_ALLOWED_GAIN) {
      Serial.println(F("ERR PID expects kp>0 and ki/kd>=0, max=10000"));
    } else if (!zero_confirmed) {
      Serial.println(F("ERR PID requires_manual_level_and_restart"));
    } else {
      kp = new_kp;
      ki = new_ki;
      kd = new_kd;
      auto_control_enabled = true;
      resetPidState();
      Serial.print(F("OK PID kp="));
      Serial.print(kp, 4);
      Serial.print(F(" ki="));
      Serial.print(ki, 4);
      Serial.print(F(" kd="));
      Serial.println(kd, 4);
      Serial.println(F("AUTO enabled; waiting for recent vision"));
    }
    return;
  }

  if (strcmp(command, "ZERO") == 0 && value_1 == nullptr) {
    if (run_mode != RunMode::LOCKED || step_high) {
      Serial.println(F("ERR ZERO requires_locked"));
      return;
    }
    command_position_pulses = 0L;
    target_command_pulse = 0.0F;
    zero_confirmed = true;
    resetPidState();
    Serial.println(F("OK ZERO confirmed; beam_must_be_manually_level"));
    return;
  }

  if (strcmp(command, "SIGN") == 0 && value_1 != nullptr &&
      value_2 == nullptr) {
    long value = 0L;
    if (run_mode != RunMode::LOCKED) {
      Serial.println(F("ERR SIGN requires_locked"));
    } else if (!parseLongStrict(value_1, value) || (value != 1L && value != -1L)) {
      Serial.println(F("ERR SIGN expects 1|-1"));
    } else {
      control_sign = static_cast<int8_t>(value);
      Serial.print(F("OK SIGN="));
      Serial.println(static_cast<int>(control_sign));
    }
    return;
  }

  if ((strcmp(command, "KP") == 0 || strcmp(command, "KI") == 0 ||
       strcmp(command, "KD") == 0 || strcmp(command, "RATE") == 0 ||
       strcmp(command, "ACCEL") == 0 ||
       strcmp(command, "RETURNRATE") == 0 ||
       strcmp(command, "DEADBAND") == 0) &&
      value_1 != nullptr && value_2 == nullptr) {
    setFloatParameter(command, value_1);
    return;
  }

  if (strcmp(command, "ARM") == 0 && value_1 == nullptr) {
    if (!zero_confirmed) {
      Serial.println(F("ERR ARM requires_ZERO"));
    } else if (control_sign == 0) {
      Serial.println(F("ERR ARM requires_SIGN"));
    } else if (kp <= 0.0F) {
      Serial.println(F("ERR ARM requires_KP_gt_0"));
    } else if (max_rate_hz <= 0.0F || max_accel_hz_s <= 0.0F ||
               return_rate_hz <= 0.0F) {
      Serial.println(F("ERR ARM requires_RATE_ACCEL_RETURNRATE"));
    } else if (!have_track || latest_vision_state < 1U) {
      Serial.println(F("ERR ARM requires_recent_vision"));
    } else {
      resetPidState();
      previous_ramp_us = micros();
      run_mode = RunMode::ARMED;
      auto_control_enabled = true;
      Serial.println(F("ARMED; PID waits for next vision frame"));
    }
    return;
  }

  if ((strcmp(command, "DISARM") == 0 || strcmp(command, "STOP") == 0) &&
      value_1 == nullptr) {
    stopAndInvalidateZero(F("user"));
    return;
  }

  if (strcmp(command, "RETURN") == 0 && value_1 == nullptr) {
    auto_control_enabled = false;
    if (return_rate_hz <= 0.0F || max_accel_hz_s <= 0.0F) {
      Serial.println(F("ERR RETURN requires_RETURNRATE_and_ACCEL"));
    } else {
      beginReturn(F("user"));
    }
    return;
  }

  if (strcmp(command, "STATUS") == 0 && value_1 == nullptr) {
    printStatus();
    return;
  }

  if (strcmp(command, "HELP") == 0 && value_1 == nullptr) {
    printHelp();
    return;
  }

  Serial.println(F("ERR unknown_or_malformed_command"));
}

void processLine(char* line) {
  if (line[0] == 'B' && line[1] == ',') {
    if (!parseVisionLine(line)) {
      Serial.println(F("ERR invalid_vision_frame"));
    }
    return;
  }
  executeCommand(line);
}

void readSerialLines() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r' || incoming == '\n') {
      if (discarding_overlong_line) {
        discarding_overlong_line = false;
        line_length = 0U;
        continue;
      }
      if (line_length > 0U) {
        line_buffer[line_length] = '\0';
        processLine(line_buffer);
        line_length = 0U;
      }
      continue;
    }

    if (discarding_overlong_line) {
      continue;
    }

    if (line_length < LINE_CAPACITY - 1U) {
      line_buffer[line_length++] = incoming;
    } else {
      line_length = 0U;
      discarding_overlong_line = true;
      Serial.println(F("ERR line_too_long"));
    }
  }
}

float returnTargetRate() {
  if (command_position_pulses > 0L) {
    return -return_rate_hz;
  }
  if (command_position_pulses < 0L) {
    return return_rate_hz;
  }
  return 0.0F;
}

void updateRateRamp() {
  const unsigned long now_us = micros();
  if (previous_ramp_us == 0UL) {
    previous_ramp_us = now_us;
    return;
  }

  unsigned long elapsed_us = now_us - previous_ramp_us;
  if (elapsed_us < 1000UL) {
    return;
  }
  if (elapsed_us > 50000UL) {
    elapsed_us = 50000UL;
  }
  previous_ramp_us = now_us;

  float target_rate = 0.0F;
  if (run_mode == RunMode::ARMED) {
    // Position-mode servo: rate proportional to remaining distance to the
    // PID target, clamped to the configured max rate, with a minimum rate
    // so the beam always keeps moving toward the target.
    const float distance = target_command_pulse - command_position_pulses;
    if (fabs(distance) > 0.5F) {
      float servo_rate = distance * SERVO_GAIN;
      const float rate_limit = fabs(servo_rate) < MIN_SERVO_RATE_HZ
                                   ? MIN_SERVO_RATE_HZ
                                   : max_rate_hz;
      servo_rate = clampFloat(servo_rate, -rate_limit, rate_limit);
      target_rate = servo_rate;
    }
  } else if (run_mode == RunMode::RETURNING) {
    target_rate = returnTargetRate();
  }

  if (max_accel_hz_s <= 0.0F) {
    current_rate_hz = 0.0F;
    return;
  }
  const float maximum_change =
      max_accel_hz_s * static_cast<float>(elapsed_us) * 0.000001F;
  current_rate_hz = moveToward(current_rate_hz, target_rate, maximum_change);
}

bool stepAllowed(int8_t direction) {
  if (run_mode == RunMode::RETURNING &&
      ((command_position_pulses > 0L && direction > 0) ||
       (command_position_pulses < 0L && direction < 0) ||
       command_position_pulses == 0L)) {
    return false;
  }
  // Software angle limit: never drive the beam past its mechanical range.
  if (command_position_pulses >= MAX_ABS_COMMAND_PULSES && direction > 0) {
    return false;
  }
  if (command_position_pulses <= -MAX_ABS_COMMAND_PULSES && direction < 0) {
    return false;
  }
  return true;
}

void activatePinsForDirection(int8_t direction, unsigned long now_us) {
  digitalWrite(PIN_X42S_STEP, LOW);
  digitalWrite(PIN_X42S_DIR,
               direction > 0 ? DIR_LEVEL_CW : DIR_LEVEL_CCW);
  pinMode(PIN_X42S_DIR, OUTPUT);
  pinMode(PIN_X42S_STEP, OUTPUT);
  pins_active = true;
  step_high = false;
  active_step_direction = direction;
  next_edge_us = now_us + DIR_SETUP_US;
}

void updatePulseGenerator() {
  const unsigned long now_us = micros();

  if (step_high) {
    if (static_cast<long>(now_us - next_edge_us) >= 0L) {
      digitalWrite(PIN_X42S_STEP, LOW);
      step_high = false;
      const float absolute_rate = max(fabs(current_rate_hz), 1.0F);
      next_edge_us = now_us +
                     static_cast<unsigned long>(500000.0F / absolute_rate);
    }
    return;
  }

  int8_t desired_direction = 0;
  if (current_rate_hz >= 1.0F) {
    desired_direction = 1;
  } else if (current_rate_hz <= -1.0F) {
    desired_direction = -1;
  }

  if (desired_direction == 0 || run_mode == RunMode::LOCKED) {
    return;
  }

  if (!pins_active || active_step_direction != desired_direction) {
    activatePinsForDirection(desired_direction, now_us);
    return;
  }

  if (static_cast<long>(now_us - next_edge_us) < 0L) {
    return;
  }
  if (!stepAllowed(desired_direction)) {
    return;
  }

  digitalWrite(PIN_X42S_STEP, HIGH);
  step_high = true;
  command_position_pulses += desired_direction;
  const float absolute_rate = max(fabs(current_rate_hz), 1.0F);
  next_edge_us = now_us +
                 static_cast<unsigned long>(500000.0F / absolute_rate);
}

void serviceSafetyState() {
  if (run_mode == RunMode::RETURNING && command_position_pulses == 0L &&
      !step_high) {
    completeReturn();
  }
}

void autoArmWhenReady() {
  if (!auto_control_enabled || run_mode != RunMode::LOCKED ||
      !zero_confirmed || kp <= 0.0F) {
    return;
  }

  if (!have_track || latest_vision_state < 1U) {
    return;
  }

  resetPidState();
  previous_ramp_us = micros();
  run_mode = RunMode::ARMED;
  Serial.println(F("AUTO ARMED; continuous center control active"));
}

void printPeriodicStatus() {
  const unsigned long now_ms = millis();
  if (now_ms - last_status_ms >= STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    printStatus();
  }
}

} // namespace

void setup() {
  setPinsHighImpedance();
  command_position_pulses = 0L;
  zero_confirmed = true;
  auto_control_enabled = false;
  Serial.begin(USB_BAUD);
  Serial.println(F("BallBeam_UNO continuous PID ready; motor output locked"));
  Serial.println(F("Startup pose is command zero; beam must be manually level"));
  Serial.println(F("Send only: PID <kp> <ki> <kd>"));
  printHelp();
  printStatus();
}

void loop() {
  readSerialLines();
  serviceSafetyState();
  autoArmWhenReady();
  updateRateRamp();
  updatePulseGenerator();
  printPeriodicStatus();
}
