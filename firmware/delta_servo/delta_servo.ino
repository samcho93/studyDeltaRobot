/*
  delta_servo.ino — studyDeltaRobot firmware for Arduino Uno / Nano
  3 hobby servos (MG996R) + one tool output (relay / MOSFET: vacuum pump or electromagnet)

  Serial 115200 baud, ASCII, one command per line ('\n', '\r' ignored):
    J <d1> <d2> <d3>   target motor angles [deg]; 0 = upper arm horizontal, + = arm down
    T <0|1>            tool off / on
    E                  EMERGENCY STOP: all servos detach (go limp), tool off, J/T ignored
    R                  reset after E: servos re-attach and hold the last angle
    ?                  status -> "OK <d1> <d2> <d3> <tool>" (current, slew-limited angles)
  Replies: J and T are silent (streamed at up to 50 Hz); E -> "ESTOP", R -> "READY",
  errors -> "ERR <reason>".  Host side: python/deltarobot/backends/serial_backend.py and
  the ROS 2 delta_driver (hardware:=serial).  See firmware/README.md.

  Angle -> pulse mapping (per servo i):
      pulse_us = ZERO_US[i] + DIR[i] * US_PER_DEG[i] * theta_deg
    ZERO_US    pulse at which the upper arm is exactly horizontal (theta = 0), found with a jig
    DIR        +1 if a LONGER pulse swings the arm DOWN, -1 if it swings it up.
               (servos mounted mirror-wise on the base usually all have the same DIR because
               every arm points outward; check each one)
    US_PER_DEG servo gain; MG996R ~ 10-11 us/deg (2000 us span / ~180-200 deg). Measure it.
  theta is clamped to [THETA_MIN, THETA_MAX] and the pulse to [PULSE_MIN, PULSE_MAX].
  The loop moves each servo toward its target at most MAX_SPEED_DEG_S (slew-rate limit),
  so a lost / jumpy command stream cannot slam the arms.
*/

#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// ------------------------------------------------------------------ configuration
const uint8_t SERVO_PIN[3] = {9, 10, 11};
const uint8_t TOOL_PIN = 7;               // HIGH = tool on (MOSFET gate / relay module input)
const bool TOOL_ACTIVE_HIGH = true;       // set false for active-LOW relay modules

// calibration — edit after the jig procedure in README.md
float ZERO_US[3]    = {1500.0, 1500.0, 1500.0};
int8_t DIR[3]       = {+1, +1, +1};
float US_PER_DEG[3] = {10.3, 10.3, 10.3};

const float THETA_MIN = -40.0;            // [deg] (design theta_min -0.7 rad)
const float THETA_MAX = 85.0;             // [deg] (design theta_max  1.5 rad)
const float HOME_DEG = 20.0;              // start pose (0.35 rad, DeltaDesign.home_theta)
const float MAX_SPEED_DEG_S = 200.0;      // slew-rate limit (MG996R @6 V ~ 300 deg/s unloaded)
const int PULSE_MIN = 600;                // hard pulse limits [us]
const int PULSE_MAX = 2400;
const unsigned long UPDATE_US = 10000;    // servo update period (100 Hz)

// ------------------------------------------------------------------ state
Servo servo[3];
float target[3];                          // commanded angle [deg]
float current[3];                         // slew-limited angle actually sent [deg]
bool toolOn = false;
bool estopped = false;
unsigned long lastUpdate = 0;

char line[64];
uint8_t lineLen = 0;
bool lineOverflow = false;
bool swallowLine = false;

// ------------------------------------------------------------------ helpers
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

int angleToPulse(uint8_t i, float deg) {
  float us = ZERO_US[i] + DIR[i] * US_PER_DEG[i] * deg;
  return (int)clampf(us + 0.5, PULSE_MIN, PULSE_MAX);
}

void setTool(bool on) {
  toolOn = on;
  digitalWrite(TOOL_PIN, (on == TOOL_ACTIVE_HIGH) ? HIGH : LOW);
}

void attachAll() {
  for (uint8_t i = 0; i < 3; i++) {
    servo[i].writeMicroseconds(angleToPulse(i, current[i]));  // set pulse before attach: no jump to 90
    servo[i].attach(SERVO_PIN[i], PULSE_MIN, PULSE_MAX);
  }
}

void detachAll() {
  for (uint8_t i = 0; i < 3; i++) servo[i].detach();
}

void emergencyStop() {
  estopped = true;
  detachAll();
  setTool(false);
  for (uint8_t i = 0; i < 3; i++) target[i] = current[i];
  Serial.println(F("ESTOP"));
}

void resetEstop() {
  if (estopped) {
    // the arms may have dropped while limp: hold the last commanded angle
    for (uint8_t i = 0; i < 3; i++) target[i] = current[i];
    attachAll();
    estopped = false;
  }
  Serial.println(F("READY"));
}

void printStatus() {
  Serial.print(F("OK"));
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(' ');
    Serial.print(current[i], 2);
  }
  Serial.print(' ');
  Serial.println(toolOn ? 1 : 0);
}

// parse "J d1 d2 d3" (after the 'J')
void commandJoints(char *args) {
  float v[3];
  char *p = args;
  for (uint8_t i = 0; i < 3; i++) {
    char *end;
    v[i] = (float)strtod(p, &end);
    if (end == p) { Serial.println(F("ERR J needs 3 numbers")); return; }
    p = end;
  }
  if (estopped) { Serial.println(F("ERR estop (send R)")); return; }
  for (uint8_t i = 0; i < 3; i++) target[i] = clampf(v[i], THETA_MIN, THETA_MAX);
}

void commandTool(char *args) {
  char *end;
  long on = strtol(args, &end, 10);
  if (end == args || (on != 0 && on != 1)) { Serial.println(F("ERR T needs 0 or 1")); return; }
  if (estopped) { Serial.println(F("ERR estop (send R)")); return; }
  setTool(on == 1);
}

void handleLine(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return;
  char cmd = s[0];
  char *args = s + 1;
  switch (cmd) {
    case 'J': case 'j': commandJoints(args); break;
    case 'T': case 't': commandTool(args); break;
    case 'E': case 'e': emergencyStop(); break;
    case 'R': case 'r': resetEstop(); break;
    case '?': printStatus(); break;
    default: Serial.println(F("ERR unknown command")); break;
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) {
        Serial.println(F("ERR line too long"));
      } else if (!swallowLine) {
        line[lineLen] = '\0';
        handleLine(line);
      }
      lineLen = 0;
      lineOverflow = false;
      swallowLine = false;
      continue;
    }
    if (swallowLine) continue;
    if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
    // 'E' at the start of a line acts immediately, without waiting for the newline
    if (lineLen == 1 && (line[0] == 'E' || line[0] == 'e')) {
      emergencyStop();
      lineLen = 0;
      swallowLine = true;    // ignore the rest of this line
    }
  }
}

void updateServos(float dt) {
  if (estopped) return;
  float step = MAX_SPEED_DEG_S * dt;
  for (uint8_t i = 0; i < 3; i++) {
    float err = target[i] - current[i];
    current[i] += clampf(err, -step, step);
    servo[i].writeMicroseconds(angleToPulse(i, current[i]));
  }
}

// ------------------------------------------------------------------ Arduino entry points
void setup() {
  pinMode(TOOL_PIN, OUTPUT);
  setTool(false);
  for (uint8_t i = 0; i < 3; i++) current[i] = target[i] = HOME_DEG;
  attachAll();
  Serial.begin(115200);
  Serial.println(F("READY delta_servo"));
  lastUpdate = micros();
}

void loop() {
  readSerial();
  unsigned long now = micros();
  unsigned long elapsed = now - lastUpdate;     // wrap-safe unsigned arithmetic
  if (elapsed >= UPDATE_US) {
    lastUpdate = now;
    float dt = elapsed * 1e-6;
    if (dt > 0.05) dt = 0.05;                   // never jump after a long pause
    updateServos(dt);
  }
}
