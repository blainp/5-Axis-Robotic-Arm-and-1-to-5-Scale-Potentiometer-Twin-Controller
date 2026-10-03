#include <AccelStepper.h>
#include <ServoTimer2.h>  // Uses Timer2 — no conflict with AccelStepper (Timer1)

// ─── STEPPER PINS ────────────────────────────────────────────────────────────
// Only 4 of these 5 physical stepper slots correspond to real joints
// (Base / Shoulder / Aux / Elbow). The unused slot simply never receives a
// nonzero motorDir from the Python side, so it stays put. The wrist is NOT
// a stepper at all — it's the MG996R servo below, driven by its own pot.
const int motorPins[5][2] = { {2, 3}, {4, 5}, {6, 7}, {8, 9}, {10, 11} };

AccelStepper steppers[5] = {
  AccelStepper(1, motorPins[0][1], motorPins[0][0]),
  AccelStepper(1, motorPins[1][1], motorPins[1][0]),
  AccelStepper(1, motorPins[2][1], motorPins[2][0]),
  AccelStepper(1, motorPins[3][1], motorPins[3][0]),
  AccelStepper(1, motorPins[4][1], motorPins[4][0])
};

// ─── SERVO SETUP ─────────────────────────────────────────────────────────────
ServoTimer2 wristServo;
ServoTimer2 clawServo;

// Track position in degrees (0–180) internally, convert on write
int wristDeg = 90;
int clawDeg  = 15;

// wristCmd is now an ABSOLUTE 0-180 target position (from the wrist
// potentiometer via Python), not a -1/0/1 direction like the steppers.
int wristCmd = 90;
int clawCmd  = 0;

// Degrees to move per servo update tick (every 10ms)
// 1 degree per tick = smooth, achievable steps for the MG996R
const int SERVO_STEP = 1;

// ─── STEPPER CONFIG ───────────────────────────────────────────────────────────
const float MAX_SPEED    = 1000.0;
const float SMOOTHING    = 0.1;
const float stepsPerDeg  = (200.0 * 80.0) / 360.0;

float currentSpeed[5] = {0, 0, 0, 0, 0};
int   motorDir[5]     = {0, 0, 0, 0, 0};

// ─── TIMING ──────────────────────────────────────────────────────────────────
unsigned long lastReport      = 0;
unsigned long lastServoUpdate = 0;

// ─── HELPERS ─────────────────────────────────────────────────────────────────
// Full MG996R range: 500–2500µs (was 1000–2000, which only used centre portion)
int degToUs(int deg) {
  return map(deg, 0, 180, 500, 2500);
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    steppers[i].setMaxSpeed(MAX_SPEED);
    steppers[i].setSpeed(0);
  }

  wristServo.attach(A0);
  clawServo.attach(A1);

  wristServo.write(degToUs(wristDeg));
  clawServo.write(degToUs(clawDeg));
}

// ─── MAIN LOOP ───────────────────────────────────────────────────────────────
void loop() {

  // 1. SERIAL
  if (Serial.available() > 0) {
    static char buffer[64];
    static int  idx = 0;
    char c = Serial.read();

    if (c == '<') {
      idx = 0;
    } else if (c == '>') {
      buffer[idx] = '\0';
      parseCommand(buffer);
      idx = 0;
    } else if (idx < 63) {
      buffer[idx++] = c;
    }
  }

  // 2. STEPPERS
  for (int i = 0; i < 5; i++) {
    float target = motorDir[i] * MAX_SPEED;
    currentSpeed[i] += (target - currentSpeed[i]) * SMOOTHING;
    steppers[i].setSpeed(currentSpeed[i]);
    steppers[i].runSpeed();
  }

  // 3. SERVO UPDATES
  if (millis() - lastServoUpdate > 10) {

    // WRIST: absolute position (0-180), driven directly by the wrist pot via Python
    wristDeg = constrain(wristCmd, 0, 180);
    wristServo.write(degToUs(wristDeg));

    // CLAW: TOTALLY UNTOUCHED - Same as your original code
    if (clawCmd == -1) {
      clawDeg = 80;   // CLOSE
    } 
    else if (clawCmd == 1) {
      clawDeg = -60;  // OPEN
    }
    clawServo.write(degToUs(clawDeg));

    lastServoUpdate = millis();
  }

  // 4. REPORT
  if (millis() - lastReport > 100) {
    for (int i = 0; i < 5; i++) {
      Serial.print(steppers[i].currentPosition() / stepsPerDeg, 1);
      if (i < 4) Serial.print(',');
    }
    Serial.println();
    lastReport = millis();
  }
}

// ─── PARSE COMMAND ───────────────────────────────────────────────────────────
void parseCommand(char* msg) {
  char* part;
  int   count = 0;

  part = strtok(msg, ",");

  while (part != NULL && count < 7) {
    int val = atoi(part);

    if      (count < 5)  motorDir[count] = val;
    // FIX: wristCmd is an ABSOLUTE 0-180 position, not a -1/0/1 direction.
    // The old `constrain(val, -1, 1)` here silently threw away every real
    // position value Python sent, which is why the wrist never moved.
    else if (count == 5) wristCmd = constrain(val, 0, 180);
    else if (count == 6) clawCmd  = constrain(val, -1, 1);

    part = strtok(NULL, ",");
    count++;
  }
}
