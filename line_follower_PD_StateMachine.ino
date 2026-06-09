// ================================================================
//  Line Follower — PD Control + State Machine Prediction
//  Track  : WHITE backdrop, BLACK line (2.5 cm wide)
//  Sensors: 2x IR, spaced ~4 cm apart (wider than the line)
//
//  HOW IT WORKS:
//    PD         → smoothly corrects steering based on current error
//    State Machine → watches history of last 6 states to predict
//                    what's coming (straight / curve / sharp corner)
//                    and adjusts base speed BEFORE it happens
// ================================================================

// --- Motor Driver (L298N) ---
#define ENA  6
#define IN1  9
#define IN2  3
#define ENB  11
#define IN3  10
#define IN4  5

// --- IR Sensors ---
#define LEFT_IR  A0
#define RIGHT_IR A1

// --- Ultrasonic (HC-SR04) ---
#define TRIG_PIN 12
#define ECHO_PIN 13
#define OBSTACLE_DIST 15   // cm

// ================================================================
//  Speed Settings
// ================================================================
#define SPEED_FAST    230   // confirmed long straight
#define SPEED_NORMAL  180   // default / gentle curve
#define SPEED_SLOW    130   // sharp or sustained corner
#define SPEED_SEARCH   90   // line fully lost, searching
#define MAX_SPEED     255
#define MIN_SPEED       0

// ================================================================
//  PD Gains  — tune these on your track
//  Start: Kp=80, Kd=25. Increase Kp for sharper turns,
//         increase Kd to reduce oscillation/wiggle.
// ================================================================
float Kp = 80.0;
float Kd = 25.0;

// ================================================================
//  State Machine
// ================================================================
#define STATE_STRAIGHT    0
#define STATE_TURN_LEFT   1
#define STATE_TURN_RIGHT  2
#define STATE_LOST        3

#define HISTORY_LEN  6          // how many past states to remember
int stateHistory[HISTORY_LEN];  // index 0 = most recent

// Pattern match results → predicted scenario
#define SCENARIO_STRAIGHT     0   // long straight ahead
#define SCENARIO_GENTLE_CURVE 1   // single turn detected
#define SCENARIO_SHARP_CORNER 2   // sustained turn same direction
#define SCENARIO_S_CURVE      3   // alternating turns (S-bend)
#define SCENARIO_LOST         4   // line missing

// ================================================================
//  PD State Variables
// ================================================================
int           lastError      = 0;
int           lastKnownError = 0;
unsigned long lastTime       = 0;

// ================================================================
//  Utility: Ultrasonic
// ================================================================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999;
  return dur * 0.034 / 2;
}

// ================================================================
//  Utility: Motors
// ================================================================
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);    analogWrite(ENB, 0);
}

// ================================================================
//  State Machine: push new state into history
//  History is a shift register: index 0 = newest, 5 = oldest
// ================================================================
void pushState(int newState) {
  for (int i = HISTORY_LEN - 1; i > 0; i--) {
    stateHistory[i] = stateHistory[i - 1];
  }
  stateHistory[0] = newState;
}

// ================================================================
//  State Machine: pattern match history → predict scenario
//  Called every loop AFTER pushState()
// ================================================================
int predictScenario() {

  // --- LOST: current state is lost ---
  if (stateHistory[0] == STATE_LOST) {
    return SCENARIO_LOST;
  }

  // --- SHARP CORNER: same turn direction for 3+ consecutive loops ---
  if ((stateHistory[0] == STATE_TURN_LEFT  &&
       stateHistory[1] == STATE_TURN_LEFT  &&
       stateHistory[2] == STATE_TURN_LEFT) ||
      (stateHistory[0] == STATE_TURN_RIGHT &&
       stateHistory[1] == STATE_TURN_RIGHT &&
       stateHistory[2] == STATE_TURN_RIGHT)) {
    return SCENARIO_SHARP_CORNER;
  }

  // --- S-CURVE: alternating left-right or right-left turns ---
  if ((stateHistory[0] == STATE_TURN_LEFT  &&
       stateHistory[1] == STATE_TURN_RIGHT &&
       stateHistory[2] == STATE_TURN_LEFT) ||
      (stateHistory[0] == STATE_TURN_RIGHT &&
       stateHistory[1] == STATE_TURN_LEFT  &&
       stateHistory[2] == STATE_TURN_RIGHT)) {
    return SCENARIO_S_CURVE;
  }

  // --- LONG STRAIGHT: all 6 history slots are straight ---
  bool allStraight = true;
  for (int i = 0; i < HISTORY_LEN; i++) {
    if (stateHistory[i] != STATE_STRAIGHT) {
      allStraight = false;
      break;
    }
  }
  if (allStraight) return SCENARIO_STRAIGHT;

  // --- GENTLE CURVE: current state is a single turn, not sustained ---
  if (stateHistory[0] == STATE_TURN_LEFT ||
      stateHistory[0] == STATE_TURN_RIGHT) {
    return SCENARIO_GENTLE_CURVE;
  }

  // --- DEFAULT: normal straight running ---
  return SCENARIO_STRAIGHT;
}

// ================================================================
//  Setup
// ================================================================
void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LEFT_IR,  INPUT);
  pinMode(RIGHT_IR, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialise history to STRAIGHT
  for (int i = 0; i < HISTORY_LEN; i++) stateHistory[i] = STATE_STRAIGHT;

  Serial.begin(9600);
  delay(2000);
  lastTime = millis();
  Serial.println("PD + State Machine Prediction Ready");
}

// ================================================================
//  Main Loop
// ================================================================
void loop() {

  // ── 1. OBSTACLE CHECK ────────────────────────────────────────
  if (getDistance() < OBSTACLE_DIST) {
    stopMotors();
    Serial.println("OBSTACLE");
    delay(100);
    return;
  }

  // ── 2. READ SENSORS ──────────────────────────────────────────
  bool leftOnLine  = (digitalRead(LEFT_IR)  == LOW);  // true = black
  bool rightOnLine = (digitalRead(RIGHT_IR) == LOW);

  // ── 3. DETERMINE CURRENT STATE + ERROR ───────────────────────
  int currentState;
  int error;

  if (!leftOnLine && !rightOnLine) {
    // Both white → centred, line is in the gap between sensors
    currentState = STATE_STRAIGHT;
    error        = 0;
  }
  else if (leftOnLine && !rightOnLine) {
    // Left on black → robot drifted left → steer right
    currentState      = STATE_TURN_RIGHT;
    error             = -1;
    lastKnownError    = -1;
  }
  else if (!leftOnLine && rightOnLine) {
    // Right on black → robot drifted right → steer left
    currentState      = STATE_TURN_LEFT;
    error             = 1;
    lastKnownError    = 1;
  }
  else {
    // Both black → fully lost
    currentState = STATE_LOST;
    error        = (lastKnownError >= 0) ? 2 : -2;
  }

  // ── 4. UPDATE STATE HISTORY ───────────────────────────────────
  pushState(currentState);

  // ── 5. PREDICT SCENARIO FROM HISTORY ─────────────────────────
  int scenario = predictScenario();

  // ── 6. SET BASE SPEED FROM PREDICTION ────────────────────────
  //  This is the "predictive" part:
  //  Robot adjusts speed BASED ON WHAT IT HAS SEEN, not just now
  int baseSpeed;
  switch (scenario) {
    case SCENARIO_STRAIGHT:
      baseSpeed = SPEED_FAST;      // confident: go fast
      break;
    case SCENARIO_GENTLE_CURVE:
      baseSpeed = SPEED_NORMAL;    // single turn: normal speed
      break;
    case SCENARIO_SHARP_CORNER:
      baseSpeed = SPEED_SLOW;      // sustained turn: slow down hard
      break;
    case SCENARIO_S_CURVE:
      baseSpeed = SPEED_SLOW;      // S-bend: slow for control
      break;
    case SCENARIO_LOST:
      baseSpeed = SPEED_SEARCH;    // lost: creep slowly while searching
      break;
    default:
      baseSpeed = SPEED_NORMAL;
  }

  // ── 7. PD CALCULATION ────────────────────────────────────────
  unsigned long now = millis();
  float dt = (float)(now - lastTime);
  if (dt < 1) dt = 1;
  lastTime = now;

  float derivative = (error - lastError) / dt;
  float correction = (Kp * error) + (Kd * derivative);
  lastError = error;

  // ── 8. APPLY TO MOTORS ────────────────────────────────────────
  int leftSpeed  = (int)(baseSpeed - correction);
  int rightSpeed = (int)(baseSpeed + correction);
  setMotors(leftSpeed, rightSpeed);

  // ── 9. DEBUG OUTPUT ───────────────────────────────────────────
  Serial.print("L:"); Serial.print(leftOnLine);
  Serial.print(" R:"); Serial.print(rightOnLine);
  Serial.print(" state:");
  switch(currentState) {
    case STATE_STRAIGHT:   Serial.print("STR"); break;
    case STATE_TURN_LEFT:  Serial.print("TL "); break;
    case STATE_TURN_RIGHT: Serial.print("TR "); break;
    case STATE_LOST:       Serial.print("LST"); break;
  }
  Serial.print(" scenario:");
  switch(scenario) {
    case SCENARIO_STRAIGHT:     Serial.print("STRAIGHT    "); break;
    case SCENARIO_GENTLE_CURVE: Serial.print("GENTLE_CURVE"); break;
    case SCENARIO_SHARP_CORNER: Serial.print("SHARP_CORNER"); break;
    case SCENARIO_S_CURVE:      Serial.print("S_CURVE     "); break;
    case SCENARIO_LOST:         Serial.print("LOST        "); break;
  }
  Serial.print(" err:"); Serial.print(error);
  Serial.print(" corr:"); Serial.print(correction, 1);
  Serial.print(" base:"); Serial.print(baseSpeed);
  Serial.print(" L:"); Serial.print(leftSpeed);
  Serial.print(" R:"); Serial.println(rightSpeed);
}
