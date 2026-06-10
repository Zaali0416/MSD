#include <Arduino.h>

// ==========================================
// PIN CONFIGURATION
// ==========================================

// Front Motor Driver (TB6612FNG)
const int F_PWMA = 7;
const int F_PWMB = 13;
const int F_INA1 = 9;
const int F_INA2 = 8;
const int F_INB1 = 11;
const int F_INB2 = 12;
const int F_STBY = 10;

// Back Motor Driver (TB6612FNG)
const int B_PWMA = 0;
const int B_PWMB = 6;
const int B_INA1 = 2;
const int B_INA2 = 1;
const int B_INB1 = 4;
const int B_INB2 = 5;
const int B_STBY = 3;

// Buttons
const int START_BUTTON = 49;
const int RESET_BUTTON = 51;

// TCRT5000 5-Channel Array
const int LINE_SENSORS[5] = {32, 34, 36, 38, 40};

// Encoders (Using Analog Pins as Digital Inputs)
const int ENCODER_BR_A = A3; // AD3
const int ENCODER_BR_B = A2; // AD2
const int ENCODER_BL_A = A1; // AD1
const int ENCODER_BL_B = A0; // AD0

// ==========================================
// ROBOT SPECIFICATIONS & CONSTANTS
// ==========================================
const float WHEEL_DIAMETER = 65.0; // EDIT THIS: Back wheel diameter in mm
const float CPR = 330.0;            // EDIT THIS: Total encoder counts per wheel revolution
const float MM_PER_TICK = (WHEEL_DIAMETER * PI) / CPR;

const float DIST_FRONT_BACK = 137.0; // mm
const float TRACK_WIDTH = 235.0;     // mm

// ==========================================
// GLOBAL VARIABLES
// ==========================================
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

int junctionCount = 0;
bool onJunction = false;
bool box1_found = false;
char box1_location = ' '; // Can be 'A', 'B', 'D', or 'E'

// Finite State Machine States
enum State {
  STANDBY,
  START_TO_JUNCTION_A,
  CHECK_BOX1_A,
  A_TO_B,
  CHECK_BOX1_B,
  B_TO_C,
  C_TO_D,
  CHECK_BOX1_D,
  D_TO_E,
  CHECK_BOX1_E,
  ROUTE_TO_C,
  C_TO_F,
  APPROACH_BOX2,
  BOX2_TO_BOX3,
  COMPLETED
};

State currentState = STANDBY;

// ==========================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// ==========================================
void isrLeftEncoder() {
  if (digitalRead(ENCODER_BL_A) == digitalRead(ENCODER_BL_B)) leftEncoderTicks++;
  else leftEncoderTicks--;
}

void isrRightEncoder() {
  if (digitalRead(ENCODER_BR_A) == digitalRead(ENCODER_BR_B)) rightEncoderTicks++;
  else rightEncoderTicks--;
}

// ==========================================
// MOTOR & NAVIGATION CORE FUNCTIONS
// ==========================================
void setMotors(int leftSpeed, int rightSpeed) {
  // Front Motors
  digitalWrite(F_STBY, HIGH);
  if (leftSpeed >= 0) {
    digitalWrite(F_INA1, HIGH); digitalWrite(F_INA2, LOW);
    analogWrite(F_PWMA, leftSpeed);
  } else {
    digitalWrite(F_INA1, LOW); digitalWrite(F_INA2, HIGH);
    analogWrite(F_PWMA, abs(leftSpeed));
  }
  if (rightSpeed >= 0) {
    digitalWrite(F_INB1, HIGH); digitalWrite(F_INB2, LOW);
    analogWrite(F_PWMB, rightSpeed);
  } else {
    digitalWrite(F_INB1, LOW); digitalWrite(F_INB2, HIGH);
    analogWrite(F_PWMB, abs(rightSpeed));
  }

  // Back Motors
  digitalWrite(B_STBY, HIGH);
  if (leftSpeed >= 0) {
    digitalWrite(B_INA1, HIGH); digitalWrite(B_INA2, LOW);
    analogWrite(B_PWMA, leftSpeed);
  } else {
    digitalWrite(B_INA1, LOW); digitalWrite(B_INA2, HIGH);
    analogWrite(B_PWMA, abs(leftSpeed));
  }
  if (rightSpeed >= 0) {
    digitalWrite(B_INB1, HIGH); digitalWrite(B_INB2, LOW);
    analogWrite(B_PWMB, rightSpeed);
  } else {
    digitalWrite(B_INB1, LOW); digitalWrite(B_INB2, HIGH);
    analogWrite(B_PWMB, abs(rightSpeed));
  }
}

void resetEncoders() {
  noInterrupts();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
  interrupts();
}

float getDistanceTraveled() {
  noInterrupts();
  long left = leftEncoderTicks;
  long right = rightEncoderTicks;
  interrupts();
  return (abs(left) + abs(right)) / 2.0 * MM_PER_TICK;
}

// Line Tracking Subroutine with built-in junction updating
void lineFollow(int baseSpeed) {
  int s1 = digitalRead(LINE_SENSORS[0]);
  int s2 = digitalRead(LINE_SENSORS[1]);
  int s3 = digitalRead(LINE_SENSORS[2]);
  int s4 = digitalRead(LINE_SENSORS[3]);
  int s5 = digitalRead(LINE_SENSORS[4]);

  // Check for Junction (All 5 sensors detect Black line, assuming HIGH = Black)
  if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == HIGH) {
    if (!onJunction) {
      junctionCount++;
      onJunction = true;
    }
  } else {
    onJunction = false;
  }

  // Basic proportional steering correction
  int error = 0;
  if (s1 == HIGH) error = -2;
  else if (s2 == HIGH) error = -1;
  else if (s3 == HIGH) error = 0;
  else if (s4 == HIGH) error = 1;
  else if (s5 == HIGH) error = 2;

  int Kp = 30; // Tune this value for turning aggressiveness
  int correction = error * Kp;
  
  setMotors(baseSpeed + correction, baseSpeed - correction);
}

void executeTurn(char direction) {
  resetEncoders();
  // Target arc length calculated from track width for a 90 degree turn
  float targetDist = (TRACK_WIDTH * PI) / 4.0; 
  
  while (getDistanceTraveled() < targetDist) {
    if (direction == 'L') {
      setMotors(-150, 150);
    } else if (direction == 'R') {
      setMotors(150, -150);
    }
  }
  setMotors(0, 0);
  delay(200); // Settling time
}

void moveDistance(float dist_mm, int speed, bool followLine = false) {
  resetEncoders();
  while (getDistanceTraveled() < dist_mm) {
    if (followLine) {
      lineFollow(speed);
    } else {
      setMotors(speed, speed);
    }
  }
  setMotors(0, 0);
}

// Simulated placeholder function for sensor arrays checking box availability
bool isOpeningDetected() {
  // Insert your distance sensor/limit switch check logic here
  return false; 
}

// ==========================================
// ARDUINO CORE SETUP & LOOP
// ==========================================
void setup() {
  // Initialize Motor Pins
  pinMode(F_PWMA, OUTPUT); pinMode(F_PWMB, OUTPUT);
  pinMode(F_INA1, OUTPUT); pinMode(F_INA2, OUTPUT);
  pinMode(F_INB1, OUTPUT); pinMode(F_INB2, OUTPUT);
  pinMode(F_STBY, OUTPUT);
  
  pinMode(B_PWMA, OUTPUT); pinMode(B_PWMB, OUTPUT);
  pinMode(B_INA1, OUTPUT); pinMode(B_INA2, OUTPUT);
  pinMode(B_INB1, OUTPUT); pinMode(B_INB2, OUTPUT);
  pinMode(B_STBY, OUTPUT);

  // Initialize Sensors & Buttons
  for(int i=0; i<5; i++) pinMode(LINE_SENSORS[i], INPUT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);

  // Initialize Encoders
  pinMode(ENCODER_BL_A, INPUT); pinMode(ENCODER_BL_B, INPUT);
  pinMode(ENCODER_BR_A, INPUT); pinMode(ENCODER_BR_B, INPUT);
  
  // Attach interrupts for precision encoder tracking
  attachInterrupt(digitalPinToInterrupt(ENCODER_BL_A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BR_A), isrRightEncoder, CHANGE);
}

void loop() {
  // Global Hardware Reset Button check
  if (digitalRead(RESET_BUTTON) == LOW) {
    setMotors(0, 0);
    currentState = STANDBY;
    junctionCount = 0;
    box1_found = false;
    delay(500);
  }

  switch (currentState) {
    
    case STANDBY:
      setMotors(0, 0);
      if (digitalRead(START_BUTTON) == LOW) {
        junctionCount = 0;
        currentState = START_TO_JUNCTION_A;
        delay(500); // Debounce
      }
      break;

    case START_TO_JUNCTION_A:
      lineFollow(150);
      // Map conditions: Ignore junc 1 (60cm), junc 2 (30cm), junc 3 (30cm). Turn left at junc 4. 
      // Ignore junc 5 (30cm), junc 6 (30cm). Turn left at junc 7. Next is Junction_A (junc 8)
      if (junctionCount == 4 && onJunction) {
        executeTurn('L');
      }
      if (junctionCount == 7 && onJunction) {
        executeTurn('L');
      }
      if (junctionCount == 8) {
        setMotors(0, 0);
        currentState = CHECK_BOX1_A;
      }
      break;

    case CHECK_BOX1_A:
      moveDistance(233.0, 100, true); // 23.3 cm forward
      if (isOpeningDetected()) {
        box1_found = true;
        box1_location = 'A';
        moveDistance(233.0, -100, false); // reverse to Junction A
        currentState = ROUTE_TO_C;
      } else {
        moveDistance(233.0, -100, false); // reverse to Junction A
        currentState = A_TO_B;
      }
      break;

    case A_TO_B:
      executeTurn('R');
      junctionCount = 0;
      // 30cm to next junction, turn left, 30cm forward to Junction_B
      while (junctionCount < 1) { lineFollow(150); }
      executeTurn('L');
      resetEncoders();
      moveDistance(300.0, 150, true);
      currentState = CHECK_BOX1_B;
      break;

    case CHECK_BOX1_B:
      executeTurn('R');
      moveDistance(233.0, 100, true);
      if (isOpeningDetected()) {
        box1_found = true; box1_location = 'B';
        moveDistance(233.0, -100, false);
        executeTurn('R'); // Orient back towards trajectory C
        currentState = B_TO_C;
      } else {
        moveDistance(233.0, -100, false);
        executeTurn('R');
        currentState = B_TO_C;
      }
      break;

    case B_TO_C:
      junctionCount = 0;
      while (junctionCount < 1) { lineFollow(150); } // Reaching Junction C
      if (box1_found) {
        currentState = C_TO_F;
      } else {
        currentState = C_TO_D;
      }
      break;

    case C_TO_D:
      executeTurn('L');
      resetEncoders();
      moveDistance(300.0, 150, true); // Move to Junction D
      currentState = CHECK_BOX1_D;
      break;

    case CHECK_BOX1_D:
      executeTurn('L');
      moveDistance(233.0, 100, true);
      if (isOpeningDetected()) {
        box1_found = true; box1_location = 'D';
        moveDistance(233.0, -100, false);
        executeTurn('L'); // Turn back towards C
        currentState = ROUTE_TO_C;
      } else {
        moveDistance(233.0, -100, false);
        executeTurn('R'); // Orient towards E
        currentState = D_TO_E;
      }
      break;

    case D_TO_E:
      // From D: 30cm, Left, 30cm, Left, 30cm to Junction_E
      junctionCount = 0;
      while(junctionCount < 1) { lineFollow(150); }
      executeTurn('L');
      junctionCount = 0;
      while(junctionCount < 1) { lineFollow(150); }
      executeTurn('L');
      resetEncoders();
      moveDistance(300.0, 150, true);
      currentState = CHECK_BOX1_E;
      break;

    case CHECK_BOX1_E:
      executeTurn('L');
      moveDistance(233.0, 100, true);
      if (isOpeningDetected()) {
        box1_found = true; box1_location = 'E';
      }
      moveDistance(233.0, -100, false);
      executeTurn('L'); // Redirect to C
      currentState = ROUTE_TO_C;
      break;

    case ROUTE_TO_C:
      // Logic handling returning from D or E back to C
      if (box1_location == 'D' || box1_location == 'E' || box1_location == ' ') {
        junctionCount = 0;
        while(junctionCount < 1) { lineFollow(150); } // Reached C
        executeTurn('L'); 
      }
      resetEncoders();
      moveDistance(300.0, 150, true);
      currentState = C_TO_F;
      break;

    case C_TO_F:
      // From C: Turn right, 30cm, ignore junc, 30cm, ignore junc, 30cm to Junction F
      executeTurn('R');
      junctionCount = 0;
      while (junctionCount < 3) { lineFollow(150); }
      currentState = APPROACH_BOX2;
      break;

    case APPROACH_BOX2:
      executeTurn('L');
      resetEncoders();
      moveDistance(300.0, 150, true); // Front at Box 2
      setMotors(0, 0);
      delay(2000); // Simulate action/drop at Box 2
      currentState = BOX2_TO_BOX3;
      break;

    case BOX2_TO_BOX3:
      // Reverse out back to Junction F
      moveDistance(300.0, -150, false);
      executeTurn('R');
      
      // Hardcoded tracking to BOX3 based on map parameters
      junctionCount = 0;
      while (junctionCount < 1) { lineFollow(150); } // 30cm forward
      executeTurn('R');
      junctionCount = 0;
      while (junctionCount < 2) { lineFollow(150); } // 30cm forward ignore, 30cm
      executeTurn('R');
      junctionCount = 0;
      while (junctionCount < 1) { lineFollow(150); } // 30cm forward
      executeTurn('L');
      junctionCount = 0;
      while (junctionCount < 1) { lineFollow(150); } // 30cm forward
      executeTurn('R');
      junctionCount = 0;
      while (junctionCount < 1) { lineFollow(150); } // 30cm forward
      executeTurn('L');
      
      // Custom distance steps
      moveDistance(300.0, 150, true); // ignore
      moveDistance(360.0, 150, true); // 36cm ignore
      junctionCount = 0;
      while (junctionCount < 1) { lineFollow(150); } // 30cm forward
      executeTurn('R');
      junctionCount = 0;
      while (junctionCount < 2) { lineFollow(150); } // 30cm forward ignore, 30cm forward to Box3
      
      setMotors(0, 0);
      currentState = COMPLETED;
      break;

    case COMPLETED:
      setMotors(0,0); // Goal reached
      break;
  }
}
