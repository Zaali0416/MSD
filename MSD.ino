#include <Arduino.h>

// ==========================================
// PIN CONFIGURATION
// ==========================================
const int F_PWMA = 7;   const int F_PWMB = 13;
const int F_INA1 = 9;   const int F_INA2 = 8;
const int F_INB1 = 11;  const int F_INB2 = 12;
const int F_STBY = 10;

const int B_PWMA = 15;  
const int B_PWMB = 6;
const int B_INA1 = 2;   
const int B_INA2 = 14;  
const int B_INB1 = 4;   const int B_INB2 = 5;
const int B_STBY = 3;

const int START_BUTTON = 49;

// Line Sensor Array Pins
const int LINE_SENSORS[5] = {32, 34, 36, 38, 40};

// Physical Port F Analog Pins
const int ENCODER_BL_A = A1; 
const int ENCODER_BL_B = A0; 
const int ENCODER_BR_A = A3; 
const int ENCODER_BR_B = A2; 

// Front Ultrasonic Sensor Configuration
const int TRIG_FRONT = 52;
const int ECHO_FRONT = 53;
const int FRONT_DISTANCE_THRESHOLD = 15; 

// ==========================================
// YOUR PATH ROUTING ARRAY
// ==========================================
char path[] = {'F', 'F', 'F', 'L', 'F', 'F', 'F'}; 
int pathIndex = 0; 
const int TOTAL_JUNCTIONS = sizeof(path) / sizeof(path[0]);

// ==========================================
// VELOCITY PROFILES (Balanced for Straight Tracking)
// ==========================================
const int CRUISE_LEFT         = 200;  
const int CRUISE_RIGHT_ADJUST = 165;  

// Standard tracking smooth correction speeds
const int CORRECTION_FAST     = 180;  
const int CORRECTION_SLOW     = 110;  

// Extreme boundary recovery speeds (Opposing forces)
const int HARD_ESCAPE_FORWARD = 160;  
const int HARD_ESCAPE_REVERSE = -90;  

// Turn & Reverse Parameters
const int REVERSE_SPEED       = -110; 
const int REVERSE_TICKS       = 100; 

// 90-DEGREE DIAGONAL POWER SPEEDS
const int TURN_POWER_HIGH = 245; // High breakaway torque
const int TURN_POWER_MID  = 185; 

const int SEARCH_POWER_HIGH = 195; // Controlled search torque to prevent overshoot
const int SEARCH_POWER_MID  = 145;

// GLOBAL TARGET VARIABLES
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

bool onJunction = false;       
bool testingActive = false;

// ==========================================
// PORT F ENCODER PROCESSING LAYER
// ==========================================
void updateEncoders() {
  static uint8_t lastPortF = 0;
  uint8_t currentPortF = PINF; 
  
  uint8_t blA = (currentPortF >> 1) & 0x01; 
  uint8_t blB = (currentPortF >> 0) & 0x01; 
  uint8_t brA = (currentPortF >> 3) & 0x01; 
  uint8_t brB = (currentPortF >> 2) & 0x01; 
  
  if (blA != ((lastPortF >> 1) & 0x01)) { if (blA != blB) leftEncoderTicks++; else leftEncoderTicks--; }
  if (brA != ((lastPortF >> 3) & 0x01)) { if (brA != brB) rightEncoderTicks++; else rightEncoderTicks--; }
  
  lastPortF = currentPortF;
}

long getAverageTicks() {
  updateEncoders();
  return (abs(leftEncoderTicks) + abs(rightEncoderTicks)) / 2;
}

void resetMovement() {
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
}

// ==========================================
// GLOBAL PHASE-INVERTED MOTOR ENGINE
// ==========================================
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(F_STBY, HIGH); 
  digitalWrite(B_STBY, HIGH);
  
  if(leftSpeed >= 0) {  
    digitalWrite(F_INA1, HIGH); digitalWrite(F_INA2, LOW); 
    digitalWrite(B_INA1, HIGH); digitalWrite(B_INA2, LOW); 
  } else {                
    digitalWrite(F_INA1, LOW);  digitalWrite(F_INA2, HIGH); 
    digitalWrite(B_INA1, LOW);  digitalWrite(B_INA2, HIGH); 
  }
  
  if(rightSpeed >= 0) { 
    digitalWrite(F_INB1, LOW);  digitalWrite(F_INB2, HIGH); 
    digitalWrite(B_INB1, HIGH); digitalWrite(B_INB2, LOW);  
  } else {                
    digitalWrite(F_INB1, HIGH); digitalWrite(F_INB2, LOW);  
    digitalWrite(B_INB1, LOW);  digitalWrite(B_INB2, HIGH); 
  }
  
  analogWrite(F_PWMA, constrain(abs(leftSpeed), 0, 255));
  analogWrite(B_PWMA, constrain(abs(leftSpeed), 0, 255));
  
  analogWrite(F_PWMB, constrain(abs(rightSpeed), 0, 255));
  analogWrite(B_PWMB, constrain(abs(rightSpeed), 0, 255));
}

// ==========================================
// SPECIAL DIAGONAL PIVOT TURN MOTOR OVERRIDE
// ==========================================
void setDiagonalTurnMotors(int frontPower, int backPower) {
  digitalWrite(F_STBY, HIGH); 
  digitalWrite(B_STBY, HIGH);
  
  // Front Left: BACKWARD | Back Left: FORWARD
  digitalWrite(F_INA1, LOW);  digitalWrite(F_INA2, HIGH);
  digitalWrite(B_INA1, HIGH); digitalWrite(B_INA2, LOW);
  
  // Front Right: FORWARD | Back Right: BACKWARD
  digitalWrite(F_INB1, LOW);  digitalWrite(F_INB2, HIGH);
  digitalWrite(B_INB1, HIGH); digitalWrite(B_INB2, LOW);
  
  analogWrite(F_PWMA, frontPower); 
  analogWrite(B_PWMA, backPower);  
  analogWrite(F_PWMB, frontPower); 
  analogWrite(B_PWMB, backPower);  
}

// ==========================================
// SENSOR UTILITIES
// ==========================================
uint8_t readSensorMask() {
  uint8_t mask = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(LINE_SENSORS[i]) == HIGH) mask |= (1 << (4 - i)); 
  }
  return mask;
}

int getFrontDistance() {
  digitalWrite(TRIG_FRONT, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  
  long duration = pulseIn(ECHO_FRONT, HIGH, 6000); 
  if (duration <= 0) return 999; 
  return duration * 0.0343 / 2;
}

// ==========================================
// 90-DEGREE DIAGONAL AXIS LEFT TURN MANEUVER
// ==========================================
void executeLeftTurn() {
  // 1. Hard Brake Snap 
  setMotors(-150, -150); delay(100);
  setMotors(0, 0); delay(150); 

 
  setMotors(120, 120); delay(100);
  setMotors(0, 0); delay(200);

  // 3. Ultrasonic Safety Check
  if (getFrontDistance() <= FRONT_DISTANCE_THRESHOLD) {
    setMotors(0, 0);
    while (getFrontDistance() <= FRONT_DISTANCE_THRESHOLD) { delay(50); }
    delay(200);
  }

  // 4. High-Torque Breakaway Phase
  unsigned long turnTimerSnapshot = millis();
  setDiagonalTurnMotors(TURN_POWER_HIGH, TURN_POWER_MID);
  delay(280); // Blindly clear old line vectors completely

  // 5. Controlled Search Sweep Phase (Slowing down slightly to hit 90° perfectly)
  while (true) {
    setDiagonalTurnMotors(SEARCH_POWER_HIGH, SEARCH_POWER_MID); 
    
    // Lock completely onto the 90-degree intersection line via center sensor
    if (digitalRead(LINE_SENSORS[2]) == HIGH) {
      break; 
    }

    if (millis() - turnTimerSnapshot > 2500) {
      break; 
    }
  }

  // 6. Active Counter-Brake Snap
  setMotors(-160, 140); delay(40); 
  setMotors(0, 0); delay(150);
}

void executeForwardPass() {
  unsigned long startMove = millis();
  while (readSensorMask() == 0b00000 && (millis() - startMove < 450)) {
    if (getFrontDistance() <= FRONT_DISTANCE_THRESHOLD) {
      setMotors(0, 0);
      while (getFrontDistance() <= FRONT_DISTANCE_THRESHOLD) { delay(10); }
    }
    setMotors(CRUISE_LEFT, CRUISE_RIGHT_ADJUST);
  }
}

// ==========================================
// MAIN SYSTEM OPERATIONAL SCHEDULE
// ==========================================
void setup() {
  Serial.begin(115200);
  
  pinMode(F_PWMA, OUTPUT); pinMode(F_PWMB, OUTPUT); pinMode(F_INA1, OUTPUT); pinMode(F_INA2, OUTPUT); pinMode(F_INB1, OUTPUT); pinMode(F_INB2, OUTPUT); pinMode(F_STBY, OUTPUT);
  pinMode(B_PWMA, OUTPUT); pinMode(B_PWMB, OUTPUT); pinMode(B_INA1, OUTPUT); pinMode(B_INA2, OUTPUT); pinMode(B_INB1, OUTPUT); pinMode(B_INB2, OUTPUT); pinMode(B_STBY, OUTPUT);
  
  for(int i=0; i<5; i++) pinMode(LINE_SENSORS[i], INPUT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  
  pinMode(ENCODER_BL_A, INPUT); pinMode(ENCODER_BL_B, INPUT);
  pinMode(ENCODER_BR_A, INPUT); pinMode(ENCODER_BR_B, INPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  
  Serial.println("[ONLINE] Sharp 90-Degree Turn Mapping Online.");
}

void loop() {
  if (!testingActive) {
    setMotors(0, 0);
    if (digitalRead(START_BUTTON) == LOW) {
      pathIndex = 0;
      onJunction = false;
      resetMovement();
      testingActive = true;
      delay(500);
    }
    return;
  }

  if (getFrontDistance() <= FRONT_DISTANCE_THRESHOLD) {
    setMotors(0, 0);
    while (getFrontDistance() <= FRONT_DISTANCE_THRESHOLD) { delay(50); }
  }

  updateEncoders();
  uint8_t sensorMask = readSensorMask();

  // --- 1. INTERSECTION ROUTING HANDLER ---
  if (sensorMask == 0b00000) { 
    if (!onJunction) {
      onJunction = true;
      
      Serial.print("[ARRAY NAVIGATION] Node: "); Serial.print(pathIndex);
      Serial.print(" | Action: "); Serial.println(path[pathIndex]);

      if (path[pathIndex] == 'L') {
        executeLeftTurn();
      } else {
        executeForwardPass();
      }

      pathIndex++; 
      
      if (pathIndex >= TOTAL_JUNCTIONS) {
        setMotors(-150, -150); delay(100); 
        setMotors(0, 0);
        testingActive = false;
        return;
      }
    }
    return;
  } 
  
  if ((sensorMask & 0b01110) != 0) {
    onJunction = false; 
  }

  // --- 2. HIGH-PRIORITY CENTER-LOCK TRACKING MATRIX ---
  bool farLeft    = (digitalRead(LINE_SENSORS[0]) == HIGH);
  bool midLeft    = (digitalRead(LINE_SENSORS[1]) == HIGH);
  bool centerOn   = (digitalRead(LINE_SENSORS[2]) == HIGH);
  bool midRight   = (digitalRead(LINE_SENSORS[3]) == HIGH);
  bool farRight   = (digitalRead(LINE_SENSORS[4]) == HIGH);

  if (centerOn) {
    setMotors(CRUISE_LEFT, CRUISE_RIGHT_ADJUST);
  } 
  else if (farRight) {
    setMotors(HARD_ESCAPE_FORWARD, HARD_ESCAPE_REVERSE);
  }
  else if (farLeft) {
    setMotors(HARD_ESCAPE_REVERSE, HARD_ESCAPE_FORWARD);
  }
  else if (midRight) {
    setMotors(CORRECTION_FAST, CORRECTION_SLOW);
  } 
  else if (midLeft) {
    setMotors(CORRECTION_SLOW, CORRECTION_FAST);
  } 
  else {
    setMotors(CRUISE_LEFT, CRUISE_RIGHT_ADJUST);
  }
}