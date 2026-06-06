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
const int LINE_SENSORS[5] = {34, 36, 38, 40, 42};

// Front Ultrasonic Sensor Configuration
const int TRIG_FRONT = 52;
const int ECHO_FRONT = 53;
const int FRONT_DISTANCE_THRESHOLD = 10; 

// ==========================================
// SPEED CONFIGURATIONS
// ==========================================
const int BASE_SPEED = 185;                  
const int BACK_RIGHT_SPEED = BASE_SPEED + 60; 

const int SOFT_LOW_SPEED = BASE_SPEED - 90;  
const int RECOVERY_SPIN_SPEED = 160;         

// Asymmetric Backup Speeds 
const int BACKUP_SPEED_FAST = 170; 
const int BACKUP_SPEED_SLOW = 120;  

// Tuning parameters for extreme sharp turns
const int SHARP_MAX_SPEED    = 255;         // Max speed (full throttle) for turning power
const int SHARP_FORWARD_SPEED = 230;         
const int SHARP_REVERSE_SPEED = 140;         

// Junction Turn Settings (Counter-Clockwise Right Turn)
const int JUNCTION_CCW_FAST = 240; 
const int JUNCTION_CCW_SLOW = 0; 
const int TURN_DURATION     = 3000; 

enum RobotState { STATE_S, STATE_F, STATE_R, STATE_L, STATE_SR, STATE_SL, STATE_RECOVERY };
RobotState currentState = STATE_S; 

enum DirectionMemory { DIR_NONE, DIR_LEFT, DIR_RIGHT };
DirectionMemory lastDirection = DIR_NONE; 

void setup() {
  Serial.begin(9600);
  Serial.println(F("===================================="));
  Serial.println(F(" LFR: MAX TUNED RIGHT POWER v9      "));
  Serial.println(F("===================================="));

  pinMode(F_PWMA, OUTPUT); pinMode(F_PWMB, OUTPUT);
  pinMode(F_INA1, OUTPUT); pinMode(F_INA2, OUTPUT);
  pinMode(F_INB1, OUTPUT); pinMode(F_INB2, OUTPUT);
  pinMode(F_STBY, OUTPUT);
  pinMode(B_PWMA, OUTPUT); pinMode(B_PWMB, OUTPUT);
  pinMode(B_INA1, OUTPUT); pinMode(B_INA2, OUTPUT);
  pinMode(B_INB1, OUTPUT); pinMode(B_INB2, OUTPUT);
  pinMode(B_STBY, OUTPUT);

  digitalWrite(F_STBY, HIGH);
  digitalWrite(B_STBY, HIGH);

  for (int i = 0; i < 5; i++) {
    pinMode(LINE_SENSORS[i], INPUT);
  }

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);
  while (digitalRead(START_BUTTON) == HIGH) {}
  delay(1000); 
}

void loop() {
  // 1. SAFETY CHECK
  int distance = getDistance();
  if (distance > 0 && distance <= FRONT_DISTANCE_THRESHOLD) {
    stopMotors(); 
    return; 
  }

  // 2. JUNCTION CHECK
  if (allSensorsOnBlack()) {
    Serial.println(F("-> [JUNCTION] Triggering Counter-Clockwise Pivot"));
    executeJunctionCounterClockwise();
    return; 
  }

  // 3. FSM PATTERN ANALYSIS (0 = Black Line, 1 = White Floor)
  String sensorPattern = readSensors();
  Serial.print(F("Sensors: ")); Serial.print(sensorPattern);

  String stateLabel = "";
  
  if (sensorPattern == "11111") { 
    currentState = STATE_RECOVERY;
    stateLabel = "RECOVERY SEEKING";
  } 
  else if (sensorPattern == "11011") { 
    currentState = STATE_F; 
    stateLabel = "FORWARD (F)";
  } 
  else if (sensorPattern == "00111" || sensorPattern == "01111" || sensorPattern == "10000") { 
    currentState = STATE_R; 
    lastDirection = DIR_RIGHT; 
    stateLabel = "SHARP RIGHT (R)";
  } 
  else if (sensorPattern == "11100" || sensorPattern == "11110") { 
    currentState = STATE_L; 
    lastDirection = DIR_LEFT;  
    stateLabel = "SHARP LEFT (L)";
  } 
  else if (sensorPattern == "00011" || sensorPattern == "10011" || sensorPattern == "11101") { 
    currentState = STATE_SR; 
    lastDirection = DIR_RIGHT; 
    stateLabel = "SOFT RIGHT (SR)";
  } 
  else if (sensorPattern == "11001" || sensorPattern == "10111") { 
    currentState = STATE_SL; 
    lastDirection = DIR_LEFT;  
    stateLabel = "SOFT LEFT (SL)";
  }
  else {
    stateLabel = "TRACKING (Retaining State)";
  }

  Serial.print(F(" | State: ")); Serial.println(stateLabel);

  // 4. MOTOR DRIVER EXECUTION
  switch (currentState) {
    case STATE_F:  
      setMotorSpeeds(BASE_SPEED, BASE_SPEED); 
      break;

    case STATE_R:  
      // UPDATED: Left motors pushed to SHARP_MAX_SPEED (255) for maximum forward driving force
      executePivotMotors(HIGH, LOW, LOW, HIGH, SHARP_MAX_SPEED, SHARP_REVERSE_SPEED);
      break;

    case STATE_L:  
      executePivotMotors(LOW, HIGH, HIGH, LOW, SHARP_REVERSE_SPEED, SHARP_FORWARD_SPEED);
      break;

    case STATE_SR: 
      setMotorSpeeds(BASE_SPEED, SOFT_LOW_SPEED); 
      break;

    case STATE_SL: 
      setMotorSpeeds(SOFT_LOW_SPEED, BASE_SPEED); 
      break;

    case STATE_RECOVERY:
      if (lastDirection == DIR_LEFT) {
        executePivotMotors(HIGH, LOW, LOW, HIGH, RECOVERY_SPIN_SPEED, RECOVERY_SPIN_SPEED);
      } 
      else if (lastDirection == DIR_RIGHT) {
        executePivotMotors(LOW, HIGH, HIGH, LOW, RECOVERY_SPIN_SPEED, RECOVERY_SPIN_SPEED);
      } 
      else {
        stopMotors();
      }
      break;

    case STATE_S:  
    default:       
      stopMotors(); 
      break;
  }
  
  delay(10); 
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

int getDistance() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  long duration = pulseIn(ECHO_FRONT, HIGH, 20000); 
  return duration / 58;
}

String readSensors() {
  String pattern = "";
  for (int i = 0; i < 5; i++) {
    pattern += String(digitalRead(LINE_SENSORS[i]));
  }
  return pattern;
}

bool allSensorsOnBlack() {
  for (int i = 0; i < 5; i++) {
    if (digitalRead(LINE_SENSORS[i]) == HIGH) return false; 
  }
  return true; 
}

void executeJunctionCounterClockwise() {
  stopMotors();
  delay(400); 

  // Left Side BACKWARD (Slow)
  digitalWrite(F_INA1, LOW);  digitalWrite(F_INA2, HIGH); analogWrite(F_PWMA, BACKUP_SPEED_FAST);
  digitalWrite(B_INA1, LOW);  digitalWrite(B_INA2, HIGH); analogWrite(B_PWMA, BACKUP_SPEED_FAST);
  
  // Right Side BACKWARD (Fast)
  digitalWrite(F_INB1, LOW);  digitalWrite(F_INB2, HIGH); analogWrite(F_PWMB, BACKUP_SPEED_SLOW);
  digitalWrite(B_INB1, LOW);  digitalWrite(B_INB2, HIGH); analogWrite(B_PWMB, BACKUP_SPEED_SLOW);

  unsigned long safetyTimeout = millis();
  while (!allSensorsOnBlack()) {
    if (millis() - safetyTimeout > 1000) {
      break; 
    }
  }

  delay(200); 
  stopMotors();
  delay(200);

  // Counter-Clockwise Turn Execution (Left Side Backward, Right Side Forward)
  executePivotMotors(LOW, HIGH, HIGH, LOW, JUNCTION_CCW_SLOW, JUNCTION_CCW_FAST);
  delay(TURN_DURATION); 
  
  stopMotors();
  delay(150);
  
  lastDirection = DIR_RIGHT; 
  currentState = STATE_F; 
}

void executePivotMotors(int ina1, int ina2, int inb1, int inb2, int leftSpeed, int rightSpeed) {
  digitalWrite(F_INA1, ina1);  digitalWrite(F_INA2, ina2);  analogWrite(F_PWMA, leftSpeed);
  digitalWrite(B_INA1, ina1);  digitalWrite(B_INA2, ina2);  analogWrite(B_PWMA, leftSpeed);
  digitalWrite(F_INB1, inb1);  digitalWrite(F_INB2, inb2);  analogWrite(F_PWMB, rightSpeed);
  digitalWrite(B_INB1, inb1);  digitalWrite(B_INB2, inb2);  analogWrite(B_PWMB, rightSpeed);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(F_INA1, HIGH);  digitalWrite(F_INA2, LOW);   analogWrite(F_PWMA, leftSpeed);
  digitalWrite(B_INA1, HIGH);  digitalWrite(B_INA2, LOW);   analogWrite(B_PWMA, leftSpeed);
  digitalWrite(F_INB1, HIGH);  digitalWrite(F_INB2, LOW);   analogWrite(F_PWMB, rightSpeed);
  
  int actualBackRightSpeed = (rightSpeed == 0) ? 0 : BACK_RIGHT_SPEED;
  actualBackRightSpeed = constrain(actualBackRightSpeed, 0, 255);
  digitalWrite(B_INB1, HIGH);  digitalWrite(B_INB2, LOW);   analogWrite(B_PWMB, actualBackRightSpeed);
}

void stopMotors() {
  executePivotMotors(LOW, LOW, LOW, LOW, 0, 0);
}