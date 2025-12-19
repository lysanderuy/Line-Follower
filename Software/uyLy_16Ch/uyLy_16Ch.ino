#include <SparkFun_TB6612.h>
#include <LSUSensor.h>

// LSU-16A Sensor
LSUSensor lsu;

// TB6612FNG Motor Driver
#define PWMA 11
#define PWMB 3
#define AIN2 10
#define AIN1 9
#define BIN1 8
#define BIN2 7
#define STBY 6
Motor leftMotor = Motor(BIN1, BIN2, PWMB, 1, STBY);
Motor rightMotor = Motor(AIN2, AIN1, PWMA, 1, STBY);

// PD Controller variables
int centerPosition = 7500;
float targetSpeed = 0.0;
float kp = 0.0;
float kd = 0.0;
int proportional = 0;
int derivative = 0;
int previousProportional;

// Built-in LED variables
const int ledPin = LED_BUILTIN;
int ledState = LOW;

// Start switch/button
#define BUTTON 2
#define SWITCH 4

//=================================================vvv SETUP vvv=================================================
void setup() {
  Serial.begin(9600);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(SWITCH, INPUT_PULLUP);

  lsu.begin();

  lsu.setThreshold(500);

  lsu.calibrate();
  blinkLED();

  //N30 4KRPM
  //targetSpeed = 60.0;
  //kp = 0.0295;
  //kd = 0.15;
  //targetSpeed = 80.0;
  //kp = 0.0445;
  //kd = 0.24;
  //targetSpeed = 100.0;
  //kp = 0.0537;
  //kd = 0.35;

  // if (digitalRead(SWITCH) == 0) {
  //   targetSpeed = 255.0;
  //   kp = 0.06;
  //   kd = 0.3;
  // } else {
  //   targetSpeed = 70.0;
  //   kp = 0.04;
  //   kd = 0.3;
  // }

  while (digitalRead(BUTTON) == 1) {
  }
}
//=================================================^^^ SETUP ^^^=================================================

//=================================================vvv MAIN LOOP vvv=================================================
void loop() {
  // Maze Solving
  // trackTime(255, .06, .3, 2000); 
  trackCrossL(75, .035, .2, 'l');
  trackCrossC(75, .035, .2, 'l');
  trackCrossR(75, .035, .2, 'r');
  trackCrossC(75, .035, .2, 'r');
  trackCrossC(75, .035, .2, 'l');
  trackCrossC(120, .035, .2, 'p');
  trackCrossC(75, .035, .2, 'l');
  trackCrossC(75, .035, .2, 'l');
  trackCrossC(120, .035, .2, 'p');
  trackCrossC(75, .035, .2, 'l');
  trackCrossC(75, .035, .2, 'l');
  trackCrossC(120, .035, .2, 'p');
  trackCrossC(120, .035, .2, 'p');
  trackCrossC(75, .035, .2, 'l');
  trackCrossR(75, .035, .2, 'r');
  trackCrossR(75, .035, .2, 'r');
  trackCrossL(75, .035, .2, 'l');
  trackCrossC(120, .035, .2, 'p');

  // trackCrossR(75, .035, .2, 'p');
  // trackCrossR(75, .035, .2, 'l');
  // trackCrossR(75, .035, .2, 'r');
  // trackCrossL(75, .035, .2, 'p');
  // trackCrossL(75, .035, .2, 'l');
  // trackCrossL(75, .035, .2, 'r');

  stop();
  while(1){}
}
//=================================================^^^ MAIN LOOP ^^^=================================================

//============================================vvv CONTROLLER FUNCTIONS vvv============================================
void pidControl(int targetSpeed, float kp, float kd) {
  float filteredPosition = lsu.position();
  proportional = filteredPosition - centerPosition;
  derivative = proportional - previousProportional;
  previousProportional = proportional;

  int PID = (proportional * kp) + (derivative * kd);

  int leftSpeed = constrain((targetSpeed + PID), -targetSpeed, targetSpeed);
  int rightSpeed = constrain((targetSpeed - PID), -targetSpeed, targetSpeed);

  leftMotor.drive(leftSpeed);
  rightMotor.drive(rightSpeed);
}
//============================================^^^ CONTROLLER FUNCTIONS ^^^============================================

//============================================vvv MAZE SOLVING FUNCTIONS vvv============================================
void trackTime(int targetSpeed, float kp, float kd, int duration) {
  unsigned long start = millis();
  while (millis() - start <=  duration) {
    lsu.readSensors();
    lsu.readLine();
    pidControl(targetSpeed, kp, kd);
  } 
}

void trackCrossC (int targetSpeed, float kp, float kd, char direction) {
  do {
    lsu.readSensors();
    lsu.readLine();
    pidControl(targetSpeed, kp, kd);
  } while (!lsu.middleAllBlack());
  
  leftMotor.drive(targetSpeed);
  rightMotor.drive(targetSpeed);
  delay (250);

  if (direction == 'p') {
    return;
  } 
  else if (direction == 'l') {
    leftMotor.drive(-targetSpeed);
    rightMotor.drive(targetSpeed);
    delay(70);
    do {
      lsu.readSensors();
      lsu.readLine();
      leftMotor.drive(-targetSpeed);
      rightMotor.drive(targetSpeed);
    } while (lsu.normalized(6) < lsu.threshold() || lsu.normalized(7) < lsu.threshold() || lsu.normalized(8) < lsu.threshold() || lsu.normalized(9) < lsu.threshold());
  } 
  else if (direction == 'r') {
    leftMotor.drive(targetSpeed);
    rightMotor.drive(-targetSpeed);
    delay(70);
    do {
      lsu.readSensors();
      lsu.readLine();
      leftMotor.drive(targetSpeed);
      rightMotor.drive(-targetSpeed);
    } while (lsu.normalized(6) < lsu.threshold() || lsu.normalized(7) < lsu.threshold() || lsu.normalized(8) < lsu.threshold() || lsu.normalized(9) < lsu.threshold());
  }
  
  lsu.readSensors();
}

void trackCrossL (int targetSpeed, float kp, float kd, char direction) {
  do {
    lsu.readSensors();
    lsu.readLine();
    pidControl(targetSpeed, kp, kd);
  } while (!lsu.leftAllBlack());
  
  leftMotor.drive(targetSpeed);
  rightMotor.drive(targetSpeed);
  delay (250);
  
  if (direction == 'p') {
    return;
  } 
  else if (direction == 'l') {
    leftMotor.drive(-targetSpeed);
    rightMotor.drive(targetSpeed);
    delay(70);
    do {
      lsu.readSensors();
      lsu.readLine();
      leftMotor.drive(-targetSpeed);
      rightMotor.drive(targetSpeed);
    } while (lsu.normalized(6) < lsu.threshold() || lsu.normalized(7) < lsu.threshold() || lsu.normalized(8) < lsu.threshold() || lsu.normalized(9) < lsu.threshold());
  } 
  else if (direction == 'r') {
    leftMotor.drive(targetSpeed);
    rightMotor.drive(-targetSpeed);
    delay(70);
    do {
      lsu.readSensors();
      lsu.readLine();
      leftMotor.drive(targetSpeed);
      rightMotor.drive(-targetSpeed);
    } while (lsu.normalized(6) < lsu.threshold() || lsu.normalized(7) < lsu.threshold() || lsu.normalized(8) < lsu.threshold() || lsu.normalized(9) < lsu.threshold());
  }
  
  lsu.readSensors();
}

void trackCrossR (int targetSpeed, float kp, float kd, char direction) {
  do {
    lsu.readSensors();
    lsu.readLine();
    pidControl(targetSpeed, kp, kd);
  } while (!lsu.rightAllBlack());
  
  leftMotor.drive(targetSpeed);
  rightMotor.drive(targetSpeed);
  delay (250);
  
  if (direction == 'p') {
    return;
  } 
  else if (direction == 'l') {
    leftMotor.drive(-targetSpeed);
    rightMotor.drive(targetSpeed);
    delay(70);
    do {
      lsu.readSensors();
      lsu.readLine();
      leftMotor.drive(-targetSpeed);
      rightMotor.drive(targetSpeed);
    } while (lsu.normalized(6) < lsu.threshold() || lsu.normalized(7) < lsu.threshold() || lsu.normalized(8) < lsu.threshold() || lsu.normalized(9) < lsu.threshold());
  } 
  else if (direction == 'r') {
    leftMotor.drive(targetSpeed);
    rightMotor.drive(-targetSpeed);
    delay(70);
    do {
      lsu.readSensors();
      lsu.readLine();
      leftMotor.drive(targetSpeed);
      rightMotor.drive(-targetSpeed);
    } while (lsu.normalized(6) < lsu.threshold() || lsu.normalized(7) < lsu.threshold() || lsu.normalized(8) < lsu.threshold() || lsu.normalized(9) < lsu.threshold());
  }

  lsu.readSensors();
}
//============================================^^^ MAZE SOLVING FUNCTIONS ^^^============================================

//================================================vvv MOTOR FUNCTIONS vvv================================================
void stop() {
  leftMotor.drive(0);
  rightMotor.drive(0);
}
//================================================^^^ MOTOR FUNCTIONS ^^^================================================

//=============================================vvv BUILT-IN LED FUNCTIONS vvv=============================================
void blinkLED() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}
//=============================================^^^ BUILT-IN LED FUNCTIONS ^^^=============================================
