#include <Arduino.h>
#include "digitalWriteFast.h"
/**
 * Hardware pin defines
 */
#define BOARD UKMARSBOT_V1
const int ENCODER_LEFT_CLK = 2;
const int ENCODER_RIGHT_CLK = 3;
const int ENCODER_LEFT_B = 4;
const int ENCODER_RIGHT_B = 5;
const int MOTOR_LEFT_DIR = 7;
const int MOTOR_RIGHT_DIR = 8;
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;
const int LED_RIGHT = 6;
const int LED_LEFT = 11;
const int EMITTER = 12;
const int SENSOR_RIGHT_MARK = A0;
const int SENSOR_1 = A1;
const int SENSOR_2 = A2;
const int SENSOR_3 = A3;
const int SENSOR_4 = A4;
const int SENSOR_LEFT_MARK = A5;
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;
/****/

// Wheel Circumference = 100.5309649149
// Gear Ratio = 20
// Shaft pulses = 12

// 12 x 20 = 1 wheel rotation = 240
// 240 encoder counts = 100.5 mm driving!

// In my testing it's really 113 mm!

/***
 * Global variables
 */

volatile int32_t encoderLeftCount;
volatile int32_t encoderRightCount;
uint32_t updateTime;
uint32_t updateInterval = 100;  // in milliseconds
const float MAX_MOTOR_VOLTS = 6.0f;
const float batteryDividerRatio = 2.0f;

float gBatteryVolts;
float getBatteryVolts() {
  int adcValue = analogRead(BATTERY_VOLTS);
  gBatteryVolts = adcValue * (5.0f * batteryDividerRatio / 1023.0f);
  return gBatteryVolts;
}

int decodeFunctionSwitch(int functionValue) {
  /**
   * Typical ADC values for all function switch settings
   */
  const int adcReading[] = {660, 647, 630, 614, 590, 570, 545, 522, 461,
                            429, 385, 343, 271, 212, 128, 44,  0};

  if (functionValue > 1000) {
    return 16;  // pushbutton closed
  }
  int result = 16;
  for (int i = 0; i < 16; i++) {
    if (functionValue > (adcReading[i] + adcReading[i + 1]) / 2) {
      result = i;
      break;
    }
  }
  return result;
}

int getFunctionSwitch() {
  int functionValue = analogRead(FUNCTION_PIN);
  int function = decodeFunctionSwitch(functionValue);
  return function;
}

void motorSetup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  digitalWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR, 0);
  digitalWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_DIR, 0);
}

void setLeftMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_LEFT_DIR, 1);
    analogWrite(MOTOR_LEFT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, 0);
    analogWrite(MOTOR_LEFT_PWM, pwm);
  }
}

void setRightMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_RIGHT_DIR, 0);
    analogWrite(MOTOR_RIGHT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, 1);
    analogWrite(MOTOR_RIGHT_PWM, pwm);
  }
}

void setMotorPWM(int left, int right) {
  setLeftMotorPWM(left);
  setRightMotorPWM(right);
}

void setLeftMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setLeftMotorPWM(motorPWM);
}

void setRightMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setRightMotorPWM(motorPWM);
}



void setMotorVolts(float left, float right) {
  setLeftMotorVolts(left);
  setRightMotorVolts(right);
}

void drive(float lSpeed, float rSpeed) {
  Serial.println("Drive Function");
  uint32_t endTime = millis() + 2000;
  float compensation = 0;
  float finalComp = 0;
  while (endTime > millis()) {
    Serial.print(encoderLeftCount);
    Serial.print(' ');
    Serial.print(encoderRightCount);
    Serial.print(' ');
    Serial.print(finalComp);
    Serial.println(); 
    if (encoderRightCount > 0 && encoderLeftCount > 0) {
      compensation = encoderRightCount / encoderLeftCount;
      //Serial.println(compensation);
      finalComp = 1 / compensation;
      setMotorVolts(lSpeed, rSpeed * finalComp);
    }
    setMotorVolts(lSpeed, rSpeed);
    delay(100);
  }
}

void driveDistance(float lSpeed, float rSpeed, float mm) {
  Serial.println("Drive Distance Function");
  float compensation; // The compensation value
  float finalComp; // 1 / compensation value
  float circumferences; // The number of circumferences to travel
  float counts; // The number of encoder counts to travel
  int firstStep; // Used for averages
  int encoderAvg = 0; // Average of 2 encoders

  //234 c = 113 mm
  circumferences = mm / 113;
  counts = circumferences * 234;
  while (encoderAvg < counts) {
    Serial.print(encoderLeftCount);
    Serial.print(' ');
    Serial.print(encoderRightCount);
    Serial.print(' ');
    Serial.print(finalComp);
    Serial.println(); 
    if (encoderRightCount > 0 && encoderLeftCount > 0) {
      compensation = (float)encoderRightCount / (float)encoderLeftCount;
      //Serial.println(compensation);
      finalComp = 1 / compensation;
      setMotorVolts(lSpeed, rSpeed * finalComp * 0.93);
    }
    setMotorVolts(lSpeed, rSpeed);
    firstStep = encoderRightCount + encoderLeftCount;
    encoderAvg = firstStep / 2;
  }
  setMotorPWM(0, 0);
}


void setupEncoder() {
    // left
  pinMode(ENCODER_LEFT_CLK, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);
  // configure the pin change
  bitClear(EICRA, ISC01);
  bitSet(EICRA, ISC00);
  // enable the interrupt
  bitSet(EIMSK, INT0);
  encoderLeftCount = 0;
  // right
  pinMode(ENCODER_RIGHT_CLK, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  // configure the pin change
  bitClear(EICRA, ISC11);
  bitSet(EICRA, ISC10);
  // enable the interrupt
  bitSet(EIMSK, INT1);
  encoderRightCount = 0;
}

ISR(INT0_vect) {
  static bool oldB = 0;
  bool newB = bool(digitalReadFast(ENCODER_LEFT_B));
  bool newA = bool(digitalReadFast(ENCODER_LEFT_CLK)) ^ newB;
  if (newA == oldB) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
  oldB = newB;
}

ISR(INT1_vect) {
  static bool oldB = 0;
  bool newB = bool(digitalReadFast(ENCODER_RIGHT_B));
  bool newA = bool(digitalReadFast(ENCODER_RIGHT_CLK)) ^ newB;
  if (newA == oldB) {
    encoderRightCount--;
  } else {
    encoderRightCount++;
  }
  oldB = newB;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Hello\n"));
  motorSetup();
  setupEncoder();
  updateTime = millis() + updateInterval;
}

void motorAction(int function) {
  switch (function) {
    case 0:
      drive(0, 0);
      Serial.println("motors off");
      break;
    case 1:
      drive(1.5, 1.5);
      Serial.println("forward 25%");
      break;
    case 2:
      drive(3.0, 3.0);
      Serial.println("forward 50%");
      break;
    case 3:
      drive(4.5, 4.5);
      Serial.println("forward 75%");
      break;
    case 4:
      drive(-1.5, -1.5);
      Serial.println("reverse 25%");
      break;
    case 5:
      drive(-3.0, -3.0);
      Serial.println("reverse 50%");
      break;
    case 6:
      drive(-4.5, -4.5);
      Serial.println("reverse 75%");
      break;
    case 7:
      drive(-1.5, 1.5);
      Serial.println("spin left 25%");
      break;
    case 8:
      drive(-3.0, 3.0);
      Serial.println("spin left 50%");
      break;
    case 9:
      drive(1.5, -1.5);
      Serial.println("spin right 25%");
      break;
    case 10:
      drive(3.0, 3.0);
      Serial.println("spin right 50%");
      break;
    case 11:
      drive(0, 1.5);
      Serial.println("pivot left 25%");
      break;
    case 12:
      drive(1.5, 0);
      Serial.println("pivot right 25%");
      break;
    case 13:
      drive(1.5, 3.0);
      Serial.println("curve left");
      break;
    case 14:
      drive(3.0, 1.5);
      Serial.println("curve right");
      break;
    case 15:
      drive(4.5, 3.0);
      Serial.println("big curve right");
      break;
    default:
      drive(0, 0);
      break;
  }
}

void runRobot() {
  int function = getFunctionSwitch();
  // run the motors for a fixed amount of time (in milliseconds)
  getBatteryVolts();  // update the battery reading
  motorAction(function);
  /* while (endTime > millis()) {
    if (getFunctionSwitch() == 16) {
      break;  // stop running if the button is pressed
    }
  } */
  // be sure to turn off the motors
  setMotorPWM(0, 0);
  // a short delay to let everything come to rest.
  delay(500);
}

void runDistance() {
  getBatteryVolts();
  driveDistance(1.5,  1.5, 100); // Speed 1.5, 100 mm
}


void loop() {
  if (getFunctionSwitch() == 16) {
    // button is pressed so wait until it is released
    while (getFunctionSwitch() == 16) {
      delay(20);
    }
    // now wait for the user to get clear
    Serial.println("Starting");
    encoderLeftCount = 0;
    encoderRightCount = 0;
    delay(500);
    //runRobot();
    runDistance();
  }
}
