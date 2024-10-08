//BLE Serial is for bluetooth module only
#include <Arduino.h>
#include "digitalWriteFast.h"
#include <SoftwareSerial.h> 
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

/***
 * Global variables
 */

SoftwareSerial bleSerial(0, 1); 

volatile int32_t encoderLeftCount;
volatile int32_t encoderRightCount;
uint32_t updateTime;
uint32_t updateInterval = 100;  // in milliseconds
const float MAX_MOTOR_VOLTS = 6.0f;
const float batteryDividerRatio = 2.0f;

// Wall Sensor data setup

const int FRONT_REFERENCE = 44;
// the default values for the side sensors when the robot is centred in a cell
const int LEFT_REFERENCE = 38;
const int RIGHT_REFERENCE = 49;

// the values above which, a wall is seen
const int FRONT_WALL_THRESHOLD = FRONT_REFERENCE / 20;  // minimum value to register a wall
const int LEFT_WALL_THRESHOLD = LEFT_REFERENCE / 2;     // minimum value to register a wall
const int RIGHT_WALL_THRESHOLD = RIGHT_REFERENCE / 2;   // minimum value to register a wall

// working copies of the reference values
int gFrontReference = FRONT_REFERENCE;
int gLeftReference = LEFT_REFERENCE;
int gRightReference = RIGHT_REFERENCE;
// the current value of the sensors
volatile int gSensorFront;
volatile int gSensorLeft;
volatile int gSensorRight;
// true f a wall is present
volatile bool gFrontWall;
volatile bool gLeftWall;
volatile bool gRightWall;
// steering and turn position errors
volatile int gSensorFrontError;   // zero when robot in cell centre
volatile float gSensorCTE;  // zero when robot in cell centre


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
    
    /* Serial.print(F("  Right: "));
    Serial.print(gSensorRight);
    Serial.print(F("  Front: "));
    Serial.print(gSensorFront);
    Serial.print(F("  Left: "));
    Serial.print(gSensorLeft);
    Serial.print(F("  Error: "));
    Serial.print(gSensorCTE);
    Serial.println(); // sends "\r\n"*/
    
    if (encoderRightCount > 0 && encoderLeftCount > 0) {
      compensation = (float)encoderRightCount / (float)encoderLeftCount;
      finalComp = 1 / compensation;
      setMotorVolts(lSpeed, rSpeed * finalComp);
      bleSerial.print(encoderLeftCount);
      bleSerial.print(' ');
      bleSerial.print(encoderRightCount);
      bleSerial.print(' ');
      bleSerial.print(lSpeed);
      bleSerial.print(' ');
      bleSerial.print(rSpeed);
      bleSerial.print(' ');
      bleSerial.print(finalComp);
      bleSerial.print(' ');
      bleSerial.print(rSpeed * finalComp);
      bleSerial.println();
    }
    /*if (gSensorLeft > 30) {
      compensation = (float)encoderRightCount / (float)encoderLeftCount;
      setMotorVolts(lSpeed * 5 * compensation, rSpeed);
    }*/

    setMotorVolts(lSpeed, rSpeed);
    delay(100);
  }
}

void analogueSetup() {
  // increase speed of ADC conversions to 28us each
  // by changing the clock prescaler from 128 to 16
  bitClear(ADCSRA, ADPS0);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);
}

void setupSystick() {
  // set the mode for timer 2
  bitClear(TCCR2A, WGM20);
  bitSet(TCCR2A, WGM21);
  bitClear(TCCR2B, WGM22);
  // set divisor to 128 => timer clock = 125kHz
  bitSet(TCCR2B, CS22);
  bitClear(TCCR2B, CS21);
  bitSet(TCCR2B, CS20);
  // set the timer frequency
  OCR2A = 249;  // (16000000/128/500)-1 = 249
  // enable the timer interrupt
  bitSet(TIMSK2, OCIE2A);
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

void updateWallSensor() {
  // first read them dark
  int right = analogRead(A0);
  int front = analogRead(A1);
  int left = analogRead(A2);
  // light them up
  digitalWrite(EMITTER, 1);
  // wait until all the detectors are stable
  delayMicroseconds(50);
  // now find the differences
  right = analogRead(A0) - right;
  front = analogRead(A1) - front;
  left = analogRead(A2) - left;
  // and go dark again.
  digitalWrite(EMITTER, 0);

  gFrontWall = front > gFrontReference / 4;
  gLeftWall = left > gLeftReference / 2;
  gRightWall = right > gRightReference / 2;
  digitalWrite(LED_LEFT, gLeftWall);
  digitalWrite(LED_RIGHT, gRightWall);
  digitalWrite(LED_BUILTIN, gFrontWall);
  // calculate the alignment error - too far right is negative
  if ((left + right) > (gLeftReference + gRightReference) / 4) {
    if (left > right) {
      gSensorCTE = (left - LEFT_REFERENCE);
      gSensorCTE /= left;
    } else {
      gSensorCTE = (RIGHT_REFERENCE - right);
      gSensorCTE /= right;
    }
  } else {
    gSensorCTE = 0;
  }
  // make the results available to the rest of the program
  gSensorLeft= left;
  gSensorRight = right;
  gSensorFront = front;
}

ISR(TIMER2_COMPA_vect) {
  updateWallSensor();
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
  bleSerial.begin(9600);
  Serial.begin(9600);
  Serial.println(F("Hello\n"));
  pinMode(EMITTER, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  digitalWrite(EMITTER, 0);  // be sure the emitter is off
  motorSetup();
  setupEncoder();
  // analogueSetup();           // increase the ADC conversion speed
  setupSystick();
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
  //encoderLeftCount = 0;
  //encoderRightCount = 0;
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
    runRobot();
  }
}
