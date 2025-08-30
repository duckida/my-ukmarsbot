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

const float circumference = 100.5f;//110;//103.9215686314;
/****/

// WALL SENSOR VALUES

const int frontWallThreshold = 68;
const int leftGapThreshold = 6;

// PID CONSTANTS
const float target = 27; // The target wall distance 29 mm

// Tuning Constants
const float Kp = 1.5  ; // The Kp value
// const float Ki = 0; // The Ki value
const float Kd = 2; // The Kd value

// Optimal battery charge - 7.2v - 7.3v
volatile int32_t encoderLeftCount;
volatile int32_t encoderRightCount;
uint32_t updateTime;
uint32_t updateInterval = 40;  // in milliseconds
const float MAX_MOTOR_VOLTS = 6.0f;
const float batteryDividerRatio = 2.0f;

// Wall Sensor data setup
/*
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


*/
// the current value of the sensors
volatile int gSensorFront;
volatile int gSensorLeft;
volatile int gSensorRight;
// true if a wall is present
volatile bool gFrontWall;
volatile bool gLeftWall;
volatile bool gRightWall;
// steering and turn position errors
volatile int gSensorFrontError;   // zero when robot in cell centre
volatile float gSensorCTE;  // zero when robot in cell centre

bool atSensingPoint = true;

// Positioning variables
int cellThreshold = 180; 
int x = 0;
int y = 0;
int direction = 0; // at start, facing up Direction 0N, 1E, 2S, 3W
// Start is 0, 0
const float multiplier = 2.3333333333;


// Gets battery voltage
float gBatteryVolts;
float getBatteryVolts() {
  int adcValue = analogRead(BATTERY_VOLTS);
  gBatteryVolts = adcValue * (5.0f * batteryDividerRatio / 1023.0f);
  return gBatteryVolts;
}

// ------------MOTORS-----------

// Initialize the pins for the motors
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

// Set speed of left motor
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

// Set speed of right motor
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

// Set speed of both motors
void setMotorPWM(int left, int right) {
  setLeftMotorPWM(left);
  setRightMotorPWM(right);
}

// Set voltage of left motor
void setLeftMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setLeftMotorPWM(motorPWM);
}

// Set voltage of right motor
void setRightMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setRightMotorPWM(motorPWM);
}

void setMotorVolts(float left, float right) {
  setLeftMotorVolts(left);
  setRightMotorVolts(right);
}

// ------------SETUP-----------

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
  // set the timer frequency to 500ms
  OCR2A = 249;  // (16000000/128/500)-1 = 249 
  // enable the timer interrupt
  bitSet(TIMSK2, OCIE2A);
}

// ------------ENCODERS-----------

// Sets up the encoders
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


// Left encoder interrupt
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
 
// Right encoder interrupt
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

// ------------WALL SENSOR READING-----------

// Read sensors
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

  /* gFrontWall = front > gFrontReference / 4;
  gLeftWall = left > gLeftReference / 2;
  gRightWall = right > gRightReference / 2;
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
  }*/
  // make the results available to the rest of the program
  gSensorLeft = max(0,left);
  gSensorRight = max(0,right);
  gSensorFront = max(0,front);
}

// ------------DRIVE-----------

// Drive a certain distance in mm
void driveDistance(float lSpeed, float rSpeed, float mm) {
  encoderLeftCount = 0;
  encoderRightCount = 0;
  float compensation; // The compensation value
  float finalComp; // 1 / compensation value
  float circumferences; // The number of circumferences to travel
  float counts; // The number of encoder counts to travel
  int firstStep; // Used for averages
  int encoderAvg = 0; // Average of 2 encoders

  //234 c = 113 mm
  circumferences = mm / circumference;
  counts = circumferences * 234;
  while (encoderAvg < counts) {
    if (encoderRightCount > 0 && encoderLeftCount > 0) {
      compensation = (float)encoderRightCount / (float)encoderLeftCount;
      //Serial.println(compensation);
      finalComp = 1 / compensation;
      setMotorVolts(lSpeed, rSpeed * finalComp);
    } else {
      setMotorVolts(lSpeed, rSpeed);
    }
    firstStep = encoderRightCount + encoderLeftCount;
    encoderAvg = firstStep / 2;
  }
  setMotorPWM(0, 0);
}

// Turn to an angle
void driveAngle(float lSpeed, float rSpeed, float deg) {
  encoderLeftCount = 0;
  encoderRightCount = 0;
  float compensation; // The compensation value
  float finalComp; // 1 / compensation value

  float difference = deg * 3.05; // The encoder difference to travel
  int encoderDifference = 0; // Difference of 2 encoders

  if (deg > 0) {
    while (encoderDifference < difference) {
      if (encoderRightCount > 0 && encoderLeftCount > 0) {
        compensation = (float)encoderRightCount / (float)encoderLeftCount;
        finalComp = 1 / compensation;
        setMotorVolts(-lSpeed, rSpeed * finalComp);
      } else { 
        setMotorVolts(-lSpeed, rSpeed);
        encoderDifference = encoderRightCount - encoderLeftCount;
      }
    }
  } else {
    while (encoderDifference > difference) {
      if (encoderRightCount > 0 && encoderLeftCount > 0) {
        compensation = (float)encoderRightCount / (float)encoderLeftCount;
        finalComp = 1 / compensation;
        setMotorVolts(lSpeed, -rSpeed * finalComp);
      } else { 
        setMotorVolts(lSpeed, -rSpeed);
        encoderDifference = encoderRightCount - encoderLeftCount;
      }
    }
    
  }
  setMotorPWM(0, 0);
}

// ------------WALL SENSOR USAGE and PID-----------


// float errorTotal = 0.0; // The sum of all error (for Integral)
float oldError = 0.0; // The old error (for Derivative)

// PID Controller
float PID() {
  int error = target - gSensorLeft; // Error = target value - current value

  // Proportional
  float proportional = error * Kp; // Proportional = error x tuning constant

  // Integral
  // errorTotal += error; // Add the error to the sum of errors
  // float integral = errorTotal * Ki; // Integral = sum of all errors x tuning constant

  // Derivative
  float derivative = (oldError - error) * Kd; // Derivative = previous error - current error, x tuning constant
  oldError = error; // Update the previous error

  float pidSum = proportional + derivative; // Sum the 3 values
  return pidSum;
}

// Wall sensor & Battery ISR
ISR(TIMER2_COMPA_vect) {
  updateWallSensor();
  getBatteryVolts();
}


// ------------POSITIONING-----------

void updateDirection(int change) {
  if (change < 0){
    direction += (360 + change) % 360; // subtracts from 360 and constrains using modulo
  } else {
    direction += change % 360; // constrains using modulo
  }
  direction = direction % 360;

  Serial.println("Changed Direction");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(direction);
  Serial.println();
}
int targetX = 2;
int targetY = 0;

void updatePosition() {
  switch(direction) {
      case 0:
        y += 1;
        break;
      case 90:
        x += 1;
        break;
      case 180:
        y -= 1;
        break;
      case 270:
        x -= 1;
        break;
    }
  if (x == targetX && y == targetY) {
    setMotorVolts(0, 0);
    delay(4000);
  }
  Serial.println("Changed Position");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(direction);
  Serial.println();
}

void gapOnLeft() {
  driveDistance(1, 1, 125); // go to the middle of the next cell 
  updatePosition(); // increment the position
  setMotorPWM(0, 0); // stop
  delay(200);
  driveAngle(1, 1, 94); // turn left
  updateDirection(-90); // update direction
  driveDistance(1, 1, 25); // go to the sensing point for the next cell
  cellThreshold = 100; // the distance to get to the middle is less
}

// Asks the user using Serial for an X and Y target
void askForTarget() {
  Serial.println("Choose a target");
  Serial.print("X: ");
  while (Serial.available() == 0) { }
  targetX = Serial.readStringUntil('\n').toInt();
  Serial.print(targetX);
  Serial.println();

  delay(500);
  
  Serial.print("Y: ");
  while (Serial.available() == 0) { }
  targetY = Serial.readStringUntil('\n').toInt();
  Serial.print(targetY);
  Serial.println();
}

// ------------MAIN CODE-----------

void setup() {
  Serial.begin(57600);
  pinMode(EMITTER, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  digitalWrite(EMITTER, 0);  // be sure the emitter is off
  analogueSetup();           // increase the ADC conversion speed
  motorSetup();
  setupEncoder();
  setupSystick();
  updateTime = millis() + updateInterval;

  delay(4000);

  Serial.println(R"(  __  __ _                                               
 |  \/  (_) ___ _ __ ___  _ __ ___   ___  _   _ ___  ___ 
 | |\/| | |/ __| '__/ _ \| '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | (__| | | (_) | | | | | | (_) | |_| \__ \  __/
 |_|  |_|_|\___|_|  \___/|_| |_| |_|\___/ \__,_|___/\___|
Let's solve it!)");


  // Target Input
  askForTarget();

  x = 0;
  y = 0;
}

float encoderAvg;

void loop() {

  // /*
  if (gSensorLeft <= leftGapThreshold) { // There's a gap on the left...
    gapOnLeft(); // So go left
  }
  if (gSensorFront > frontWallThreshold) { // There's a wall in front...
    Serial.println("Wall in front");
    Serial.print("Distance: ");
    encoderAvg = (encoderLeftCount + encoderRightCount) / 2; // average the encoders
    Serial.print(encoderAvg / multiplier);
    Serial.println();
    setMotorPWM(0, 0); // Stop
    driveAngle(1.0, 1.0, -90); // Turn right
    updateDirection(90); // Update direction
  }


  encoderAvg = (encoderLeftCount + encoderRightCount) / 2; // average the encoders
  if (encoderAvg >= cellThreshold * multiplier) { // If we've traveled more than 1 cell...
    setMotorVolts(0,0); // STOP!!
    updatePosition(); // Add 1 to our coordinates
    encoderLeftCount = 0; // Reset encoder counts
    encoderRightCount = 0;
    cellThreshold = 180; // Reset cell threshold.
    delay(200);
  } 

  float pid = PID() / 100;
  setMotorVolts(1.5 - pid, 1.5 + pid); // DRIVE!!
 // */

  /*
  Serial.print(gSensorLeft);
  Serial.print(" ");
  Serial.print(gSensorFront);
  Serial.print(" ");
  Serial.print(gSensorRight);
  Serial.println();
  delay(200);
  */
}
