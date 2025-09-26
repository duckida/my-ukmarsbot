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
const int leftGapThreshold = 14;
const int rightGapThreshold = 12;

// PID CONSTANTS
const float target = 26; // The target wall distance 29 mm

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

// FLOODFILL - WALL STORAGE
// Define maze dimensions
const int MAZE_WIDTH = 2;
const int MAZE_HEIGHT = 2;
const int QUEUE_SIZE = MAZE_WIDTH * MAZE_HEIGHT;

const byte NORTH = 0;  // bit 0 (value 1)
const byte EAST  = 1;  // bit 1 (value 2)
const byte SOUTH = 2;  // bit 2 (value 4)
const byte WEST  = 3;  // bit 3 (value 8)


const int GOAL_X = 1;
const int GOAL_Y = 1;

struct Cell {
  uint8_t x;
  uint8_t y;
  uint8_t cost;

  byte walls;
};

struct QueueObject {
  uint8_t x;
  uint8_t y;
};

Cell maze[MAZE_HEIGHT][MAZE_WIDTH];

bool positionUpdated = false;
bool mazeUpdated = false;

void setWall(int x, int y, int dir) {
  if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return;

  if (dir == 0) { // North
    maze[y][x].walls |= (1 << NORTH);  // Set North wall
    if (y + 1 < MAZE_HEIGHT) maze[y+1][x].walls |= (1 << SOUTH);
  }
  else if (dir == 90) { // East
    maze[y][x].walls |= (1 << EAST);
    if (x + 1 < MAZE_WIDTH) maze[y][x+1].walls |= (1 << WEST);
  }
  else if (dir == 180) { // South
    maze[y][x].walls |= (1 << SOUTH);
    if (y - 1 >= 0) maze[y-1][x].walls |= (1 << NORTH);
  }
  else if (dir == 270) { // West
    maze[y][x].walls |= (1 << WEST);
    if (x - 1 >= 0) maze[y][x-1].walls |= (1 << EAST);
  }
  mazeUpdated = true;
}

// Check if a wall exists in that direction
bool hasWall(int x, int y, int dir) {
  if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return true; // out of bounds = wall

  if (dir == 0)      return maze[y][x].walls & (1 << NORTH);
  else if (dir == 90)  return maze[y][x].walls & (1 << EAST);
  else if (dir == 180) return maze[y][x].walls & (1 << SOUTH);
  else if (dir == 270) return maze[y][x].walls & (1 << WEST);

  return true; // invalid dir treated as wall
}

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
      }
      encoderDifference = encoderRightCount - encoderLeftCount;
    }

  } else {
    while (encoderDifference > difference) {
      if (encoderRightCount > 0 && encoderLeftCount > 0) {
        compensation = (float)encoderRightCount / (float)encoderLeftCount;
        finalComp = 1 / compensation;
        setMotorVolts(lSpeed, -rSpeed * finalComp);
      } else {
        setMotorVolts(lSpeed, -rSpeed);
      }
      encoderDifference = encoderRightCount - encoderLeftCount;
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

int calculateDirection(int change) {
  int mydirection = direction;
  if (change < 0){
    mydirection += (360 + change) % 360; // subtracts from 360 and constrains using modulo
  } else {
    mydirection += change % 360; // constrains using modulo
  }
  mydirection = mydirection % 360;
  return mydirection;
}

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
  positionUpdated = true;
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


Cell goal = {GOAL_X, GOAL_Y, 0, 0};

// Queue for the floodfill
QueueObject queue[QUEUE_SIZE];
int head = 0;
int tail = 0;

// Adds a cell to the queue
void enqueue(Cell cell) {
  // Check if the queue is full before adding
  if (((tail + 1) % QUEUE_SIZE) == head) {
    return; // Queue is full, can't add more
  }
  queue[tail].x = cell.x;
  queue[tail].y = cell.y;
  tail = (tail + 1) % QUEUE_SIZE;
}

// Gets the next cell from the queue
Cell dequeue() {
  Cell cell;
  cell = maze[queue[head].y][queue[head].x];
  head = (head + 1) % QUEUE_SIZE;
  return cell;
}

// Checks if the queue is empty
bool isQueueEmpty() {
  return head == tail;
}

int currentWallCheckDirections[4] = {0, 90, 180, 270};

void calculateFlood() {
  head = tail = 0;
  // Set them all to a high value
  for (int i = 0; i < MAZE_HEIGHT; i++) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      maze[i][j].cost = 254;
      maze[i][j].x = j;
      maze[i][j].y = i;
    }
  }
  maze[goal.y][goal.x].cost = 0;
  enqueue(maze[goal.y][goal.x]);

  while (isQueueEmpty() == false) {
    Cell currentCell = dequeue();
    int neighborWallCheckDirections[4] = {180, 270, 0, 90};


    for (int n = 0; n < 4; n++) {
      Cell neighbor;
      if (n == 0) { // North
        neighbor.x = currentCell.x;
        neighbor.y = currentCell.y + 1;
      } else if (n == 1) { // East
        neighbor.x = currentCell.x + 1;
        neighbor.y = currentCell.y;
      } else if (n == 2) { // South
        neighbor.x = currentCell.x;
        neighbor.y = currentCell.y - 1;
      } else if (n == 3) { // West
        neighbor.x = currentCell.x - 1;
        neighbor.y = currentCell.y;
      }
      if (neighbor.x > -1 && neighbor.x < MAZE_WIDTH && neighbor.y > -1 && neighbor.y < MAZE_HEIGHT) { // it's valid
        if (!(hasWall(neighbor.x, neighbor.y, neighborWallCheckDirections[n]))) {
          if (maze[neighbor.y][neighbor.x].cost > currentCell.cost + 1) {
            maze[neighbor.y][neighbor.x].cost = currentCell.cost + 1;
            Cell newcell;
            newcell.x = neighbor.x;
            newcell.y = neighbor.y;
            newcell.cost = currentCell.cost + 1;
            enqueue(newcell);
          }
        }
      }
    }
  }
}

void printSensors() {
  Serial.print(gSensorLeft);
  Serial.print(" ");
  Serial.print(gSensorFront);
  Serial.print(" ");
  Serial.print(gSensorRight);
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

  x = 0;
  y = 0;
}

float encoderAvg;

bool leftWall = false;
bool rightWall = false;
bool frontWall = false;

void loop() {
  if (gSensorLeft <= leftGapThreshold) { // There's a gap on the left...
    leftWall = false;
  } else {
    leftWall = true;
    setWall(x, y, calculateDirection(270));
  }
  if (gSensorRight <= rightGapThreshold) { // There's a gap on the left...
    rightWall = false;
  } else {
    rightWall = true;
    setWall(x, y, calculateDirection(90));
  }
  if (gSensorFront > frontWallThreshold) { // There's a wall in front...
    frontWall = true;
    setWall(x, y, direction);
  } else {
    frontWall = false;
  }

    // --- Flood update only when needed ---
  if (mazeUpdated || positionUpdated) {
    calculateFlood();
    mazeUpdated = false;
    positionUpdated = false;
  }


  int lowestCost = 254;
  int lowestCostCell = 254;

  Cell currentCell = maze[y][x];



  for (int n = 0; n < 4; n++) {
      Cell neighbor;
      int cellId;
      if (n == 0) { // North
        neighbor.x = currentCell.x;
        neighbor.y = currentCell.y + 1;
        cellId = 0;
      } else if (n == 1) { // East
        neighbor.x = currentCell.x + 1;
        neighbor.y = currentCell.y;
        cellId = 90;
      } else if (n == 2) { // South
        neighbor.x = currentCell.x;
        neighbor.y = currentCell.y - 1;
        cellId = 180;
      } else if (n == 3) { // West
        neighbor.x = currentCell.x - 1;
        neighbor.y = currentCell.y;
        cellId = 270;
      }

      if (neighbor.x >= 0 && neighbor.x < MAZE_WIDTH && neighbor.y >= 0 && neighbor.y < MAZE_HEIGHT) { // check if neighbor is valid
        neighbor.cost = maze[neighbor.y][neighbor.x].cost;

        if (neighbor.cost < lowestCost && !hasWall(x, y, cellId)) {
          lowestCost = neighbor.cost;
          lowestCostCell = cellId;
        }
      }
  }

  if (x == goal.x && y == goal.y) {
    setMotorPWM(0, 0);
  }

  if (lowestCostCell == 0) {
    driveDistance(1.5, 1.5, 180);
    updatePosition();
  } else if (lowestCostCell == 90) {
    driveAngle(1.0, 1.0, -90);
    updateDirection(90);
    driveDistance(1.5, 1.5, 180);
    updatePosition();
  } else if (lowestCostCell == 270) {
    driveAngle(1.0, 1.0, 90);
    updateDirection(-90);
    driveDistance(1.5, 1.5, 180);
    updatePosition();
  } else if (lowestCostCell == 180) {
    driveAngle(1.0, 1.0, 180);
    updateDirection(180);
    driveDistance(1.5, 1.5, 180);
    updatePosition();
  } else if (lowestCostCell == 254) {
    setMotorPWM(0, 0);
  }
  delay(500);
}
