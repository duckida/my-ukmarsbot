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
const int frontWallThreshold = 40;
const int leftGapThreshold = 18;
const int rightGapThreshold = 18;

float eDifference;

// PID CONSTANTS
float target = 12; // The target walldistance or encoderdifference

// Tuning Constants
const float Kp = 2; // The Kp value
// const float Ki = 0; // The Ki value
const float Kd = 0.25; // The Kd value

// Optimal battery charge - 7.2v - 7.3v
volatile int32_t encoderLeftCount;
volatile int32_t encoderRightCount;
uint32_t updateTime;
uint32_t updateInterval = 40;  // in milliseconds
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
const uint8_t MAZE_WIDTH = 16;
const uint8_t MAZE_HEIGHT = 16;
const uint8_t QUEUE_SIZE = 64;//MAZE_WIDTH * MAZE_HEIGHT;

#define COMPASS_NORTH 0
#define COMPASS_EAST 90
#define COMPASS_WEST 270
#define COMPASS_SOUTH 180

uint8_t GOAL_X = 2;
uint8_t GOAL_Y = 4;

const uint8_t START_X = 0;
const uint8_t START_Y = 0;

enum Wall {
  CLEAR = 0,
  WALL = 1,
  UNKNOWN = 2
};

struct Cell {
  uint8_t x;
  uint8_t y;
  uint8_t cost;

  Wall north : 2;
  Wall east : 2;
  Wall west : 2;
  Wall south : 2;
};

struct QueueObject {
  uint8_t x;
  uint8_t y;
};

Cell maze[MAZE_HEIGHT][MAZE_WIDTH];

bool positionUpdated = false;
bool mazeUpdated = false;

void setWall(int x, int y, int dir, Wall state) {
  if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return;

  if (dir == COMPASS_NORTH) { // North
    maze[y][x].north = state;  // Set North wall
    if (y + 1 < MAZE_HEIGHT) maze[y+1][x].south = state;
  }
  else if (dir == COMPASS_EAST) { // East
    maze[y][x].east = state;
    if (x + 1 < MAZE_WIDTH) maze[y][x+1].west = state;
  }
  else if (dir == COMPASS_SOUTH) { // South
    maze[y][x].south = state;
    if (y - 1 >= 0) maze[y-1][x].north = state;
  }
  else if (dir == COMPASS_WEST) { // West
    maze[y][x].west = state;
    if (x - 1 >= 0) maze[y][x-1].east = state;
  }
  mazeUpdated = true;
}


// Check if a wall exists in that direction
bool hasWall(int x, int y, int dir) {
  Wall wallState;
  if (dir == COMPASS_NORTH)      wallState = maze[y][x].north;
  else if (dir == COMPASS_EAST)  wallState = maze[y][x].east;
  else if (dir == COMPASS_SOUTH) wallState = maze[y][x].south;
  else if (dir == COMPASS_WEST)  wallState = maze[y][x].west;
  else return false; // error case

  return (wallState == WALL); // true if there's a wall, false otherwise
}

// Check if a wall is known in that direction
Wall wallKnown(int x, int y, int dir) {
  Wall wallState;
  if (dir == COMPASS_NORTH)      wallState = maze[y][x].north;
  else if (dir == COMPASS_EAST)  wallState = maze[y][x].east;
  else if (dir == COMPASS_SOUTH) wallState = maze[y][x].south;
  else if (dir == COMPASS_WEST)  wallState = maze[y][x].west;
  else return UNKNOWN; // error case

  Serial.println(wallState);
  return wallState; // returns 0 1 or 2?
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
  //getBatteryVolts();
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setLeftMotorPWM(motorPWM);
}

// Set voltage of right motor
void setRightMotorVolts(float volts) {
  //getBatteryVolts();
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
  eDifference = encoderLeftCount - encoderRightCount;
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
  eDifference = encoderLeftCount - encoderRightCount;
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

void drivePID(float lSpeed, float rSpeed, float distance, bool direction) { // direction : true for right
  Serial.println("PIDDrive START");
  encoderLeftCount = encoderRightCount = 0;
  Serial.println(gSensorLeft);
  
  float encoderAvg = (encoderLeftCount + encoderRightCount) / 2;
      
  float circumferences = distance / circumference;
  float counts = circumferences * 234;

  if (direction == true) { // right
    while (encoderAvg < counts) {
      float pid = rightPID() / 100;
      setMotorVolts(lSpeed + pid, rSpeed - pid);
      encoderAvg = (encoderLeftCount + encoderRightCount) / 2;
    }
  } else {
    while (encoderAvg < counts) {
      float pid = PID() / 100;
      setMotorVolts(lSpeed - pid, rSpeed + pid);
      encoderAvg = (encoderLeftCount + encoderRightCount) / 2;
    }
  }

  setMotorVolts(0,0);
  Serial.println("PIDDrive END");
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

// PID Controller
float rightPID() {
  int error = target - gSensorRight; // Error = target value - current value

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
  // getBatteryVolts();
}



// ------------POSITIONING-----------

int calculateDirection(int change) { // the function, given a direction, adds the given `change` to the current direction and returns a bound integer.
  int currentDirection = direction;
  if (change < 0){
    currentDirection += (360 + change) % 360; // subtracts from 360 and constrains using modulo
  } else {
    currentDirection += change % 360; // constrains using modulo
  }
  currentDirection = currentDirection % 360;
  return currentDirection;
}

void updateDirection(int change) { // this does the same as `calculateDirection()` but updates the robot's direction.
  direction = calculateDirection(change);

  Serial.println("Changed Direction");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(direction);
  Serial.println();
}

void updatePosition() {
  switch(direction) {
      case COMPASS_NORTH:
        y += 1;
        break;
      case COMPASS_EAST:
        x += 1;
        break;
      case COMPASS_SOUTH:
        y -= 1;
        break;
      case COMPASS_WEST:
        x -= 1;
        break;
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
/*void askForTarget() {
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
}*/


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

void printMazeWithWalls() {
  // Print top border (north edge of top row)
  Serial.print("+");
  for (int x = 0; x < MAZE_WIDTH; x++) {
    bool isTopEdge = (MAZE_HEIGHT - 1 == MAZE_HEIGHT - 1); // always true for top row
    bool knownNorthWall = hasWall(x, MAZE_HEIGHT - 1, 0); // 0° = North
    if (knownNorthWall) {
      Serial.print("===");
    } else {
      // Undiscovered top edge: use quotes or tildes
      Serial.print("\"\"\""); // or "~~~"
    }
    Serial.print("+");
  }
  Serial.println();

  // Print maze from top to bottom
  for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
    // Print cell content row
    // Handle leftmost vertical wall (west of first cell)
    bool isLeftEdge = (0 == 0);
    bool knownWestWallOfFirstCell = hasWall(0, y, COMPASS_WEST); // West wall of (0, y)
    if (knownWestWallOfFirstCell) {
      Serial.print("|");
    } else {
      Serial.print("["); // Undiscovered left edge
    }

    for (int x = 0; x < MAZE_WIDTH; x++) {
      // Print cell content
      if (x == ::x && y == ::y) {
        if (maze[y][x].cost < 10) {
          Serial.print(" ");
          Serial.print(maze[y][x].cost);
          Serial.print("*");
        } else {
          Serial.print(maze[y][x].cost);
          Serial.print("*");
        }
      } else {
        if (maze[y][x].cost < 10) {
          Serial.print(" ");
          Serial.print(maze[y][x].cost);
          Serial.print(" ");
        } else {
          Serial.print(maze[y][x].cost);
          Serial.print(" ");
        }
      }

      // Print east wall (or right edge)
      if (x == MAZE_WIDTH - 1) {
        // Rightmost cell: check east wall (90°)
        bool knownEastWall = hasWall(x, y, COMPASS_EAST);
        if (knownEastWall) {
          Serial.print("|");
        } else {
          Serial.print("]"); // Undiscovered right edge
        }
      } else {
        // Internal vertical wall: between x and x+1 → it's the east wall of (x,y)
        bool knownInternalWall = hasWall(x, y, COMPASS_EAST); // East wall
        if (knownInternalWall) {
          Serial.print("|");
        } else {
          Serial.print(" ");
        }
      }
    }
    Serial.println();

    // Print horizontal wall row (south walls of this row)
    Serial.print("+");
    for (int x = 0; x < MAZE_WIDTH; x++) {
      bool isBottomEdge = (y == 0);
      bool knownSouthWall = hasWall(x, y, COMPASS_SOUTH); // 180° = South
      if (knownSouthWall) {
        Serial.print("===");
      } else if (isBottomEdge) {
        // Undiscovered bottom edge
        Serial.print("___");
      } else {
        // Internal unknown wall → just spaces (no wall)
        Serial.print("   ");
      }
      Serial.print("+");
    }
    Serial.println();
  }
  Serial.println();
}

// ------------MAIN CODE-----------

void debugUnknown() {
  for (int i = 0; i < MAZE_HEIGHT; i++) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      if(maze[i][j].west == UNKNOWN) {
        Serial.print("UNKNOWN @ ");
        Serial.print(j);
        Serial.print(" ");
        Serial.print(i);
        Serial.println();
      }
    }
  }
}

bool leftWall = false;
bool rightWall = false;
bool frontWall = false;

void setup() {
  Serial.begin(57600);
  pinMode(EMITTER, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  digitalWrite(EMITTER, 0);  // be sure the emitter is off
  analogueSetup();           // increase the ADC conversion speed
  motorSetup();
  setupEncoder();
  setupSystick();
  updateTime = millis() + updateInterval;

  x = 0;
  y = 0;

  for (int i = 0; i < MAZE_HEIGHT; i++) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      maze[i][j].north = UNKNOWN;
      maze[i][j].east = UNKNOWN;
      maze[i][j].south = UNKNOWN;
      maze[i][j].west = UNKNOWN;
    }
  }

  setWall(0, 0, COMPASS_EAST, WALL);
  setWall(0, 0, COMPASS_WEST, WALL);

  delay(3000);
  encoderLeftCount = 0;
  encoderRightCount = 0;
  target = gSensorLeft;
  getBatteryVolts();
  driveDistance(1.5, 1.5, 24);

}

int state; // 0=solving 1=returning 2=stopped 3=failed

void goACellForward() {
  // from middle of cell
  driveDistance(1.5, 1.5, 50); // go to sensing point
  Serial.println("I'm at the Sensing Point");

  delay(200);

  uint8_t leftTotal = 0;
  uint8_t rightTotal = 0;

  uint8_t leftAvg; uint8_t rightAvg;

  uint8_t i;
  for (i=0; i<3; i++) {
    updateWallSensor();
    leftTotal += gSensorLeft;
    rightTotal += gSensorRight;
    Serial.print(gSensorLeft);
    Serial.print(" ");
    Serial.print(gSensorRight); Serial.println();
  }

  leftAvg = leftTotal / 3;
  rightAvg = rightTotal / 3;

  Serial.print("Averages: ");


  Serial.print(leftAvg);
  Serial.print(" ");
  Serial.print(rightAvg); Serial.println();

  // updateWallSensor(); // read the sensors
  if (leftAvg <= leftGapThreshold) {
    leftWall = false;
  } else {
    leftWall = true;
    Serial.println("Wall on the left");
    Serial.print("Sensor: ");
    Serial.print(leftAvg);
    Serial.print(" & Threshold: ");
    Serial.print(leftGapThreshold);
    Serial.println();
  }
  if (rightAvg <= rightGapThreshold) {
    rightWall = false;
  } else {
    rightWall = true;
    Serial.println("Wall on the right");
    Serial.print("Sensor: ");
    Serial.print(rightAvg);
    Serial.print(" & Threshold: ");
    Serial.print(rightGapThreshold);
    Serial.println();
  }

  // go to the middle of next cell
  //driveDistance(1.5, 1.5, 100);

  if (leftWall) drivePID(1.5, 1.5, 105, false);
  else if (rightWall) drivePID(1.5, 1.5, 105, true);
  else driveDistance(1.5, 1.5, 105);

  updatePosition(); // update position

  Serial.println("I'm in the middle of the next cell"); 
  if (state == 0) { // exploring
    Serial.println("i'll update the wall.");
    if (leftWall == true) setWall(x, y, calculateDirection(270), WALL); // set walls
    else setWall(x, y, calculateDirection(270), CLEAR);
    if (rightWall == true) setWall(x, y, calculateDirection(90), WALL);
    else setWall(x, y, calculateDirection(90), CLEAR);
  } // otherwise don't update walls, leave as unknown

}

void mazeLoop() {
  if (state == 0) { // solving state
    delay(100);
    // 1. check front walls
    uint16_t frontTotal = 0;
    uint8_t i; 
    for (i=0; i<3; i++) {
      updateWallSensor();
      frontTotal += gSensorFront;
      Serial.print("FrontR ");
      Serial.print(gSensorFront); Serial.println();
    }

    uint16_t frontAvg = frontTotal / 3;

    Serial.print("FrontA ");
    Serial.print(frontAvg); Serial.println();

    Serial.print("FrontSensor: ");
    Serial.print(frontAvg);
    Serial.print("Threshold: ");
    Serial.print(frontWallThreshold);
    Serial.println();
    if (state == 0) { // exploring
      if (frontAvg > frontWallThreshold) { // There's a wall in front...
        frontWall = true;
        setWall(x, y, direction, WALL);
      } else {
        frontWall = false;
        Serial.println("frontwall ha nai.");
        setWall(x, y, direction, CLEAR);
      }
    }


      // --- Flood update only when needed ---
    if (mazeUpdated || positionUpdated) {
      calculateFlood();
      // printMazeWithWalls();
      mazeUpdated = false;
      positionUpdated = false;
    }


    int lowestCost = 254;
    int lowestCostCell;

    Cell currentCell = maze[y][x];



    for (int n = 0; n < 4; n++) {
      //Serial.println("start!");
      //    Serial.println(n);
        Cell neighbor;
        int cellId;
        switch (n) {
          case 0: // north
            //Serial.println("Printing N");
            neighbor.x = currentCell.x;
            neighbor.y = currentCell.y + 1;
            cellId = COMPASS_NORTH;
            break;
          case 1: // east
            //Serial.println("Printing E");
            neighbor.x = currentCell.x + 1;
            neighbor.y = currentCell.y;
            cellId = COMPASS_EAST;
            break;
          case 2:
            //Serial.println("Printing S");
            neighbor.x = currentCell.x;
            neighbor.y = currentCell.y - 1;
            cellId = COMPASS_SOUTH;
            break;
          case 3: 
            //Serial.println("Printing W");
            neighbor.x = currentCell.x - 1;
            neighbor.y = currentCell.y;
            cellId = COMPASS_WEST;
            break;
        }

        //Serial.println("before the `if` check");
        //Serial.print("cellid ");
       //Serial.print(cellId);
        //Serial.print(" x ");
        //Serial.print(neighbor.x);
        //Serial.print(" y ");
        //Serial.print(neighbor.y);
        //Serial.println();

        if (neighbor.x >= 0 && neighbor.x < MAZE_WIDTH && neighbor.y >= 0 && neighbor.y < MAZE_HEIGHT) { // check if neighbor is valid
          neighbor.cost = maze[neighbor.y][neighbor.x].cost; // fetch the cost of neighbor
          if (neighbor.cost < lowestCost && !hasWall(x, y, cellId)) {
            //Serial.print("*The lowest is: ");
            //Serial.print(neighbor.cost);
            //Serial.println();
            //Serial.println("and it passed wall check.");
            lowestCost = neighbor.cost;
            lowestCostCell = cellId;
          }
        } else {
          //Serial.println("it's not valid :(");
          //Serial.print("The one it thinks isn't valid is ");
          //Serial.print(cellId);
          //Serial.print(" with cost ");
          //Serial.print(neighbor.cost);
          //Serial.println();
        }
    }

    //Serial.print("FINAL ");
    //Serial.print(lowestCostCell);
    //Serial.print(" with cost ");
    //Serial.print(lowestCost);
    //Serial.println();

    delay(200);

    int turningAngle = lowestCostCell - direction;
    if (turningAngle < 0) {
      turningAngle += 360;
    }

    Serial.println(turningAngle);
    switch (turningAngle) {
      case 0:
        Serial.println("I'll go Forward");
        goACellForward();
        break;
      case 90: 
        Serial.println("I'll go Right");
        driveAngle(1.0, 1.0, -95); // turn right
        updateDirection(90); // update direction
        goACellForward();
        break;
      case 180:
        Serial.println("I'll go South");
        driveAngle(1.0, 1.0, 180); // turn 180
        updateDirection(180); // update direction
        goACellForward();
        break;
      case 270:
        Serial.println("I'll go Left");
        driveAngle(1.0, 1.0, 90); // turn left
        updateDirection(270);
        goACellForward();
        break;
      case 254:
        state = 3; // failed
    }

  } 
  else if (state == 1) { // returning state
    int cellToGo;

    Cell currentCell = maze[y][x];

    for (int n = 0; n < 4; n++) {
      Serial.println("anneyeong haseyo!");
      //Serial.println(n);
        Cell neighbor;
        int cellId;
        switch (n) {
          case 0: // north
            //Serial.println("Printing N");
            neighbor.x = currentCell.x;
            neighbor.y = currentCell.y + 1;
            cellId = COMPASS_NORTH;
            break;
          case 1: // east
            //Serial.println("Printing E");
            neighbor.x = currentCell.x + 1;
            neighbor.y = currentCell.y;
            cellId = COMPASS_EAST;
            break;
          case 2:
            //Serial.println("Printing S");
            neighbor.x = currentCell.x;
            neighbor.y = currentCell.y - 1;
            cellId = COMPASS_SOUTH;
            break;
          case 3: 
            //Serial.println("Printing W");
            neighbor.x = currentCell.x - 1;
            neighbor.y = currentCell.y;
            cellId = COMPASS_WEST;
            break;
        }

        //Serial.println("before the `if` check");
        //Serial.print("cellid ");
        //Serial.print(cellId);
        //Serial.print(" x ");
        //Serial.print(neighbor.x);
        //Serial.print(" y ");
        //Serial.print(neighbor.y);
       // Serial.println();
        Serial.print("Is the wall of ");
        Serial.print(cellId);
        Serial.print(" known? ");
        Serial.print(wallKnown(neighbor.x, neighbor.y, cellId));
        Serial.println();
        debugUnknown();
        if ((neighbor.x >= 0) && (neighbor.x < MAZE_WIDTH) && (neighbor.y >= 0) && (neighbor.y < MAZE_HEIGHT)) { // check if neighbor is valid
          neighbor.cost = maze[neighbor.y][neighbor.x].cost; // fetch the cost of neighbor
          if ((neighbor.cost - currentCell.cost == 1) && (wallKnown(neighbor.x, neighbor.y, cellId) == 0)) {
            cellToGo = cellId;
          }
        } else {
          Serial.println("check failed.");
        }
    }

    delay(200);

    int turningAngle = cellToGo - direction;
    if (turningAngle < 0) {
      turningAngle += 360;
    } // ????

    Serial.print("I'll go to ");
    Serial.print(cellToGo);
    Serial.println();

    Serial.println(turningAngle);
    switch (turningAngle) {
      case 0:
        Serial.println("I'll go Forward");
        goACellForward();
        break;
      case 90: 
        Serial.println("I'll go Right");
        driveAngle(1.0, 1.0, -95); // turn right
        updateDirection(90); // update direction
        goACellForward();
        break;
      case 180:
        Serial.println("I'll go South");
        driveAngle(1.0, 1.0, 180); // turn 180
        updateDirection(180); // update direction
        goACellForward();
        break;
      case 270:
        Serial.println("I'll go Left");
        driveAngle(1.0, 1.0, 90); // turn left
        updateDirection(270);
        goACellForward();
        break;
    }

  } 
  else if (state == 2) { // finished
    Serial.println("Finished!");
  }
  else if (state == 3) { // failed
    Serial.println("Failed");
    setMotorPWM(0,0);
  }
}



/// ----------------------------LOOP----------------------------

void loop() {

  //printSensors();
  //delay(100);

  
  getBatteryVolts();
  // printMazeWithWalls();

  if ((x == goal.x && y == goal.y) && (state == 0)) {
    setMotorPWM(0, 0);
    state = 1;
    delay(500);
    Serial.println("----I'm going home!----");
  }

  if ((x == maze[START_X][START_Y].x && y == maze[START_X][START_Y].y) && (state == 1)) {
    setMotorPWM(0, 0);
    state = 2;
  }

  mazeLoop();
  delay(500);
}
