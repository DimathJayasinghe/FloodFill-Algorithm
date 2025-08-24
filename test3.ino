#include <cstdint>
#include <Arduino.h>
#include <queue>
using namespace std;

// Board dimensions
#define SIZE 4

// Target position
#define TARGET_ROW 0
#define TARGET_COL 3

// Starting position
#define START_ROW 3
#define START_COL 0
#define START_DIR NORTH

enum TargetType {START, TARGET} targetType;

enum Direction{
    NORTH = 0x01,
    EAST  = 0x02,
    SOUTH = 0x04,
    WEST  = 0x08
}Direction;

// Robo relative directions
enum RelativeDirection {
    FORWARD,
    LEFT,
    RIGHT,
    BACKWARD
}relDirection;

// MAZE WALL BITMASKS
struct Cell{
    uint8_t walls;
    int steps;
}maze[SIZE][SIZE];

// Robot state
struct Robot{
    int row;
    int col;
    int dir;
}robot;

// IR Sensor PINS
#define IR_LEFT_PIN 4
#define IR_FRONT_PIN 7   
#define IR_RIGHT_PIN 6

// Motor PINS
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

// Motor speed
#define BASE_SPEED 140
#define TURN_SPEED 160
#define SLOW_SPEED 100
#define TURN_TIME 232    // ms for 90° turn (CALIBRATE THIS)
#define MOVE_TIME 500    // ms to move forward (CALIBRATE THIS)


// MOTOR CALIBRATION FACTORS
float leftMotorFactor = 1.1;
float rightMotorFactor = 1.1;

#define LEFT_TURN_SPEED 1.3

// PID Constants (AUTO TUNED VALUES)
float Kp = 0.4;   // Proportional gain
float Ki = 0.008;  // Integral gain
float Kd = 0.12;   // Derivative gain
int SENSOR_OFFSET = 0;  // Add this define with your other constants

// PID Variables
float error = 0, lastError = 0;
float P = 0, I = 0, D = 0;
float PID_value = 0;








////////////////////////////////////////
// Sensor functions
void setupSensors() {
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_FRONT_PIN, INPUT); 
    pinMode(IR_RIGHT_PIN, INPUT);
}
bool isLeftWallDetected() {
    return digitalRead(IR_LEFT_PIN) == LOW; 
}
bool isFrontWallDetected() {
    return digitalRead(IR_FRONT_PIN) == LOW;
}
bool isRightWallDetected() {
    return digitalRead(IR_RIGHT_PIN) == LOW;
}
int getLeftDistance() {
    return analogRead(IR_LEFT_PIN);  // Higher values typically mean greater distance
}
int getRightDistance() {
    return analogRead(IR_RIGHT_PIN); // Higher values typically mean greater distance
}

////////////////////////////////////////
// Tune PID function
void calibrateSensors() {
  // Take multiple readings
  int leftTotal = 0, rightTotal = 0;
  int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    leftTotal += getLeftDistance();
    rightTotal += getRightDistance();
  }
  
  int leftAvg = leftTotal / samples;
  int rightAvg = rightTotal / samples;
  SENSOR_OFFSET = rightAvg - leftAvg;
}

void tunePID() {
  I = 0;
  int i=0;
  while (i<100) {
    if (isLeftWallDetected() && isRightWallDetected()) {
      int leftDist = getLeftDistance();
      int rightDist = getRightDistance();
      
      // Calculate error with offset
      calibrateSensors();
      error = (rightDist - leftDist) - SENSOR_OFFSET;
      
      // PID calculation
      P = error;
      I += error;
      D = error - lastError;
      lastError = error;
      
      I = constrain(I, -100, 100);
      
      PID_value = (Kp * P) + (Ki * I) + (Kd * D);
      
      // Output debugging info
    //   Serial.print("Error: "); Serial.print(error);
    //   Serial.print(" | PID: "); Serial.print(PID_value);
    //   Serial.print(" | L: "); Serial.print(leftDist);
    //   Serial.print(" | R: "); Serial.println(rightDist);
      
      // Apply to motors
      int leftSpeed = BASE_SPEED * leftMotorFactor - PID_value;
      int rightSpeed = BASE_SPEED * rightMotorFactor + PID_value;
      
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, leftSpeed);
      analogWrite(ENB, rightSpeed);
    } else {
    //   Serial.println("Not enough walls for PID testing!");
      break;
    }
    i++;
  }
  
  stopMotors();
}


////////////////////////////////////////
// Motor functions
void setupMotors() {
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
}
void stopMotors() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    delay(50);
}

void moveForward() {
  unsigned long startTime = millis();
  unsigned long currentTime = startTime;
  
  // Reset I term for new movement
  I = 0;
  
  // Continue until we've moved the required amount of time
  while (currentTime - startTime < MOVE_TIME) {
    // Only apply PID correction if both walls are present
    if (isLeftWallDetected() && isRightWallDetected()) {
      tunePID();
      // Apply PID to motor speeds - positive PID means adjust toward left
      int leftSpeed = BASE_SPEED * leftMotorFactor - PID_value;
      int rightSpeed = BASE_SPEED * rightMotorFactor + PID_value;
      
      // Constrain speeds to valid range
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      
      // Apply to motors
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, leftSpeed);
      analogWrite(ENB, rightSpeed);
    } 
    else {
      // No walls on both sides or can't determine, move straight
      int leftSpeed = BASE_SPEED * leftMotorFactor;
      int rightSpeed = BASE_SPEED * rightMotorFactor;
      
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, leftSpeed);
      analogWrite(ENB, rightSpeed);
    }
    
    // Small delay for readings to stabilize
    delay(20);
    currentTime = millis();
  }
  
  stopMotors();
  
  // Update robot position based on current direction
  switch(robot.dir) {
    case NORTH:
      if(robot.row > 0) robot.row--;
      break;
    case EAST:
      if(robot.col < SIZE-1) robot.col++;
      break;
    case SOUTH:
      if(robot.row < SIZE-1) robot.row++;
      break;
    case WEST:
      if(robot.col > 0) robot.col--;
      break;
  }
}
void turnLeft() {
    int leftSpeed = TURN_SPEED * leftMotorFactor;
    int rightSpeed = TURN_SPEED * rightMotorFactor * LEFT_TURN_SPEED;
    // int rightSpeed = TURN_SPEED * rightMotorFactor;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    // Pivot turn (left wheels backward, right wheels forward)
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);

    delay(TURN_TIME+20);
    stopMotors();
    delay(100);

    // Update robot direction (90° counter-clockwise)
    robot.dir = (robot.dir == NORTH) ? WEST : 
                (robot.dir == WEST) ? SOUTH : 
                (robot.dir == SOUTH) ? EAST : NORTH;
}
void turnRight() {

  int leftSpeed = TURN_SPEED * leftMotorFactor;
  int rightSpeed = TURN_SPEED * rightMotorFactor;
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Pivot turn (right wheels backward, left wheels forward)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  
  delay(TURN_TIME);
  stopMotors();
  delay(100);

  // Update robot direction (90° clockwise)
  robot.dir = (robot.dir == NORTH) ? EAST : 
              (robot.dir == EAST) ? SOUTH : 
              (robot.dir == SOUTH) ? WEST : NORTH;
}

void tiltLeft() {
    int leftSpeed = SLOW_SPEED * leftMotorFactor;
    int rightSpeed = BASE_SPEED * rightMotorFactor;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    delay(MOVE_TIME / 2);
    stopMotors();
}
void tiltRight() {
    int leftSpeed = BASE_SPEED * leftMotorFactor;
    int rightSpeed = SLOW_SPEED * rightMotorFactor;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    delay(MOVE_TIME / 2);
    stopMotors();
}

////////////////////////////////////////
// Flood fill algorithm
void floodFill(TargetType tType){
    int goal_row, goal_col;
    if(tType == TARGET){
        goal_row = TARGET_ROW;
        goal_col = TARGET_COL;
    } else {
        goal_row = START_ROW;
        goal_col = START_COL;
    }
    queue<std::pair<int, int>> q;
    q.push({goal_row, goal_col});
    while(!q.empty()){
        auto [r, c] = q.front();
        q.pop();
        int current_steps = maze[r][c].steps;
        // Check all four directions
        if(r > 0 && !(maze[r][c].walls & NORTH) && maze[r-1][c].steps > current_steps + 1){
            maze[r-1][c].steps = current_steps + 1;
            q.push({r-1, c});
        }
        if(c < SIZE-1 && !(maze[r][c].walls & EAST) && maze[r][c+1].steps > current_steps + 1){
            maze[r][c+1].steps = current_steps + 1;
            q.push({r, c+1});
        }
        if(r < SIZE-1 && !(maze[r][c].walls & SOUTH) && maze[r+1][c].steps > current_steps + 1){
            maze[r+1][c].steps = current_steps + 1;
            q.push({r+1, c});
        }
        if(c > 0 && !(maze[r][c].walls & WEST) && maze[r][c-1].steps > current_steps + 1){
            maze[r][c-1].steps = current_steps + 1;
            q.push({r, c-1});
        }
    } 
}

////////////////////////////////////////
// Maze functions
void setupMaze(){
    for(int r=0; r<SIZE; r++){
        for(int c=0; c<SIZE; c++){
            maze[r][c].walls = 0;
            maze[r][c].steps = 255; // Initialize steps to a high value
        }
    }
}
void updateWalls(RelativeDirection relDir){
    uint8_t direction;
    switch(relDir){
        case FORWARD:
            direction = robot.dir;
            break;
        case LEFT:
            direction = (robot.dir == NORTH) ? WEST : (robot.dir == WEST) ? SOUTH : (robot.dir == SOUTH) ? EAST : NORTH;
            break;
        case RIGHT:
            direction = (robot.dir == NORTH) ? EAST : (robot.dir == EAST) ? SOUTH : (robot.dir == SOUTH) ? WEST : NORTH;
            break;
    }
    maze[robot.row][robot.col].walls |= direction;
    // Update the adjacent cell's wall in the opposite direction
    int adjRow = robot.row, adjCol = robot.col;
    if(direction == NORTH && robot.row > 0) adjRow--;
    else if(direction == EAST && robot.col < SIZE-1) adjCol++;
    else if(direction == SOUTH && robot.row < SIZE-1) adjRow++;
    else if(direction == WEST && robot.col > 0) adjCol--;
    if(adjRow >= 0 && adjRow < SIZE && adjCol >= 0 && adjCol < SIZE){
        maze[adjRow][adjCol].walls |= (direction == NORTH) ? SOUTH : (direction == EAST) ? WEST : (direction == SOUTH) ? NORTH : EAST;
    }
}
void setGoalToTarget(){
    for(int r=0;r<SIZE;r++){
        for(int c=0;c<SIZE;c++){
            maze[r][c].steps = 255;
        }
    }
    maze[TARGET_ROW][TARGET_COL].steps = 0;
    floodFill(TARGET);
}
void setGoalToStart(){
    for(int r=0;r<SIZE;r++){
        for(int c=0;c<SIZE;c++){
            maze[r][c].steps = 255;
        }
    }
    maze[START_ROW][START_COL].steps = 0;
    floodFill(START);
}

// Setup robot
void setupRobot(){
    robot.row = START_ROW;
    robot.col = START_COL;
    robot.dir = START_DIR;
}

// Main program
// Initialize robot position and direction
void setup(){
    Serial.begin(115200);
    setupSensors();
    setupMotors();
    setupMaze();
    setupRobot();
    delay(100);
    floodFill(TARGET);
}



// Check for walls and update maze
void checkForWalls(){
    if(isFrontWallDetected()){
        Serial.print(" F:"); Serial.print(isFrontWallDetected() ? "WALL" : "OPEN");  // FRONT SENSOR DISPLAY
        updateWalls(FORWARD);
    }
    if(isLeftWallDetected()){
        updateWalls(LEFT);
        Serial.print(" L:"); Serial.print( isLeftWallDetected()? "WALL" : "OPEN");
    }
    if(isRightWallDetected()){
        Serial.print(" R:"); Serial.println(isRightWallDetected() ? "WALL" : "OPEN");
        updateWalls(RIGHT);
    }
    Serial.println();
}
void doTheNextMove(){
    int current_steps = maze[robot.row][robot.col].steps;
    int next_row = robot.row, next_col = robot.col;
    int next_dir = robot.dir;
    
    // Check all four directions for the lowest step count
    if(robot.row > 0 && !(maze[robot.row][robot.col].walls & NORTH) && maze[robot.row-1][robot.col].steps < current_steps){
        current_steps = maze[robot.row-1][robot.col].steps;
        next_row = robot.row - 1;
        next_col = robot.col;
        next_dir = NORTH;
    }
    if(robot.col < SIZE-1 && !(maze[robot.row][robot.col].walls & EAST) && maze[robot.row][robot.col+1].steps < current_steps){
        current_steps = maze[robot.row][robot.col+1].steps;
        next_row = robot.row;
        next_col = robot.col + 1;
        next_dir = EAST;
    }
    if(robot.row < SIZE-1 && !(maze[robot.row][robot.col].walls & SOUTH) && maze[robot.row+1][robot.col].steps < current_steps){
        current_steps = maze[robot.row+1][robot.col].steps;
        next_row = robot.row + 1;
        next_col = robot.col;
        next_dir = SOUTH;
    }
    if(robot.col > 0 && !(maze[robot.row][robot.col].walls & WEST) && maze[robot.row][robot.col-1].steps < current_steps){
        current_steps = maze[robot.row][robot.col-1].steps;
        next_row = robot.row;
        next_col = robot.col - 1;
        next_dir = WEST;
    }

    
    // Determine the turn needed
    if(next_dir != robot.dir){
        if((robot.dir == NORTH && next_dir == WEST) || (robot.dir == WEST && next_dir == SOUTH) ||
           (robot.dir == SOUTH && next_dir == EAST) || (robot.dir == EAST && next_dir == NORTH)){
            turnLeft();
            Serial.println("Turning Left");
            
        } else if((robot.dir == NORTH && next_dir == EAST) || (robot.dir == EAST && next_dir == SOUTH) ||
                  (robot.dir == SOUTH && next_dir == WEST) || (robot.dir == WEST && next_dir == NORTH)){
            turnRight();
            Serial.println("Turning Right");
        } else if ((robot.dir == NORTH && next_dir == SOUTH) || (robot.dir == SOUTH && next_dir == NORTH) ||
                  (robot.dir == EAST && next_dir == WEST) || (robot.dir == WEST && next_dir == EAST)){
            // U-turn
            turnRight();
            turnRight();
            Serial.println("U-turn");
        }   
    }
}

void printMatrix(){
    for(int r=0;r<SIZE;r++){
        for(int c=0;c<SIZE;c++){
            Serial.print(maze[r][c].steps); Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}

// Add after your other functions
void printPIDInfo() {
  Serial.print("P=");
  Serial.print(P);
  Serial.print(" I=");
  Serial.print(I);
  Serial.print(" D=");
  Serial.print(D);
  Serial.print(" PID=");
  Serial.println(PID_value);
}


// testing loop
/**
 void loop() {
   Serial.println("\nPress 't' to test PID, 'p', 'i', or 'd' to adjust values, 'c' to calibrate sensors");
   
   while (!Serial.available()) {
     delay(100);
   }
   
   char cmd = Serial.read();
   
   switch(cmd) {
     case 't':
       testPID();
       break;
     case 'p':
       Kp += 0.05;
       Serial.print("Kp increased to: "); Serial.println(Kp);
       break;
     case 'P':
       Kp = max(0.00, Kp - 0.05);
       Serial.print("Kp decreased to: "); Serial.println(Kp);
       break;
     case 'i':
       Ki += 0.005;
       Serial.print("Ki increased to: "); Serial.println(Ki);
       break;
     case 'I':
       Ki = max(0.00, Ki - 0.005);
       Serial.print("Ki decreased to: "); Serial.println(Ki);
       break;
     case 'd':
       Kd += 0.05;
       Serial.print("Kd increased to: "); Serial.println(Kd);
       break;
     case 'D':
       Kd = max(0.00, Kd - 0.05);
       Serial.print("Kd decreased to: "); Serial.println(Kd);
       break;
     case 'c':
       calibrateSensors();
       break;
   }
 * 
}
 */


void loop(){
    
    checkForWalls();
    setGoalToTarget();
    doTheNextMove();
    moveForward();
    delay(1000); // Adjust delay as needed
    if(robot.row == TARGET_ROW && robot.col == TARGET_COL){
        Serial.println("Reached the target!");
        stopMotors();
        while(true); // Stop the program
    }
}
