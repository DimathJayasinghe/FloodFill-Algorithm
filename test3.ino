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
#define TURN_TIME 269    // ms for 90° turn (CALIBRATE THIS)
#define MOVE_TIME 630    // ms to move forward (CALIBRATE THIS)

// MOTOR CALIBRATION FACTORS
float leftMotorFactor = 1.10;
float rightMotorFactor = 1.2;



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
  int leftSpeed = BASE_SPEED * leftMotorFactor;
  int rightSpeed = BASE_SPEED * rightMotorFactor;
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  delay(MOVE_TIME);
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
    // Pivot turn (left wheels backward, right wheels forward)
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    
    int leftSpeed = TURN_SPEED * leftMotorFactor;
    int rightSpeed = TURN_SPEED * rightMotorFactor;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    delay(TURN_TIME);
    stopMotors();
    delay(100);

    // Update robot direction (90° counter-clockwise)
    robot.dir = (robot.dir == NORTH) ? WEST : 
                (robot.dir == WEST) ? SOUTH : 
                (robot.dir == SOUTH) ? EAST : NORTH;
}
void turnRight() {
  // Pivot turn (right wheels backward, left wheels forward)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  
  int leftSpeed = TURN_SPEED * leftMotorFactor;
  int rightSpeed = TURN_SPEED * rightMotorFactor;
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
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

// Main loop
void loop(){
    setGoalToTarget();
    checkForWalls();
    Serial.println();
    floodFill(TARGET);
    // printMatrix();
    doTheNextMove();
    moveForward();

  
    // Serial.print("Robot Position: (");
    // Serial.print(robot.row);
    // Serial.print(", ");
    // Serial.print(robot.col);
    // Serial.print(") Direction: ");
    // switch(robot.dir){
    //     case NORTH: Serial.println("NORTH"); break;
    //     case EAST: Serial.println("EAST"); break;
    //     case SOUTH: Serial.println("SOUTH"); break;
    //     case WEST: Serial.println("WEST"); break;
    // }
    // Serial.print("Current Steps: ");
    // Serial.println(maze[robot.row][robot.col].steps);
    delay(1000); // Adjust delay as needed
    if(robot.row == TARGET_ROW && robot.col == TARGET_COL){
        Serial.println("Reached the target!");
        stopMotors();
        while(true); // Stop the program
    }
}





