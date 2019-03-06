#include <MsTimer2.h>
#include <EnableInterrupt.h>
#include <DualVNH5019MotorShield.h>
//#include <ArduinoPIDLibrary.h>
#include <PID_v1.h>

const int LEFT_PULSE = 3;           // M1 - LEFT motor pin number on Arduino board
const int RIGHT_PULSE = 11;         // M2 - RIGHT motor pin number on Arduino board
const int MOVE_FAST_SPEED = 380;    //if target distance more than 30cm 
const int MOVE_MAX_SPEED = 350;     //if target distance more than 60cm
const int MOVE_MIN_SPEED = 200;     //if target distance less than 60cm
const int TURN_MAX_SPEED = 300;     //change this value to calibrate turning. If the rotation overshoots, decrease the speed 
const int ROTATE_MAX_SPEED = 150;   //used in rotateLeft() and rotateRight()
const int TURN_TICKS_L = 760;       //change this left encoder ticks value to calibrate left turn 
const int TURN_TICKS_R = 720;       //change this right encoder ticks value to calibrate right turn 
//const int TURN_TICKS_R = 763;

//TICKS[0] for general cm -> ticks calibration. 
//TICKS[1-9] with specific distance (by grids) e.g. distance=5, TICKS[5] 
// const int TICKS[10] = {440, 1155, 1760, 2380, 2985, 3615, 4195, 4775, 5370};  
const int TICKS[10] = {510, 1190, 1800, 2380, 3020, 3615, 4195, 4775, 5390, 0};  // for movement of each grid
const int LEFTTICK[14] = {20, 25, 30, 35, 40, 360, 50, 55, 489, 65, 70, 75, 80, 85};
const int RIGHTTICK[14] = {20, 25, 30, 35, 40, 313, 50, 55, 450, 65, 70, 75, 80, 85};
const double DIST_WALL_CENTER_BOX = 1.58;   //for aligning to the front wall/obstacle. Used in alignFront()
const double kp = 7.8, ki = 1.88, kd = 0;  //change values for moving straight line 
//const double kp = 7.35, ki = 1.25, kd = 5;  //whenever we increase kd, it goes to the right
//const double kp = 7.90, ki = 1.25, kd = 0;
//const double kp = 7.00, ki = 1.25, kd = 0;
//const double kp = 7.35, ki = 2, kd = 0;


int TENCM_TICKS_OFFSET = 0; //not used

// for PID
double tick_R = 0;
double tick_L = 0;
double speed_O = 0;
double previous_tick_R = 0;
double previous_error = 0;

//motor declaration as 'md' e.g. to set motor speed ==> md.setSpeeds(Motor1, Motor2)
DualVNH5019MotorShield md;  //our motor is Pololu Dual VNH5019 Motor Driver Shield 
PID myPID(&tick_R, &speed_O, &tick_L, kp, ki, kd, REVERSE);

//--------------------------Motor Codes-------------------------------
void setupMotorEncoder() {
  md.init();
  pinMode(LEFT_PULSE, INPUT);
  pinMode(RIGHT_PULSE, INPUT);
  enableInterrupt(LEFT_PULSE, leftMotorTime, CHANGE); //Enables interrupt on a left motor (M1) - enable interrupt basically enables the interrupt flag and enables Interrupt service routines to run
  enableInterrupt(RIGHT_PULSE, rightMotorTime, CHANGE); //Enables interrupt on a left motor (M1)
}

// not used
void stopMotorEncoder() {
  disableInterrupt(LEFT_PULSE);
  disableInterrupt(RIGHT_PULSE);
}

void setupPID() {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-400, 400);   //change this value for PID calibration. This is the maximum speed PID sets to
  //myPID.SetOutputLimits(-370, 370);   
  myPID.SetSampleTime(5);
}

// when forward command is received, taking in the parameter of how many cm it should move
void moveForward(int distance) {
  initializeTick();   // set all tick to 0
  initializeMotor_Start();  // set motor and brake to 0
  distance = cmToTicks(distance); // convert grid movement to tick value
  double currentSpeed = 0;
  boolean initialStatus = true;

  if (distance < 60) {    // if number of tick to move < 60, move at a slower speed of 200
    currentSpeed = MOVE_MIN_SPEED;
  } else {                // if number of tick to move >= 60, move at the max speed of 350
    currentSpeed = MOVE_MAX_SPEED;
  }

  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
    if (myPID.Compute()) {
      if (initialStatus) {
        //md.setSpeeds(0, currentSpeed);
        //delay(5);
        initialStatus = false;
      }
      md.setSpeeds(currentSpeed + speed_O, currentSpeed - speed_O);
    }
  }
  initializeMotor_End();  //brakes the motor
}

// ignore for now (thad)
// move forward faster (for fastestpath or clear straight path)
void moveForwardFast(int distance) {
  initializeTick();
  initializeMotor_Start();
  distance = cmToTicks(distance);
  double currentSpeed = 0;
  if (distance < 30) {
    currentSpeed = MOVE_MAX_SPEED;  //350
  } else {
    currentSpeed = MOVE_FAST_SPEED; //375
  }
  double offset = 0;
  int last_tick_R = 0;

  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
    if (distance - tick_R < 150)
      currentSpeed = 150;
    if ((tick_R - last_tick_R) >= 10 || tick_R == 0 || tick_R == last_tick_R) {
      last_tick_R = tick_R;
      offset += 0.1;
    }
    if (myPID.Compute() || tick_R == last_tick_R) {
      if (offset >= 1)
        md.setSpeeds(currentSpeed + speed_O, currentSpeed - speed_O);
      else
        md.setSpeeds(offset * (currentSpeed + speed_O), offset * (currentSpeed - speed_O));
    }
  }
  initializeMotor_End();  //brakes the motor
}

// when backward command is received, taking in the parameter of how many cm it should move
void moveBackwards(int distance) {
  initializeTick();
  initializeMotor_Start();
  distance = cmToTicks(distance);
  double currentSpeed = 0;
  boolean initialStatus = true;
  
  if (distance < 60) {
    currentSpeed = MOVE_MIN_SPEED;
  } else {
    currentSpeed = MOVE_MAX_SPEED;
  }
   
  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
    if (myPID.Compute()) {
      if (initialStatus) {
        //md.setSpeeds(0, currentSpeed);
        //delay(5);
        initialStatus = false;
      }
      md.setSpeeds(-(currentSpeed + speed_O), -(currentSpeed - speed_O));
    }
  }
  initializeMotorBack_End();  //brakes the motor
}

// ignore for now (thad)
// move backward faster
void moveBackwardsFast(int distance) {
  initializeTick();
  initializeMotor_Start();
  distance = cmToTicks(distance);
  double currentSpeed = 0;
  if (distance < 30) {
    currentSpeed = MOVE_MAX_SPEED;  //350
  } else {
    currentSpeed = MOVE_FAST_SPEED; //400
  }
  double offset = 0;
  long last_tick_R = 0;

  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
    if (distance - tick_R < 150)
      currentSpeed = 150;
    if ((tick_R - last_tick_R) >= 10 || tick_R == 0 || tick_R == last_tick_R) {
      last_tick_R = tick_R;
      offset += 0.1;
    }
    if (myPID.Compute() || tick_R == last_tick_R) {
      if (offset >= 1)
        md.setSpeeds(-(currentSpeed + speed_O), -(currentSpeed - speed_O));
      else
        md.setSpeeds(-(offset * (currentSpeed + speed_O)), -(offset * (currentSpeed - speed_O)));
    }
  }
  initializeMotor_End();  //brakes the motor
}

// when left command is received, taking in the parameter of how much angle it should rotate anticlockwise
void turnLeft(int angle) {
  //initializeMotor_Start();
  int i=0;    // for loop iterator
  double currentSpeed = TURN_MAX_SPEED;

  if (angle >= 90) {
    for (i = 0; i<angle; i+=90) {
      Serial.println(i);
      initializeTick();
      initializeMotor_Start();
      while (tick_R < TURN_TICKS_L || tick_L - 15 < TURN_TICKS_L) {
        if (myPID.Compute())
          md.setSpeeds(-(currentSpeed + speed_O), currentSpeed - speed_O);
      }
      initializeMotorTurn_End();   //brakes the motor
    }
  }
  if (angle - i > 0) {
    turnLeftDeg(angle-i);
  }
  initializeMotorTurn_End();   //brakes the motor
}


// will turn <90 degree
void turnLeftDeg(int angle) {
  int index = (angle-20)/5; // index is the index no. of the LEFTTICKS array of size 14
  int tick;
  if (index <= 14)
    tick = LEFTTICK[index];
    
  initializeMotor_Start();
  double currentSpeed = TURN_MAX_SPEED;
  initializeTick();
  while (tick_R < tick || tick_L < tick) {
    if (myPID.Compute())
      md.setSpeeds(-(currentSpeed + speed_O), currentSpeed - speed_O);
  }
}

// when right command is received, taking in the parameter of how much angle it should rotate clockwise
void turnRight(int angle) {
  initializeMotor_Start();
  int i=0;    // for loop iterator
  double currentSpeed = TURN_MAX_SPEED;

  if (angle >= 90) {
    for (i=0; i<angle; i=i+90) {
      initializeTick();
      while (tick_R < (TURN_TICKS_R) || tick_L -16 < (TURN_TICKS_R)) {
        if (myPID.Compute())
          md.setSpeeds((currentSpeed + speed_O), -(currentSpeed - speed_O));
      }
    }
  }
  if (angle - i > 0) {
    turnRightDeg(angle-i);
  }
  initializeMotor_End();   //brakes the motor
}

// will turn <90 degree
void turnRightDeg(int angle) {
  int index = (angle-20)/5; // index is the index no. of the LEFTTICKS array of size 14
  int tick;
  if (index <= 14)
    tick = RIGHTTICK[index];

  initializeMotor_Start();
  double currentSpeed = TURN_MAX_SPEED;
  initializeTick();
  while (tick_R < tick || tick_L < tick) {
    if (myPID.Compute())
      md.setSpeeds((currentSpeed + speed_O), -(currentSpeed - speed_O));
  }
}

//for enableInterrupt() function, to add the tick for counting 
void leftMotorTime() {
  tick_L++;
}

//for enableInterrupt() function, to add the tick for counting
void rightMotorTime() {
  tick_R++;
}

// to reset/initialize the ticks and speed for PID
void initializeTick() {
  tick_R = 0;
  tick_L = 0;
  speed_O = 0;
  previous_tick_R = 0;
}

// to reset/initialize the motor speed and brakes for PID
void initializeMotor_Start() {
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}

// brakes when moving forward (please revise) - thad
void initializeMotor_End() {
  md.setSpeeds(0, 0);
  //md.setBrakes(350, 347);
  //md.setBrakes(400, 400);
  
  for (int i = 200; i <400; i+=50) {
    md.setBrakes(i*1.1, i);
    delay(10);
  }
  
  delay(50);
}

// brakes when moving backward (please revise) - thad
void initializeMotorBack_End() {
  md.setSpeeds(0, 0);
  //md.setBrakes(350, 347);
  md.setBrakes(400, 400);
  /*
  for (int i = 200; i <400; i+=50) {
    md.setBrakes(i*2, i*0.98);
    delay(10);
  }
  */
  delay(50);
}

// brakes when turning left/right (please revise) - thad
void initializeMotorTurn_End() {
  md.setSpeeds(0, 0);
  //md.setBrakes(350, 347);
  md.setBrakes(400, 400);
  /*
  for (int i = 200; i <400; i+=50) {
    md.setBrakes(i*2, i);
    delay(20);
  }
  */
  delay(50);
}

// converting distance (cm) to ticks
int cmToTicks(int cm) {
  int dist = (cm / 10) - 1; //dist is the index no. of the TICKS array of size 10
  if (dist < 10)
    return TICKS[dist]; //TICKS[10] = {545, 1155, 1760, 2380, 2985, 3615, 4195, 4775, 5370};
  return 0;
}

/*
// printing movement message to android
void printMessage(String directionString, int value) {
  if (directionString.equals("forward") || directionString.equals("back")) {
    Serial.print("B{\"status\":[{\"status\":\"moving ");
    Serial.print(directionString);
    Serial.print(" by: ");
  } else if (directionString.equals("left") || directionString.equals("right")) {
    Serial.print("B{\"status\":[{\"status\":\"turning ");
    Serial.print(directionString);
    Serial.print(" by angle of: ");
  } else
    return;
  Serial.print(value);
  Serial.print("\"}]}");
  Serial.flush();
}*/

//use this function to check RPM of the motors
void testRPM(int M1Speed, int M2Speed){
  md.setSpeeds(M1Speed, M2Speed);  //setSpeeds(Motor1, Motor2)
  delay(1000);
  Serial.println(tick_R/562.25 * 60 );
  Serial.println(tick_L/562.25 * 60);
  tick_R = 0;
  tick_L = 0;
}

//to avoid 1x1 obstacle
void avoid(){

  while(1){
    moveForward(1*10);
    
    int frontIR1 = (int)getFrontIR1();
    int frontIR2 = (int)getFrontIR2();
    int frontIR3 = (int)getFrontIR3();
    int rightIR1 = (int)getRightIR1();
    int rightIR2 = (int)getRightIR2();
    int leftIR1 = (int)getLeftIR1();

    int flag = 0;
        
    if(frontIR2 == 1){  //Obstacle is in front of Front Center sensor
      Serial.println("Obstacle Detected by Front Center Sensor");
      delay(500);
      //turnRight(90);
      turnLeft(90);
      delay(500);
      moveForward(2*10);
      delay(500);
      //turnLeft(90);
      turnRight(90);
      delay(500);
      moveForward(4*10);
      delay(500);
      //turnLeft(90);
      turnRight(90);
      delay(500);
      moveForward(2*10);
      delay(500);
      //turnRight(90);
      turnLeft(90);
    }
    else if(frontIR1 == 1 && frontIR2 != 1){ //Obstacle is in front of Front Left sensor
      Serial.println("Obstacle Detected by Front Left Sensor");
      delay(500);
      turnRight(90);
      delay(500);
      moveForward(1*10);
      delay(500);
      turnLeft(90);
      delay(500);
      moveForward(4*10);
      delay(500);
      turnLeft(90);
      delay(500);
      moveForward(1*10);
      delay(500);
      turnRight(90);
    }
    else if(frontIR3 == 1 && frontIR2 != 1){ //Obstacle is in front of Front Right sensor
      Serial.println("Obstacle Detected by Front Right Sensor");
      delay(500);
      turnRight(90);
      delay(500);
      moveForward(3*10);
      delay(500);
      turnLeft(90);
      delay(500);
      moveForward(4*10);
      delay(500);
      turnLeft(90);
      delay(50000);
      moveForward(3*10);
      delay(5000);
      turnRight(90);
    }
    delay(500);
  }  
}

//to avoid 1x1 obstacle diagonally
void avoidDiagonal(){

  while(1){
    moveForward(1*10);
    
    int frontIR1 = (int)getFrontIR1();
    int frontIR2 = (int)getFrontIR2();
    int frontIR3 = (int)getFrontIR3();
    int rightIR1 = (int)getRightIR1();
    int rightIR2 = (int)getRightIR2();
    int leftIR1 = (int)getLeftIR1();

    int flag = 0;
        
    if(frontIR2 == 2){  //Obstacle is in front of Front Center sensor
      Serial.println("Obstacle Detected by Front Center Sensor");
      delay(500);
      turnLeft(45);
      delay(500);
      moveForwardTick(2546);
      delay(500);
      turnRight(45);
      turnRight(45);
      delay(500);
      moveForwardTick(2600);
      delay(500);
      turnLeft(45);
    }
    else if(frontIR1 == 1 && frontIR2 != 1){ //Obstacle is in front of Front Left sensor
      Serial.println("Obstacle Detected by Front Left Sensor");
      delay(500);
      turnRight(45);
      delay(500);
      moveForwardTick(580);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForwardTick(580);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForwardTick(580);
      delay(500);
      turnRight(45);
    }
    else if(frontIR3 == 1 && frontIR2 != 1){ //Obstacle is in front of Front Right sensor
      Serial.println("Obstacle Detected by Front Right Sensor");
      delay(500);
      turnRight(45);
      delay(500);
      moveForwardTick(580);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForwardTick(580);
      delay(500);
      turnLeft(45);
      delay(50000);
      moveForwardTick(580);
      delay(5000);
      turnRight(45);
    }
    delay(500);
  }  
}

// for moving diagonal
void moveForwardTick(int distance) {
  initializeTick();   // set all tick to 0
  initializeMotor_Start();  // set motor and brake to 0
  double currentSpeed = 0;
  
  if (distance < 60) {    // if number of tick to move < 60, move at a slower speed of 200
    currentSpeed = MOVE_MIN_SPEED;
  } else {                // if number of tick to move >= 60, move at the max speed of 350
    currentSpeed = MOVE_MAX_SPEED;
  }
  
  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
    if (myPID.Compute()) {
      md.setSpeeds(currentSpeed + speed_O, currentSpeed - speed_O);
    }
  }
  initializeMotor_End();  //brakes the motor
}

void alignRight() {
  delay(2);
  double diff = (readRightSensor_2() - 4.51) - (readRightSensor_1() -0.16);
  
  while (abs(diff) > 0.04)
  {
    if (diff < 0)
    {
      rotateRight(abs(diff*5), 1);
      Serial.println("First if");
    }
    else if (diff > 0)
    {
      rotateRight(abs(diff*5), -1);
      Serial.println("Second if");
    }
     diff = (readRightSensor_2() - 4.51) - (readRightSensor_1() -0.16);
     Serial.println(diff);
  
  }
}

void rotateRight(int distance, int direct) {
  initializeTick();
  initializeMotor_Start();
  double currentSpeed = ROTATE_MAX_SPEED;
  while (tick_R < distance) {
    if (myPID.Compute())
      md.setSpeeds(0, direct*(currentSpeed - speed_O));
  }
  initializeMotor_End();
}
