#include <MsTimer2.h>
#include <EnableInterrupt.h>
#include <DualVNH5019MotorShield.h>
//#include <ArduinoPIDLibrary.h>
#include <PID_v1.h>

const int LEFT_PULSE = 3;           // M1 - LEFT motor pin number on Arduino board
const int RIGHT_PULSE = 11;         // M2 - RIGHT motor pin number on Arduino board
const int MOVE_FAST_SPEED = 400;    //if target distance more than 30cm 
const int MOVE_MAX_SPEED = 350;     //if target distance more than 60cm
const int MOVE_MIN_SPEED = 200;     //if target distance less than 60cm
const int TURN_MAX_SPEED = 260;     //change this value to calibrate turning. If the rotation overshoots, decrease the speed 
const int ROTATE_MAX_SPEED = 150;   //used in rotateLeft() and rotateRight()
const int TURN_TICKS_L = 785;       //change this aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeqwj encoder ticks value to calibrate left turn (increased this from 770)
const int TURN_TICKS_R = 763;       //change this right encoder ticks value to calibrate right turn 

//TICKS[0] for general cm -> ticks calibration. 
//TICKS[1-9] with specific distance (by grids) e.g. distance=5, TICKS[5] 
const int TICKS[10] = {545, 1155, 1760, 2380, 2985, 3615, 4195, 4775, 5370};  
const double DIST_WALL_CENTER_BOX = 1.58;   //for aligning to the front wall/obstacle. Used in alignFront()
const double kp = 7.35, ki = 1.25, kd = 0;  //change values for moving straight line 
//const double kp = 7.35, ki = 1.25, kd = 5;
//const double kp = 7.90, ki = 1.25, kd = 0;
//const double kp = 7.00, ki = 1.25, kd = 0;
//const double kp = 7.35, ki = 2, kd = 0;


int TENCM_TICKS_OFFSET = 0; //not used

//for PID
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

//not used 
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

void moveForward(int distance) {
  initializeTick();
  initializeMotor_Start();
  distance = cmToTicks(distance);
  double currentSpeed = 0;
  if (distance < 60) {
    currentSpeed = MOVE_MIN_SPEED;
  } else {
    currentSpeed = MOVE_MAX_SPEED;
  }
  double offset = 0;
  int last_tick_R = 0;  

  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
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

void moveBackwards(int distance) {
  initializeTick();
  initializeMotor_Start();
  distance = cmToTicks(distance);
  double currentSpeed = 0;
  if (distance < 60) {
    currentSpeed = MOVE_MIN_SPEED;
  } else {
    currentSpeed = MOVE_MAX_SPEED;
  }
  double offset = 0;
  long last_tick_R = 0;

  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
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

void moveForwardFast(int distance) {
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

//use this function in alignFront() for aligning to the front wall/obstacle
void moveForwardCalibrate(int distance) {
  initializeTick();
  initializeMotor_Start();
  distance = cmToTicksCalibrate(distance);
  if (distance < 3)
    return;
  double currentSpeed = 0;
  //  Serial.print("HELLO: ");
  //  Serial.println(distance);
  if (distance < 60) {
    currentSpeed = MOVE_MIN_SPEED;
  } else {
    currentSpeed = MOVE_MAX_SPEED;
  }
  double offset = 0;
  int last_tick_R = 0;

  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
    if ((tick_R - last_tick_R) >= 10 || tick_R == 0 || tick_R == last_tick_R) {
      last_tick_R = tick_R;
      offset += 0.1;
    }
    if (myPID.Compute() || tick_R == last_tick_R) {
      if (offset >= 1 || distance < 110)
        md.setSpeeds(currentSpeed + speed_O, currentSpeed - speed_O);
      else
        md.setSpeeds(offset * (currentSpeed + speed_O), offset * (currentSpeed - speed_O));
    }
  }
  initializeMotor_End();  //brakes the motor
}


void moveBackwardsCalibrate(int distance) {
  initializeTick();
  initializeMotor_Start();
  distance = cmToTicksCalibrate(distance);
  if (distance < 3)
    return;
  double currentSpeed = 0;
  //  Serial.print("HELLO: ");
  //  Serial.println(distance);
  if (distance < 60) {
    currentSpeed = MOVE_MIN_SPEED;
  } else {
    currentSpeed = MOVE_MAX_SPEED;
  }
  double offset = 0;
  long last_tick_R = 0;

  //error checking feedback in a while loop
  while (tick_R <= distance || tick_L <= distance) {
    if ((tick_R - last_tick_R) >= 10 || tick_R == 0 || tick_R == last_tick_R) {
      last_tick_R = tick_R;
      offset += 0.1;
    }
    if (myPID.Compute() || tick_R == last_tick_R) {
      if (offset >= 1 || distance < 110)
        md.setSpeeds(-(currentSpeed + speed_O), -(currentSpeed - speed_O));
      else
        md.setSpeeds(-(offset * (currentSpeed + speed_O)), -(offset * (currentSpeed - speed_O)));
    }
  }
  initializeMotor_End();  //brakes the motor
}

//Rotation anti-clockwise on its own axis based on the 'value' number
//Use 'value' in the for loop in main script for the angle of rotation (for checklist purposes)
/*  if value == 3, robot turns 270 degrees (3 grids)
for (int k = 0; k < value; k++) {
          turnRight();
        }   */
void turnLeft() {
  initializeTick();
  initializeMotor_Start();
  double currentSpeed = TURN_MAX_SPEED; //260
  double offset = 0;

  while (tick_R < TURN_TICKS_L || tick_L < TURN_TICKS_L) {
    //    offset = computePID();
    if (myPID.Compute())
      md.setSpeeds(-(currentSpeed + speed_O), currentSpeed - speed_O);
  }
  initializeMotor_End();  //brakes the motor
  initializeLeftTurnEnd();
}

//Rotation clockwise on its own axis
void turnRight() {
  initializeTick();
  initializeMotor_Start();
  double currentSpeed = TURN_MAX_SPEED;
  double offset = 0;

  while (tick_R < (TURN_TICKS_R + 10) || tick_L < (TURN_TICKS_R + 10)) {
    //    offset = computePID();
    if (myPID.Compute())
      md.setSpeeds((currentSpeed + speed_O), -(currentSpeed - speed_O));
  }
  initializeMotor_End();   //brakes the motor
  initializeRightTurnEnd();
}

void rotateLeft(int distance) {
  initializeTick();
  initializeMotor_Start();
  double currentSpeed = ROTATE_MAX_SPEED;
  double offset = 0;
  if (distance < 3)
    return;
  while (tick_R < distance || tick_L < distance) {
    //    offset = computePID();
    if (myPID.Compute())
      md.setSpeeds(-(currentSpeed + speed_O), currentSpeed - speed_O);
  }
  initializeMotor_End();  //brakes the motor
}

void rotateRight(int distance) {
  initializeTick();
  initializeMotor_Start();
  double currentSpeed = ROTATE_MAX_SPEED;
  double offset = 0;
  if (distance < 3)
    return;
  while (tick_R < distance || tick_L < distance) {
    //    offset = computePID();
    if (myPID.Compute())
      md.setSpeeds((currentSpeed + speed_O), -(currentSpeed - speed_O));
  }
  initializeMotor_End();  //brakes the motor
}

void alignRight() {
  delay(2);
  double diff = getRightIR1() - getRightIR2();
  int rotated = 0;
  while (abs(diff) >= 0.4 && getRightIR1_Block() == getRightIR2_Block() && rotated < 15) {
    rotated++;
    if (diff > 0) {
      rotateLeft(abs(diff * 5));
      diff = getRightIR1() - getRightIR2();
      if (getRightIR1_Block() != getRightIR2_Block()) {
        rotateRight(abs(diff * 4));
        diff = getRightIR1() - getRightIR2();
      }
    } else {
      rotateRight(abs(diff * 5));
      diff = getRightIR1() - getRightIR2();
      if (getRightIR1_Block() != getRightIR2_Block()) {
        rotateLeft(abs(diff * 4));
        diff = getRightIR1() - getRightIR2();
      }
    }
    delay(1);
  }
  delay(2);
}

void alignFront() {
  delay(2);
  double diff_dis;
  int moved = 0;
  double previous_turn = 0;
  if (getFrontIR1_Block() != 1 || getFrontIR3_Block() != 1 ) {
    do {
      diff_dis = getMin(getFrontIR1(), getFrontIR2(), getFrontIR3()) - DIST_WALL_CENTER_BOX; //DIST_WALL_CENTER_BOX = 1.58cm
      if (diff_dis > 0) {
        moveForwardCalibrate(abs(diff_dis));
      } else {
        moveBackwardsCalibrate(abs(diff_dis));
      }
      delay(2);
      diff_dis = getMin(getFrontIR1(), getFrontIR2(), getFrontIR3()) - DIST_WALL_CENTER_BOX;
      moved++;
    } while (abs(diff_dis) > 0.2 && moved < 15);
    return;
  }
  delay(2);
  moved = 0;
  double diff = getFrontIR1() - getFrontIR3();
  while (abs(diff) >= 0.4 && moved < 15) {
    moved++;
    previous_turn = abs(diff * 5);
    if (diff > 0) {
      rotateLeft(previous_turn);
      diff = getFrontIR1() - getFrontIR3();
      if (getFrontIR1_Block() != getFrontIR3_Block()) {
        rotateRight(previous_turn);
        break;
      }
    } else {
      rotateRight(previous_turn);
      diff = getFrontIR1() - getFrontIR3();
      if (getFrontIR1_Block() != getFrontIR3_Block()) {
        rotateRight(previous_turn);
        break;
      }
    }
    delay(2);
  }
  delay(2);
  moved = 0;
  do {
    diff_dis = getMin(getFrontIR1(), getFrontIR2(), getFrontIR3()) - DIST_WALL_CENTER_BOX;
    if (diff_dis > 0) {
      moveForwardCalibrate(abs(diff_dis));
    } else {
      moveBackwardsCalibrate(abs(diff_dis));
    }
    delay(2);
    diff_dis = getMin(getFrontIR1(), getFrontIR2(), getFrontIR3()) - DIST_WALL_CENTER_BOX;
    moved++;
  } while (abs(diff_dis) > 0.2 && moved < 20);
  moved = 0;
  delay(2);
  diff = getFrontIR1() - getFrontIR3();
  while (abs(diff) >= 0.4 && moved < 15) {
    moved++;
    previous_turn = abs(diff * 5);
    if (diff > 0) {
      rotateLeft(previous_turn);
      diff = getFrontIR1() - getFrontIR3();
      if (getFrontIR1_Block() != getFrontIR3_Block())
        rotateRight(previous_turn);
    } else {
      rotateRight(previous_turn);
      diff = getFrontIR1() - getFrontIR3();
      if (getFrontIR1_Block() != getFrontIR3_Block())
        rotateRight(previous_turn);
    }
    delay(2);
  }
}


void initializeLeftTurnEnd() {
  //  rotateRight(8);
  //  rotateLeft(6);
}


void initializeRightTurnEnd() {
  //  rotateLeft(8);
  //  rotateRight(6);
}

//used in alignFront()
double getMin(double f1, double f2, double f3) {
  if (f1 < f2) {
    if (f1 < f3) {
      return f1;
    } else {
      return f3;
    }
  } else {
    if (f2 < f3) {
      return f2;
    } else {
      return f3;
    }
  }
}

//for enableInterrupt() function 
void leftMotorTime() {
  tick_L++;
}

//for enableInterrupt() function
void rightMotorTime() {
  tick_R++;
}

void initializeTick() {
  tick_R = 0;
  tick_L = 0;
  speed_O = 0;
  previous_tick_R = 0;
}

void initializeMotor_Start() {
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}
//brakes the motor
void initializeMotor_End() {
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400);
  delay(5);
}

int cmToTicks(int cm) {
  int dist = (cm / 10) - 1; //dist is the index no. of the TICKS array of size 10
  if (dist < 10)
    return TICKS[dist]; //TICKS[10] = {545, 1155, 1760, 2380, 2985, 3615, 4195, 4775, 5370};
  return 0;
}

int cmToTicksCalibrate(int cm) {
  double ret = ((double)cm * TICKS[0] / 10.0) + 0.5;  //TICKS[10] = {545, 1155, 1760, 2380, 2985, 3615, 4195, 4775, 5370};
  return ret;
}

//use this function to check RPM of the motors
void testRPM(int M1Speed, int M2Speed){
  md.setSpeeds(M1Speed, M2Speed);  //setSpeeds(Motor1, Motor2)
  delay(1000);
  Serial.println(tick_R/562.25 * 60 );
  Serial.println(tick_L/562.25 * 60);
  tick_R = 0;
  tick_L = 0;
}


void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}
