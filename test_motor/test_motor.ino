#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>

volatile long M1Ticks = 0, M2Ticks = 0;
//motors pin
const int M1A = 3; //Right
const int M1B = 5;
const int M2A = 11;
const int M2B = 13; //Left

int M1Turn = 0; // for RPM counts left Motor
int M2Turn = 0; // for RPM counts right Motor

double motor_readings[50];

#define MAX_SPEED 400 
#define SPEED 300

DualVNH5019MotorShield md;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  md.init();
  Serial.println("Motor Test");
  enableInterrupt(M1A, encoder1, RISING);
  enableInterrupt(M2A, encoder2, RISING);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //M1Ticks = 0 ;
  //M2Ticks = 0 ;
  //md.setSpeeds(400,0);
  //delay(750);
 
  //md.setM1Speed(400); //these 2 lines can be done in 1 line using md.setSpeeds() function 
  //md.setM2Speed(400);

  testRPM(400,400);
  //turnRight();
  //turnLeft();

  /*
  md.setSpeeds(400,400);  //setSpeeds(Motor1, Motor2)
  delay(1000);
  Serial.println(M1Ticks/562.25 * 60 );
  Serial.println(M2Ticks/562.25 * 60);
  M1Ticks = 0;
  M2Ticks = 0;
  */

}


//Function for making a left turn on a 2x2 grid.
void turnLeft(){
  md.setSpeeds(200,400);
  delay(750);
  md.setBrakes(400,400);
  delay(100);
  md.setBrakes(0,0);
}

//Function for making a right turn on a 2x2 grid.
void turnRight(){
  md.setSpeeds(400,200);
  delay(750);
  md.setBrakes(400,400);
  delay(100);
  md.setBrakes(0,0);
}

void testRPM(int M1Speed, int M2Speed){
  md.setSpeeds(M1Speed, M2Speed);  //setSpeeds(Motor1, Motor2)
  delay(1000);
  Serial.println(M1Ticks/562.25 * 60 );
  Serial.println(M2Ticks/562.25 * 60);
  M1Ticks = 0;
  M2Ticks = 0;
  
}

void encoder1() 
{
  M1Ticks++;
}

void encoder2() 
{ 
  M2Ticks++;
}
