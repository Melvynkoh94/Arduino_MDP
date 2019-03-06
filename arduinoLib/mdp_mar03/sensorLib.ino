#include <SharpIR2.h> //for long-range
#include <SharpIR.h>  //for short-range
#include <RunningMedian.h>
//short range - 10-80cm (OPTIMAL 10-30CM)
//long range - 20-150cm (OPTIMAL 20 -60CM)
//our short range sensors work best between 10-30 cm. 
//30cm onwards there'll be larger discrepancies (discrepancies must be within 10%)


const int MAX_SMALL_SENSOR = 80;    // max distance for small sensor is 80cm
const int MAX_BIG_SENSOR = 150;     // max distance for big sensor is 150cm
const int NUM_SAMPLES_MEDIAN = 11;  // getting a maximum sample of 15

/*
double frontIR1_Diffs[] = {5.50, 14.75, 24.30, 35.00};  //use in readFrontSensor_1() to compare with frontIR1_Value
double frontIR2_Diffs[] = {5.70, 15.65, 24.75, 39.00};
double frontIR3_Diffs[] = {5.15, 14.55, 23.50, 36.00};


double rightIR1_Diffs[] = {6.90, 17.05, 26.75, 41.00};
double rightIR2_Diffs[] = {7.5, 17.75, 28.75, 47.00};

//double leftIR1_Diffs[] = {20.25, 24.15, 32.00, 40.55, 49.00, 54.00};
double leftIR1_Diffs[] = {20.25, 24.55, 33.2, 42.25, 51.2, 57.20};
*/

int frontIR1_Value = 0, frontIR2_Value = 0, frontIR3_Value = 0;
//int frontIR1_Block = 0, frontIR2_Block = 0, frontIR3_Block = 0;
int rightIR1_Value = 0, rightIR2_Value = 0, leftIR1_Value = 0;
//int rightIR1_Block = 0, rightIR2_Block = 0, leftIR1_Block = 0;


//declaration of our IR sensors. "GP2Y0A21YK0F" is just the model number of the SharpIR sensor
//A0-A5 == Arduino pin 1-6

SharpIR frontIR_1(SharpIR::GP2Y0A21YK0F, A0); //FRONT left  
SharpIR frontIR_2(SharpIR::GP2Y0A21YK0F, A5); //FRONT center
SharpIR frontIR_3(SharpIR::GP2Y0A21YK0F, A1); //FRONT right
SharpIR rightIR_1(SharpIR::GP2Y0A21YK0F, A3); //RIGHT back
SharpIR rightIR_2(SharpIR::GP2Y0A21YK0F, A2); //RIGHT front
//SharpIR leftIR_1(SharpIR::GP2Y0A02YK0F, A4);  //LEFT long-range, it's GP2Y0A02YK0F, not GP2Y0A21YK0F!!
//SharpIR leftIR_1(SharpIR::GP2Y0A21YK0F, A4);
SharpIR2 leftIR_1(A4, 20150);  //LEFT 

void setupSensorInterrupt() {
  //  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  //  //  ADCSRA |= bit (ADPS0) | bit (ADPS2);// 32  prescaler
  //  ADCSRA |= bit (ADPS2); // 16  prescaler
  //    MsTimer2::set(35, readSensors);
  //    MsTimer2::start();
}

//this function not really used. 
//It is only used in above setupSensorInterrupt() 
/*
void readSensors() {
  readFrontSensor_1();
  readFrontSensor_2();
  readFrontSensor_3();
  readRightSensor_1();
  readRightSensor_2();
  readLeftSensor_1();
}*/

// read and return the median of (5*11) front left sensor values in grid distance
int getFrontIR1() {
  int median = readFrontSensor_1();
  return (shortGrid(median, 2, 3, 10));
}

// read and return the median of (5*11) front center sensor values in grid distance
int getFrontIR2() {
  int median = readFrontSensor_2();
  return (shortGrid(median, -2, 0, 0));
}

// read and return the median of (5*11) front right sensor values in grid distance
double getFrontIR3() {
  int median = readFrontSensor_3();
  return (shortGrid(median, 0, 0, 5));
}

// read and return the median of (5*11) right back sensor values in grid distance
int getRightIR1() {
  int median = readRightSensor_1();
  return (shortGrid(median, -5, -3, -1));
}

// read and return the median of (5*11) right front sensor values in grid distance
int getRightIR2() {
  int median = readRightSensor_2();
  return (shortGrid(median, 0, 1, 4));
}

// read and return the median of (5*11) left front sensor values in grid distance
int getLeftIR1() {
  int median = readLeftSensor_1();
  return (longGrid(median));
}

// read and return the median of (55) front left sensor values in cm
int readFrontSensor_1() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    RunningMedian frontIR1_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
    for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
      double irDistance = frontIR_1.getDistance() - 12;   // for offseting the value for sensor distance (original Thad <-10>)
    
      //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
      //anything nearer than that to the IR sensors is default set to 0
  
      //Serial.println(irDistance);
      
      frontIR1_Median.add(irDistance);    // add in the array  
      if (frontIR1_Median.getCount() == NUM_SAMPLES_MEDIAN) {
        frontIR1_Value = frontIR1_Median.getMedian();
      }
    }
    temp.add(frontIR1_Value);
  }
  return temp.getMedian();
}

// read and return the median of (55) front center sensor values in cm
int readFrontSensor_2() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    RunningMedian frontIR2_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
    for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
      double irDistance = frontIR_2.getDistance() - 8;   // for offseting the value for sensor distance <-9>
    
      //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
      //anything nearer than that to the IR sensors is default set to 0
  
      //Serial.println(irDistance);
      
      frontIR2_Median.add(irDistance);    // add in the array  
      if (frontIR2_Median.getCount() == NUM_SAMPLES_MEDIAN) {
        frontIR2_Value = frontIR2_Median.getMedian();
      }
    }
    temp.add(frontIR2_Value);
  }
  return temp.getMedian();
}

// read and return the median of (55) front right sensor values in cm
int readFrontSensor_3() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    RunningMedian frontIR3_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
    for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
      double irDistance = frontIR_3.getDistance() - 10;   // for offseting the value for sensor distance <-11>
    
      //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
      //anything nearer than that to the IR sensors is default set to 0
  
      //Serial.println(irDistance);
      
      frontIR3_Median.add(irDistance);    // add in the array  
      if (frontIR3_Median.getCount() == NUM_SAMPLES_MEDIAN) {
        frontIR3_Value = frontIR3_Median.getMedian();
      }
    }
    temp.add(frontIR3_Value);
  }
  return temp.getMedian();
}

// read and return the median of (55) right back sensor values in cm
int readRightSensor_1() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    RunningMedian rightIR1_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
    for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
      double irDistance = rightIR_1.getDistance() - 7;   // for offseting the value for sensor distance <-11>
    
      //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
      //anything nearer than that to the IR sensors is default set to 0
  
      //Serial.println(irDistance);
      
      rightIR1_Median.add(irDistance);    // add in the array  
      if (rightIR1_Median.getCount() == NUM_SAMPLES_MEDIAN) {
        rightIR1_Value = rightIR1_Median.getMedian();
      }
    }
    temp.add(rightIR1_Value);
  }
  return temp.getMedian();
}

// read and return the median of (55) right front sensor values in cm
int readRightSensor_2() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    RunningMedian rightIR2_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
    for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
      double irDistance = rightIR_2.getDistance() - 11;   // for offseting the value for sensor distance <-11>
    
      //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
      //anything nearer than that to the IR sensors is default set to 0
  
      //Serial.println(irDistance);
      rightIR2_Median.add(irDistance);
      //rightIR2_Median.add(shortGrid(irDistance, -4, 0, 0));    // add in the array  
      if (rightIR2_Median.getCount() == NUM_SAMPLES_MEDIAN) {
        //if (rightIR2_Median.
        rightIR2_Value = rightIR2_Median.getMedian();
      }
    }
    temp.add(rightIR2_Value);
  }
  return temp.getMedian();
}

//Real is 17cm, IR gives 20cm reading   - **check please, and conclude
//anything nearer than 20cm is rubbish  - **check please, and conclude
// read and return the median of (55) left sensor values in cm
int readLeftSensor_1() {
  double irDistance = leftIR_1.distance();
  /*
  RunningMedian temp = RunningMedian(5);
  
  for (int n=0; n<5; n++) {
    RunningMedian leftIR_1_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
    for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
      //double irDistance = leftIR_1.getDistance();   // for offseting the value for sensor distance <-11>
      double irDistance = leftIR_1.distance();
      //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
      //anything nearer than that to the IR sensors is default set to 0
  
      //Serial.println(irDistance);
      
      //leftIR_1_Median.add(longGrid(irDistance));    // add in the array  
      leftIR_1_Median.add(irDistance);    // add in the array  
      if (leftIR_1_Median.getCount() == NUM_SAMPLES_MEDIAN) {
        leftIR1_Value = leftIR_1_Median.getMedian();
      }
    }
    if(21 <leftIR1_Value < 24){
      leftIR1_Value += 5;
    }
    else if ( leftIR1_Value > 42){
      leftIR1_Value -= 5;
    }
    //leftIR1_Value = -1;
    temp.add(leftIR1_Value);
  }
  
  return temp.getMedian();
  */
  return irDistance;
}

// determine which grid it belongs for short sensor
int shortGrid(int distance, int offset1, int offset2,  int offset3) {
  //Serial.print(distance);
  //Serial.print(",");
  if (distance < 6 - offset1)
    return 1;
  else if (distance < 17 - offset2)
    return 2;
  else if (distance < (28 - offset3))
    return 3;
  else
    return -1;
}

// determine which grid it belongs for long sensor (edit accordingly, see how far it can detect accurately - thad)
int longGrid(int distance) {
  //Serial.println(distance);
  //Serial.print(",");
  if (distance <= 21)
    return 1;
  else if (distance <= 27)
    return 2;
  else if (distance <= 37)
    return 3;
  else if (distance <= 47)
    return 4;
  else if (distance <= 57)
    return 5;
  else if (distance <= 69)
    return 6;
  else if (distance <= 78)
    return 7;
  else
    return -1;
  //return distance;
}

/*
int getFrontIR1_Block() {
  return frontIR1_Block;
}
int getFrontIR2_Block() {
  return frontIR2_Block;
}
int getFrontIR3_Block() {
  return frontIR3_Block;
}
int getRightIR1_Block() {
  return rightIR1_Block;
}
int getRightIR2_Block() {
  return rightIR2_Block;
}
int getLeftIR1_Block() {
  return leftIR1_Block;
}*/
