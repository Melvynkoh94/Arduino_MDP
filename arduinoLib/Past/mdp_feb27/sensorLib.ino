#include <SharpIR.h>
#include <RunningMedian.h>
//short range - 10-80cm
//long range - 20-150cm
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

/*
SharpIR frontIR_1(SharpIR::GP2Y0A21YK0F, A0); 
SharpIR frontIR_2(SharpIR::GP2Y0A21YK0F, A4);
SharpIR frontIR_3(SharpIR::GP2Y0A21YK0F, A2);
SharpIR rightIR_1(SharpIR::GP2Y0A21YK0F, A3);
SharpIR rightIR_2(SharpIR::GP2Y0A21YK0F, A5);
SharpIR leftIR_1(SharpIR::GP2Y0A02YK0F, A1);
 */

//declaration of our IR sensors. "GP2Y0A21YK0F" is just the model number of the SharpIR sensor
//A0-A5 == Arduino pin 1-6
SharpIR frontIR_1(SharpIR::GP2Y0A21YK0F, A0); //FRONT left  
SharpIR frontIR_2(SharpIR::GP2Y0A21YK0F, A5); //FRONT center
SharpIR frontIR_3(SharpIR::GP2Y0A21YK0F, A1); //FRONT right
SharpIR rightIR_1(SharpIR::GP2Y0A21YK0F, A3); //RIGHT back
SharpIR rightIR_2(SharpIR::GP2Y0A21YK0F, A2); //RIGHT front
SharpIR leftIR_1(SharpIR::GP2Y0A02YK0F, A4);  //LEFT 


RunningMedian frontIR1_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
RunningMedian frontIR2_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
RunningMedian frontIR3_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
RunningMedian rightIR1_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
RunningMedian rightIR2_Median = RunningMedian(NUM_SAMPLES_MEDIAN);
RunningMedian leftIR_1_Median = RunningMedian(NUM_SAMPLES_MEDIAN);

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

// read and return the median of (5*11) front left sensor values
int getFrontIR1() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    temp.add(readFrontSensor_1());
  }
  return temp.getMedian();
}

// read and return the median of (5*11) front center sensor values
int getFrontIR2() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    temp.add(readFrontSensor_2());
  }
  //while(true);
  return temp.getMedian();
}

// read and return the median of (5*11) front right sensor values
double getFrontIR3() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    temp.add(readFrontSensor_3());
  }
  return temp.getMedian();
}

// read and return the median of (5*11) right back sensor values
int getRightIR1() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    temp.add(readRightSensor_1());
  }
  return temp.getMedian();
}

// read and return the median of (5*11) right front sensor values
int getRightIR2() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    temp.add(readRightSensor_2());
  }
  return temp.getMedian();
}

// read and return the median of (5*11) left front sensor values
int getLeftIR1() {
  RunningMedian temp = RunningMedian(5);
  for (int n=0; n<5; n++) {
    temp.add(readLeftSensor_1());
  }
  return temp.getMedian();
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

// determine which grid it belongs
int shortGrid(int distance, int offset) {
  //Serial.print(distance);
  //Serial.print(",");
  if (distance < 6)
    return 1;
  else if (distance < 17)
    return 2;
  else if (distance < (28 - offset))
    return 3;
  else
    return -1;
}

// read and return the median of (11) front left sensor values
int readFrontSensor_1() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    double irDistance = frontIR_1.getDistance() - 12;   // for offseting the value for sensor distance (original Thad <-10>)
  
    //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
    //anything nearer than that to the IR sensors is default set to 0

    //Serial.println(irDistance);
    
    frontIR1_Median.add(shortGrid(irDistance, 0));    // add in the array  
    if (frontIR1_Median.getCount() == NUM_SAMPLES_MEDIAN) {
      frontIR1_Value = frontIR1_Median.getMedian();
    }
  }
  return frontIR1_Value;
}

// read and return the median of (11) front center sensor values
int readFrontSensor_2() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    double irDistance = frontIR_2.getDistance() - 8;   // for offseting the value for sensor distance <-9>
  
    //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
    //anything nearer than that to the IR sensors is default set to 0

    //Serial.println(irDistance);
    
    frontIR2_Median.add(shortGrid(irDistance, 3));    // add in the array  
    if (frontIR2_Median.getCount() == NUM_SAMPLES_MEDIAN) {
      frontIR2_Value = frontIR2_Median.getMedian();
    }
  }
  return frontIR2_Value;
}

// read and return the median of (11) front right sensor values
int readFrontSensor_3() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    double irDistance = frontIR_3.getDistance() - 10;   // for offseting the value for sensor distance <-11>
  
    //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
    //anything nearer than that to the IR sensors is default set to 0

    //Serial.println(irDistance);
    
    frontIR3_Median.add(shortGrid(irDistance, 0));    // add in the array  
    if (frontIR3_Median.getCount() == NUM_SAMPLES_MEDIAN) {
      frontIR3_Value = frontIR3_Median.getMedian();
    }
  }
  return frontIR3_Value;
}

// read and return the median of (11) right back sensor values
int readRightSensor_1() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    double irDistance = rightIR_1.getDistance() - 7;   // for offseting the value for sensor distance <-11>
  
    //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
    //anything nearer than that to the IR sensors is default set to 0

    //Serial.println(irDistance);
    
    rightIR1_Median.add(shortGrid(irDistance, 0));    // add in the array  
    if (rightIR1_Median.getCount() == NUM_SAMPLES_MEDIAN) {
      rightIR1_Value = rightIR1_Median.getMedian();
    }
  }
  return rightIR1_Value;
}

// read and return the median of (11) right front sensor values
int readRightSensor_2() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    double irDistance = rightIR_2.getDistance() - 11;   // for offseting the value for sensor distance <-11>
  
    //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
    //anything nearer than that to the IR sensors is default set to 0

    //Serial.println(irDistance);
    
    rightIR2_Median.add(irDistance);    // add in the array  
    if (rightIR2_Median.getCount() == NUM_SAMPLES_MEDIAN) {
      rightIR2_Value = rightIR2_Median.getMedian();
    }
  }
  return rightIR2_Value;
}

//Real is 17cm, IR gives 20cm reading   - **check please, and conclude
//anything nearer than 20cm is rubbish  - **check please, and conclude
// read and return the median of (11) left sensor values
int readLeftSensor_1() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    double irDistance = leftIR_1.getDistance() - 11;   // for offseting the value for sensor distance <-11>
  
    //reference point at 3x3 grid boundary (30cmx30cm) is 0cm
    //anything nearer than that to the IR sensors is default set to 0

    //Serial.println(irDistance);
    
    leftIR_1_Median.add(irDistance);    // add in the array  
    if (leftIR_1_Median.getCount() == NUM_SAMPLES_MEDIAN) {
      leftIR1_Value = leftIR_1_Median.getMedian();
    }
  }
  return leftIR1_Value;
}
