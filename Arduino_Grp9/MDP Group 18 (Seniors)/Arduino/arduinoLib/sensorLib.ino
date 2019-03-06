#include <SharpIR.h>
#include <RunningMedian.h>


const int MAX_SMALL_SENSOR = 80;
const int MAX_BIG_SENSOR = 150;
const int NUM_SAMPLES_MEDIAN = 15;

double frontIR1_Diffs[] = {5.50, 14.75, 24.30, 35.00};
double frontIR2_Diffs[] = {5.70, 15.65, 24.75, 39.00};
double frontIR3_Diffs[] = {5.15, 14.55, 23.50, 36.00};


double rightIR1_Diffs[] = {6.90, 17.05, 26.75, 41.00};
double rightIR2_Diffs[] = {7.5, 17.75, 28.75, 47.00};

//double leftIR1_Diffs[] = {20.25, 24.15, 32.00, 40.55, 49.00, 54.00};
double leftIR1_Diffs[] = {20.25, 24.55, 33.2, 42.25, 51.2, 57.20};

double frontIR1_Value = 0, frontIR2_Value = 0, frontIR3_Value = 0;
int  frontIR1_Block = 0, frontIR2_Block = 0, frontIR3_Block = 0;
double rightIR1_Value = 0, rightIR2_Value = 0, leftIR1_Value = 0;
int rightIR1_Block = 0, rightIR2_Block = 0, leftIR1_Block = 0;


SharpIR frontIR_1(SharpIR::GP2Y0A21YK0F, A0);
SharpIR frontIR_2(SharpIR::GP2Y0A21YK0F, A4);
SharpIR frontIR_3(SharpIR::GP2Y0A21YK0F, A2);
SharpIR rightIR_1(SharpIR::GP2Y0A21YK0F, A3);
SharpIR rightIR_2(SharpIR::GP2Y0A21YK0F, A5);
SharpIR leftIR_1(SharpIR::GP2Y0A02YK0F, A1);
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

void readSensors() {
  readFrontSensor_1();
  readFrontSensor_2();
  readFrontSensor_3();
  readRightSensor_1();
  readRightSensor_2();
  readLeftSensor_1();
}


double getFrontIR1() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    readFrontSensor_1();
  }
  return frontIR1_Value;
}
double getFrontIR2() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    readFrontSensor_2();
  }
  return frontIR2_Value;
}
double getFrontIR3() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    readFrontSensor_3();
  }
  return frontIR3_Value;
}
double getRightIR1() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    readRightSensor_1();
  }
  return rightIR1_Value;
}
double getRightIR2() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    readRightSensor_2();
  }
  return rightIR2_Value;
}
double getLeftIR1() {
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    readLeftSensor_1();
  }
  return leftIR1_Value;
}

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
}

void readFrontSensor_1() {
  double irDistance = frontIR_1.getDistance() - 9;
//  Serial.println(irDistance + 9);
  frontIR1_Median.add(irDistance);
  if (frontIR1_Median.getCount() >= NUM_SAMPLES_MEDIAN) {
    if (abs(frontIR1_Median.getHighest() - frontIR1_Median.getLowest()) > 40) {
      frontIR1_Value = MAX_SMALL_SENSOR;
    } else {
      frontIR1_Value = frontIR1_Median.getMedian();
    }
  }
  for (int m = 0; m < 3; m++) {
    if (frontIR1_Value <= frontIR1_Diffs[m]) {
      frontIR1_Block = m + 1;
      return;
    }
  }
  frontIR1_Block = 9;
}

void readFrontSensor_2() {
  double irDistance = frontIR_2.getDistance() - 9;
  frontIR2_Median.add(irDistance);
  if (frontIR2_Median.getCount() >= NUM_SAMPLES_MEDIAN) {
    if (abs(frontIR2_Median.getHighest() - frontIR2_Median.getLowest()) > 40) {
      frontIR2_Value = MAX_SMALL_SENSOR;
    } else {
      frontIR2_Value = frontIR2_Median.getMedian();
    }
  }
  for (int m = 0; m < 3; m++) {
    if (frontIR2_Value <= frontIR2_Diffs[m]) {
      frontIR2_Block = m + 1;
      return;
    }
  }
  frontIR2_Block = 9;
}

void readFrontSensor_3() {
  double irDistance = frontIR_3.getDistance() - 9;
  frontIR3_Median.add(irDistance);
  if (frontIR3_Median.getCount() >= NUM_SAMPLES_MEDIAN) {
    if (abs(frontIR3_Median.getHighest() - frontIR3_Median.getLowest()) > 40) {
      frontIR3_Value = MAX_SMALL_SENSOR;
    } else {
      frontIR3_Value = frontIR3_Median.getMedian();
    }
  }
  for (int m = 0; m < 3; m++) {
    if (frontIR3_Value <= frontIR3_Diffs[m]) {
      frontIR3_Block = m + 1;
      return;
    }
  }
  frontIR3_Block = 9;
}

void readRightSensor_1() {
  double irDistance = rightIR_1.getDistance() - 9;
  rightIR1_Median.add(irDistance);
  if (rightIR1_Median.getCount() >= NUM_SAMPLES_MEDIAN) {
    if (abs(rightIR1_Median.getHighest() - rightIR1_Median.getLowest()) > 40) {
      rightIR1_Value = MAX_SMALL_SENSOR;
    } else {
      rightIR1_Value = rightIR1_Median.getMedian();
    }
  }
  for (int m = 0; m < 3; m++) {
    if (rightIR1_Value <= rightIR1_Diffs[m]) {
      rightIR1_Block = m + 1;
      return;
    }
  }
  rightIR1_Block = 9;
}

void readRightSensor_2() {
  double irDistance = rightIR_2.getDistance() - 9;
  rightIR2_Median.add(irDistance);
  if (rightIR2_Median.getCount() >= NUM_SAMPLES_MEDIAN) {
    if (abs(rightIR2_Median.getHighest() - rightIR2_Median.getLowest()) > 40) {
      rightIR2_Value = MAX_SMALL_SENSOR;
    } else {
      rightIR2_Value = rightIR2_Median.getMedian();
    }
  }
  for (int m = 0; m < 3; m++) {
    if (rightIR2_Value <= rightIR2_Diffs[m]) {
      rightIR2_Block = m + 1;
      return;
    }
  }
  rightIR2_Block = 9;
}

void readLeftSensor_1() {
  double irDistance = leftIR_1.getDistance();
  leftIR_1_Median.add(irDistance);
  if (leftIR_1_Median.getCount() >= NUM_SAMPLES_MEDIAN) {
    if (abs(leftIR_1_Median.getHighest() - leftIR_1_Median.getLowest()) > 40) {
      leftIR1_Value = MAX_BIG_SENSOR;
    } else {
      leftIR1_Value = leftIR_1_Median.getMedian();
    }
  }
  for (int m = 0; m < 6; m++) {
    if (leftIR1_Value <= leftIR1_Diffs[m]) {
      leftIR1_Block = m + 1;
      return;
    }
  }
  leftIR1_Block = 9;
}
