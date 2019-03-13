// the setup routine runs once when you press reset
void setup() {
  // initialize serial communication
  setupSerialConnection();
  setupMotorEncoder();
  setupSensorInterrupt();
  setupPID();
  // make arduino pause execution for all code for the stated milliseconds
  // for the analog-to-digital converter to settle after the last reading

  //                       _oo0oo_
  //                      o8888888o
  //                      88" . "88
  //                      (| -_- |)
  //                      0\  =  /0
  //                    ___/`---'\___
  //                  .' \\|     |// '.
  //                 / \\|||  :  |||// \
  //                / _||||| -:- |||||- \
  //               |   | \\\  -  /// |   |
  //               | \_|  ''\---/''  |_/ |
  //               \  .-\__  '-'  ___/-. /
  //             ___'. .'  /--.--\  `. .'___
  //          ."" '<  `.___\_<|>_/___.' >' "".
  //         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
  //         \  \ `_.   \_ __\ /__ _/   .-` /  /
  //     =====`-.____`.___ \_____/___.-`___.-'=====
  //                       `=---='
  //     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  delay(20);  
}

// the loop routine runs over and over again forever
void loop() {
  //delay(2);

  // if not connected
  if (!Serial) {
    //Serial.println("Waiting for connection");
  }
  //testRPM(400, 400);
/*
  // for hardcode string
  // to store char array for movement value
  char gridMoveValueChar[100];
  char movement;
  String test = "W"; //M|D|W|W|W|A|P|W|W|W|W|W|W|W|W|W|A|P|W|W|W|A|P|W|W|W|W|W|W|W|W|W|
  String gridMoveValueString;
  int gridMoveValueInt;
  int count;
  int strLen = test.length();
  int pos = 0;

  // while data is available in the serial buffer
  while (strLen > pos) {
    
    // to store movement value in int
    gridMoveValueInt = 0;
    gridMoveValueString = "";
    count = 0;
    
    // read the next character of the serial buffer
    char character = test.charAt(pos);
    pos++;
    char nextChar = test.charAt(pos);
    if (character == '|')
      continue;
    if (character == '\0' || character == '\n')
      break;
    if (nextChar == '|' || nextChar == '\0') {
      gridMoveValueString += '1';
      //gridMoveValueChar[count] = '1';
      count++;
      pos++;
    } else {
      while (nextChar != '|' && nextChar != '\0') {
        gridMoveValueString += (char)nextChar;
        //gridMoveValueChar[count] = nextChar;
        pos++;
        count++;
        nextChar = test.charAt(pos); 
      }
      pos++;
    }
    //gridMoveValueChar[count] = '\0';
    //gridMoveValueInt = atoi(gridMoveValueChar);
    gridMoveValueInt = gridMoveValueString.toInt();
    
    switch (character) {
      // move forward 
      case 'W':
        moveForward(gridMoveValueInt * 10);
        delay(10);
        printDistanceReading();
        printSensorReading();
        delay(1000);
        break;
      // move forward fast (for fastestpath or clear straight path)
      case 'F':
        moveForwardFast(gridMoveValueInt * 10);
        delay(10);
        printDistanceReading();
        printSensorReading();
        break;
      // turn left
      case 'A':
        if (gridMoveValueInt <= 12)     // lesser or equal to 12, will be seen as number of 90 degrees to be turned (max 1080 deg)
          turnLeft(gridMoveValueInt*90);
        else
          turnLeft(gridMoveValueInt);   // more than 12, will be seen as degrees to be turned
        delay(10);
        printDistanceReading();
        //printSensorReading();
        pos = pos + 1;
        break;
      // move backward (for fastestpath or clear straight path)
      case 'S':
        moveBackwards(gridMoveValueInt * 10);
        delay(10);
        printDistanceReading();
        printSensorReading();
        break;
      // move backward fast
      case 'B':
        moveBackwardsFast(gridMoveValueInt * 10);
        delay(10);
        printDistanceReading();
        printSensorReading();
        break;
      // turn right
      case 'D':
        if (gridMoveValueInt <= 12)     // lesser or equal to 12, will be seen as number of 90 degrees to be turned (max 1080 deg)
          turnRight(gridMoveValueInt*90);
        else
          turnRight(gridMoveValueInt);  // more than 12, will be seen as degrees to be turned
        delay(10000);
        printDistanceReading();
        printSensorReading();
        pos = pos + 1;
        break;
      // to get current sensor reading
      case 'U':
        printDistanceReading();
        printSensorReading();
        pos = pos + 1;
        break;
      case 'P':
        alignRight(gridMoveValueInt);
        delay(10);
        break;
      case 'O':
        alignFront();
        delay(10);
        break;
      case 'N':
        forwardCalibration(gridMoveValueInt);
        break;
      case 'M':
        alignRight(1);
        turnRight(90);
        alignRight(3);
        alignFront();
        turnRight(90);
        alignFront();
        turnLeft(90);
        alignRight(2);
        alignFront();
        turnLeft(90);
        alignRight(1);
        delay(1000);
        break;
    }
  }
  */
  
  //for rceiving string from rpi 
  // to store char array for movement value
  char gridMoveValueChar[100];  
  int gridMoveValueInt;
  String gridMoveValueString;
  int count;
  
  int dummy;

  // while data is available in the serial buffer
  while (Serial.available() > 0){

    dummy = Serial.peek();
    if (dummy > 90 || dummy < 49)
    {
      //Serial.println(dummy);
      dummy = Serial.read();
      continue;
    }
    
    gridMoveValueString = "";
    count = 0;
    char character = Serial.read();
    char nextChar = Serial.peek();

    //char gridMoveValueInt = Serial.read();
    //if (character == '|')
     // continue;

    if (character == '|')
      continue;
    if (character == '\0' || character == '\n')
      break;
    if (nextChar == '|') {
      gridMoveValueString += '1';
      //gridMoveValueChar[count] = '1';
      count++;
    } else {
        while (nextChar != '|' && nextChar != '\0' && nextChar != '\n') {
          nextChar = Serial.read();
          gridMoveValueString += (char)nextChar;
          //gridMoveValueChar[count] = nextChar;
          count++;
          nextChar = Serial.peek(); 
        }
    }
    //Serial.println(character);
    //Serial.flush();
    //gridMoveValueChar[count] = '\0';
    //gridMoveValueInt = atoi(gridMoveValueChar);
    gridMoveValueInt = gridMoveValueString.toInt();
  
    // read the next character of the serial buffer
    switch (character) {
      // move forward 
      case 'W':
        moveForward(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        break;
      // move forward fast (for fastestpath or clear straight path)
      case 'F':
        moveForwardFast(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        break;
      // turn left
      case 'A':
        if (gridMoveValueInt <= 12)     // lesser or equal to 12, will be seen as number of 90 degrees to be turned (max 1080 deg)
          turnLeft(gridMoveValueInt*90);
        else
          turnLeft(gridMoveValueInt);   // more than 12, will be seen as degrees to be turned
        delay(10);
        printSensorReading();
        break;
      // move backward
      case 'S':
        moveBackwards(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        break;
      // move backward fast
      case 'B':
        moveBackwardsFast(gridMoveValueInt * 10);
        delay(100);
        printSensorReading();
        break;
      // turn right
      case 'D':
        if (gridMoveValueInt <= 12)     // lesser or equal to 12, will be seen as number of 90 degrees to be turned (max 1080 deg)
          turnRight(gridMoveValueInt*90);
        else
          turnRight(gridMoveValueInt);  // more than 12, will be seen as degrees to be turned
        delay(10);
        printSensorReading();
        break;
      // to get current sensor reading
      case 'K':
        //Serial.println("Inside K");
        printSensorReading();
        break;
      case 'P':
        alignRight(gridMoveValueInt);
        delay(10);
        printSensorReading();
        break;
      case 'O':
        alignFront();
        delay(10);
        printSensorReading();
        break;  
      case 'M':
        alignRight(1);
        turnRight(90);
        alignRight(3);
        alignFront();
        turnRight(90);
        alignFront();
        turnLeft(90);
        alignRight(2);
        alignFront();
        turnLeft(90);
        alignRight(1);
        break;
      case '|':
      case '\0':
      case '\n':
        break;     
    }
  }
}

// serial codes
void setupSerialConnection() {
  // initialize serial communication at 9600 bits per seconds
  Serial.begin(115200);
  // deadlock if not connected
  while (!Serial);
}

void printSensorReading() {
  // print sensor reading to serial monitor
  Serial.print("XF1:");
  Serial.print((int)getFrontIR1()); // print front-left sensor distance
  Serial.print("|F2:");  
  Serial.print((int)getFrontIR2()); // print front-center sensor distance
  Serial.print("|F3:");
  Serial.print((int)getFrontIR3()); // print front-right sensor distance
  Serial.print("|R1:");
  Serial.print((int)getRightIR1()); // print right-back sensor distance
  Serial.print("|R2:");  
  Serial.print((int)getRightIR2()); // print right-front sensor distance
  Serial.print("|L1:");
  Serial.print((int)getLeftIR1());  // print left long range sensor distance
  Serial.print("|\n");
  // flush waits for transmission of outoing serial data to complete
  Serial.flush();
  delay(10);
}

void printDistanceReading() {
  // print sensor reading to serial monitor
  Serial.print("                                  F1:");
  Serial.print(readFrontSensor_1()); // print front-left sensor distance
  Serial.print("|F2:");  
  Serial.print(readFrontSensor_2()); // print front-center sensor distance
  Serial.print("|F3:");
  Serial.print(readFrontSensor_3()); // print front-right sensor distance
  Serial.print("|R1:");
  Serial.print(readRightSensor_1()); // print right-back sensor distance
  Serial.print("|R2:");  
  Serial.print(readRightSensor_2()); // print right-front sensor distance
  Serial.print("|L1:");
  Serial.print(readLeftSensor_1());  // print left long range sensor distance
  Serial.print("|\n");
  // flush waits for transmission of outoing serial data to complete
  Serial.flush();
  delay(10);
}
