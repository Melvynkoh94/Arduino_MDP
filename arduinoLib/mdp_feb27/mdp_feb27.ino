// the setup routine runs once when you press reset
void setup() {
  // initialize serial communication
  setupSerialConnection();
  setupMotorEncoder();
  setupSensorInterrupt();
  setupPID();
  // make arduino pause execution for all code for the stated milliseconds
  // for the analog-to-digital converter to settle after the last reading
  delay(20);  
}

// the loop routine runs over and over again forever
void loop() {
  delay(0);
  
  // if not connected
  if (!Serial) {
    //Serial.println("Waiting for connection");
  }

  //testRPM(400, 400);

  // for hardcode string
  // to store char array for movement value
  char gridMoveValueChar[5];
  // to store movement value in int
  int gridMoveValueInt= 0;
  int count = 0;
  char movement;
  String test = "A4";
  int strLen = test.length();
  int pos = 0;

  // while data is available in the serial buffer
  while (strLen > pos) {
    // read the next character of the serial buffer
    char character = test.charAt(pos);
    pos++;
    count = 0;
    char nextChar = test.charAt(pos);
    if (character == '|')
      continue;
    if (nextChar == '|' || nextChar == '\0') {
      gridMoveValueChar[count] = '1';
      count++;
      pos++;
    } else {
      while (nextChar != '|' && nextChar != '\0') {
        gridMoveValueChar[count] = nextChar;
        pos++;
        count++;
        nextChar = test.charAt(pos); 
      }
      pos++;
    }
    gridMoveValueChar[count] = '\0';
    gridMoveValueInt = atoi(gridMoveValueChar);
    
    switch (character) {
      // move forward 
      case 'W':
        moveForward(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        delay(2000);
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
        pos = pos + 1;
        break;
      // move backward (for fastestpath or clear straight path)
      case 'S':
        moveBackwards(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        break;
      // move backward fast
      case 'B':
        moveBackwardsFast(gridMoveValueInt * 10);
        delay(10);
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
        pos = pos + 1;
        break;
      // to get current sensor reading
      case 'U':
        printSensorReading();
        pos = pos + 1;
        break;
    }
  }

  /*
  
// for receiving string from rpi

  // to store char array for movement value
  char gridMoveValueChar[5];
  // to store movement value in int
  int gridMoveValueInt = 0;
  int count = 0;
  char movement;

  // while data is available in the serial buffer
  while (Serial.available() > 0){
    char character = Serial.read();
    char nextChar = Serial.peek();

    if (nextChar == '|') {
      gridMoveValueChar[count] = '1';
      count++;
    } else {
        while (nextChar != '|' && nextChar != '\0') {
        nextChar = Serial.read();
        gridMoveValueChar[count] = nextChar;
        count++;
        nextChar = Serial.peek(); 
      }
    }
    gridMoveValueChar[count] = '\0';
    gridMoveValueInt = atoi(gridMoveValueChar);
    
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
        delay(10);
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
      case 'U':
        printSensorReading();
        break;
      case '|':
        break;
      }
  }
  */
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
}
