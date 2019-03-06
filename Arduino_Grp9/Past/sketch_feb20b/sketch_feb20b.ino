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
  delay(2);
  //Serial.println("Loop Start");
  
  // if not connected
  if (!Serial) {
    //Serial.println("Waiting for connection");
  }

  // for hardcode string
  /*
  // to store char array of move value
  char gridMoveValueChar[2];
  // to store move value in int
  int gridMoveValueInt = 0;
  String test = "W4|A";
  int strLen = test.length();
  int pos = 0;

  Serial.println(test);
  // while data is available in the serial buffer
  while (strLen > pos) {
    // read the next character of the serial buffer
    switch (test.charAt(pos)) {
      // move forward 
      case 'W':
        gridMoveValueChar[0] = test.charAt(pos+1);
        gridMoveValueChar[1] = '\0';
        gridMoveValueInt = atoi(gridMoveValueChar);
        moveForward(gridMoveValueInt * 10);
        Serial.print("Move forward: ");
        Serial.println(gridMoveValueInt);
        delay(10);
        printSensorReading();
        pos = pos + 2;
        break;
      // turn left
      case 'A':
        Serial.print("Turn left");
        turnLeft();
        delay(10);
        printSensorReading();
        pos = pos + 1;
        break;
      // move backward
      case 'S':
        gridMoveValueChar[0] = test.charAt(pos+1);
        gridMoveValueChar[1] = '\0';
        gridMoveValueInt = atoi(gridMoveValueChar);
        Serial.print("Move backward: ");
        Serial.println(gridMoveValueInt);
        moveBackwards(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        pos = pos + 2;
        break;
      // turn right
      case 'D':
        Serial.println("Turn right");
        turnRight();
        delay(10);
        printSensorReading();
        pos = pos + 1;
        break;
      // to get current sensor reading
      case 'U':
        printSensorReading();
        pos = pos + 1;
        break;
      case '|':
        pos = pos + 1;
        break;
    }
  }*/
  
  // for receiving string from rpi

  // to store char array of move value
  char gridMoveValueChar[2];
  // to store move value in int
  int gridMoveValueInt = 0;
  char movement;

 // while data is available in the serial buffer
  while (Serial.available() > 0){
    //if(Serial.available()){
       char character = Serial.read();
    // read the next character of the serial buffer
    switch (character) {
    // move forward 
      case 'W':
        movement = Serial.read();
        gridMoveValueChar[0] = movement;
        gridMoveValueChar[1] = '\0';
        gridMoveValueInt = atoi(gridMoveValueChar);
        moveForward(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        break;
      // turn left
      case 'A':
        turnLeft();
        delay(10);
        printSensorReading();
        break;
      // move backward
      case 'S':
        movement = Serial.read();
        gridMoveValueChar[0] = movement;
        gridMoveValueChar[1] = '\0';
        gridMoveValueInt = atoi(gridMoveValueChar);
        moveBackwards(gridMoveValueInt * 10);
        delay(10);
        printSensorReading();
        break;
      // turn right
      case 'D':
        turnRight();
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
  Serial.print((int)getFrontIR1());
  //Serial.print((int));  // print front-left sensor distance;
  Serial.print("|F2:");  
  Serial.print((int)getFrontIR2());
  //Serial.print((int));  // print front-center sensor distance;
  Serial.print("|F3:");
  Serial.print((int)getFrontIR3());  
  //Serial.print((int));  // print front-right sensor distance;
  Serial.print("|R1:");
  Serial.print((int)getRightIR1());  
  //Serial.print((int));  // print right-back sensor distance;
  Serial.print("|R2:");  
  Serial.print((int)getRightIR2());
  //Serial.print((int));  // print right-front sensor distance;
  Serial.print("|L1:");  
  Serial.print((int)getLeftIR1());
  //Serial.print((int));  // print left long range sensor distance;
  Serial.print("|\n");
  // flush waits for transmission of outoing serial data to complete
  Serial.flush();
  
}
