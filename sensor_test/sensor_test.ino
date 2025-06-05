#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

Servo armServo; // declare servo class

// for wifi kill switch
const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";

WiFiUDP udp;
unsigned int localUdpPort = 2390;
char incomingPacket[255];
bool killSwitch = false;

// sensor values
float distanceCM = 0.0;
float distanceCMleft = 0.0;
float distanceCMright = 0.0;
const int threshold = 400;
const int numSensors = 9;

// sensor pins
const int trigPin = 12;
const int echoPin = 13;
const int leftIRPin = A1;
const int rightIRPin = A2;
const int servoPin = 43;

// led strip pins
const int ledPin1 = 52;
const int ledPin2 = 53;


// the vertical 1x9 board
const int sensorPinsV[numSensors] = {32,33,34,35,36,37,38,38,40};
unsigned int sensorValuesV[numSensors];

// the horizontal 1x9 board
const int sensorPinsH[numSensors] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
unsigned int sensorValuesH[numSensors];

// for left and right 1x2 sensors
int numSensors2 = 2;
const int sensorPinsRIGHT[2] = {44, 45};
unsigned int sensorValuesRIGHT[2];

const int sensorPinsLEFT[2] = {46, 47};
unsigned int sensorValuesLEFT[2];

// function to read the reflectance sensors
void readSensorArray(const int pins[], unsigned int values[]) {
  unsigned long timeout = 3000;

  // set all pins to output and drive to high
  for (int i = 0; i < numSensors; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }

  // wait!
  delayMicroseconds(10);

  // set pins to input
  for (int i = 0; i < numSensors; i++) {
    pinMode(pins[i], INPUT);
  }

  // measure discharge time
  unsigned long startTime = micros();
  for (int i = 0; i < numSensors; i++) {
    values[i] = timeout; // initialise to max timeout
  }

  while ((micros() - startTime) < timeout) {
    for (int i = 0; i < numSensors; i++) {
      if (digitalRead(pins[i]) == LOW && values[i] == timeout) { // if it finished discharging
        values[i] = micros() - startTime; // measuring how long each pin stays high before going low
        // sets the value to the sensor values to be used in the loop
      }
    }
  }
}

// voltage --> distance function to return the distance from the wall in cm
float voltageToDistance(float voltage) {
  if (voltage > 0.3) {
    return 12.08 * pow(voltage, -1.058);  // from THE formula: scaling factor is 12.08, and invert the value
  } else {
    return 80.0;  // max or default value
  }
}

// IR read
float readIRDistance(int analogPin) {
  int sensorValue = analogRead(analogPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  return voltageToDistance(voltage);  // IR sensor returns a voltage so must change into distance in cm
}

// ultrasonic read
float readUltrasonicDistance() {
  // trigger sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  float dist = duration * 0.034 / 2;

  if (dist <= 0 || dist > 80) {
    return 80.0;  // max or default value
  }
  return dist;  // return distance in cm
}


// wall detection
void checkWallsAndAct() {
  // read distance ad ultrasonic sensor values
  distanceCMleft = readIRDistance(leftIRPin);
  distanceCMright = readIRDistance(rightIRPin);
  distanceCM = readUltrasonicDistance();

  // if wall is closer than set threshold values
  bool wallLeft = distanceCMleft < 7;
  bool wallRight = distanceCMright < 7;
  bool wallFront = distanceCM < 17;

  // no motor functions since it's just sensor test
  if (wallFront) { // if a wall is in front, decide whether to turn right or left or go back
    if (wallLeft && !wallRight) { // if a wall is left but not right -> turn right
      Serial.println("Obstacle ahead + left wall => TURN RIGHT");
    } else if (wallRight && !wallLeft) {  // if a wall is right but not left -> turn left
      Serial.println("Obstacle ahead + right wall => TURN LEFT");
    } else if (wallLeft && wallRight) { // if a wall is right and left -> go back
      Serial.println("Walls on all sides => STOP or BACKTRACK");
    } else {  // same as above
      Serial.println("Obstacle ahead, space on sides => BACKTRACK");
    }
  } else { 
    if (wallLeft && wallRight) {
      Serial.println("Between walls, no front obstacle => GO FORWARD");
    } else {
      Serial.println("Free space => GO FORWARD");
    }
  }
}

void checkKillSwitch() {  // check for wifi
  Serial.println("Checking for kill switch...");
  int packetSize = udp.parsePacket();
  Serial.print("Packet size: ");
  Serial.println(packetSize);

  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
    Serial.print("Received UDP packet: ");
    Serial.println(incomingPacket);
    if (String(incomingPacket) == "Stop") {
      killSwitch = true;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // ultrasonic setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // arm setup
  armServo.attach(servoPin);
  armServo.write(180);  // calibrated to be up
  delay(1000);

  // led setup
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);

}

void loop() {
  // wifi stop
  checkKillSwitch();
  if (killSwitch) {
    Serial.println("Kill switch activated! Robot stopped.");
  }

  // distance sensors
  distanceCMleft = readIRDistance(leftIRPin);
  distanceCMright = readIRDistance(rightIRPin);
  distanceCM = readUltrasonicDistance();

  // print sensor values
  Serial.print("L: ");
  Serial.print(distanceCMleft, 1);
  Serial.print(" cm | F: ");
  Serial.print(distanceCM, 1);
  Serial.print(" cm | R: ");
  Serial.print(distanceCMright, 1);
  Serial.println(" cm");

  checkWallsAndAct();
  Serial.println();
  delay(500);

  readSensorArray(sensorPinsV, sensorValuesV); // vertical
  readSensorArray(sensorPinsH, sensorValuesH); // horizontal
  readSensorArray(sensorPinsRIGHT, sensorValuesRIGHT); // right
  readSensorArray(sensorPinsLEFT, sensorValuesLEFT); // left

  Serial.print("Vertical:\t");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValuesV[i]);
    Serial.print('\t');
  }
  Serial.println();

  Serial.print("Horizontal:\t");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValuesH[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println();

  Serial.print("right:\t");
  for (int i = 0; i < numSensors2; i++) {
    Serial.print(sensorValuesRIGHT[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println();

  Serial.print("left:\t");
  for (int i = 0; i < numSensors2; i++) {
    Serial.print(sensorValuesLEFT[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println();

  // DEADEND IDENTIFICATION
  // are horizontal white?
  bool allHorizontalWhite = true;
  for (int i = 0; i < numSensors; i++) {
    if (sensorValuesH[i] > threshold) {
      allHorizontalWhite = false;
      break;
    }
  }

  // are vertical white?
  bool allVerticalWhite = true;
  for (int i = 0; i < numSensors; i++) {
    if (sensorValuesV[i] > threshold) {
      allVerticalWhite = false;
      break;
    }
  }

  // are all vertical black?
  bool allVerticalBlack = true;
    for (int i = 0; i < numSensors; i++) {
      if (sensorValuesV[i] < threshold) {
        allVerticalBlack = false;
        break;
      }
  }

  int blackMiddleCount = 0;
  for (int i = 4; i <= 6; i++) {
    if (sensorValuesH[i] >= threshold) {
      blackMiddleCount++;
    }
  }

  if (allVerticalBlack && blackMiddleCount) {
    Serial.println("Black line - GO");
    // CODE TO GO FORWARD
  }

  if (allVerticalWhite && allHorizontalWhite) {
    Serial.println("Dead end detected!");
    // CODE TO BACKTRACK
  }


  // FORK IDENTIFICATION
  bool leftDetected = sensorValuesH[0] > threshold;
  bool rightDetected = sensorValuesH[numSensors - 1] > threshold;
  if (allVerticalWhite && leftDetected && rightDetected) {
    Serial.println("Fork detected!");
    // CODE TO MAKE IT GO RIGHT
  }


  // SMOOTH CURVE HANDLING
  bool rightCurve = (sensorValuesH[7] > threshold && sensorValuesH[8] > threshold);
  bool leftCurve = (sensorValuesH[0] > threshold && sensorValuesH[1] > threshold);

  if (allVerticalWhite && rightCurve) {
    Serial.println("Right curve detected!");
    delay(100);

    while (true) {
      // move forward but reduce left motor speed
      delay(100);
      readSensorArray(sensorPinsV, sensorValuesV);
      int count = 0;
      for (int i = 2; i <= 6; i++) {
        if (sensorValuesV[i] < threshold) count++;
      }
      if (count >= 4) break;
    }

    // MOVE FORWARD
    return;
  }

  if (allVerticalWhite && leftCurve) {
    Serial.println("Left curve detected!");
    delay(100);

    while (true) {
      // move forward but reduce right motor speed
      delay(100);
      readSensorArray(sensorPinsV, sensorValuesV);
      int count = 0;
      for (int i = 2; i <= 6; i++) {
        if (sensorValuesV[i] < threshold) count++;
      }
      if (count >= 4) break;
    }

    // MOVE FORWARD
    return;
  }


  // DOTTED LINE DETECTION
  int middleHorizontalWhite = 0;
  for (int i = 3; i <= 5; i++) {
    if (sensorValuesH[i] < threshold) {
      middleHorizontalWhite++;
    }
  }

  bool allMiddleVerticalBlack = true;
  for (int i = 2; i <= 6; i++) {
    if (sensorValuesV[i] > threshold) {
      allMiddleVerticalBlack = false;
      break;
    }
  }

  if (allMiddleVerticalBlack && middleHorizontalWhite) {
    Serial.println("Dotted line detected - treat as normal line");
    // MOVE FORWARD
  }


  delay(100);
}

