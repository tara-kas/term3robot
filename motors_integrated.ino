#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Motoron.h>
#include <Wire.h>

MotoronI2C mc(0x10);

// wifi
const char* ssid = "sofiya";                                                                           
const char* password = "12345678";                                                                    
WiFiUDP udp;  
unsigned int localUdpPort = 2000;
char incomingPacket[255];
bool killSwitch = false;

// pin allocation
const int distanceSensorPin = A0; //front 
float distanceCM = 0.0;
float distanceCMleft = 0.0;
float distanceCMright = 0.0;
const int leftWallSensorPin = A1;
const int rightWallSensorPin = A2;
const int threshold = 400; 
const int numSensors = 9;

// reflective sensors 
// the vertical board 
const int sensorPinsV[numSensors] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
unsigned int sensorValuesV[numSensors];

// the horizontal board
const int sensorPinsH[numSensors] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
unsigned int sensorValuesH[numSensors];


// motor functions
void moveForward() {
  mc.setSpeed(1, 200);
  mc.setSpeed(2, 200);
  mc.setSpeed(3, 200);
}

void stopMotors() {
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  mc.setSpeed(3, 0);
}

void moveBackward(int speed) {
  mc.setSpeed(1, -speed);
  mc.setSpeed(2, -speed);
  mc.setSpeed(3, -speed);
}

void turnLeft(int speed) {
  mc.setSpeed(1, -speed);  
  mc.setSpeed(2, speed);   
  mc.setSpeed(3, 200);    
}

void turnRight(int speed) {
  mc.setSpeed(1, speed);   
  mc.setSpeed(2, -speed); 
  mc.setSpeed(3, 200); 
}



void readSensorArray(const int pins[], unsigned int values[]) {
  unsigned long timeout = 3000; 

  // set all pins to output and drive to high
  for (int i = 0; i < numSensors; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }

  // wait
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
      if (digitalRead(pins[i]) == LOW && values[i] == timeout) {  // if it finished discharging
        values[i] = micros() - startTime; // measuring how long each pin stays high before going low 
      }
    }
  }
}

int calculateLineError(const unsigned int values[]) {
  long weightedSum = 0;
  int sum = 0;

  for (int i = 0; i < numSensors; i++) {
    int position = i - 4;  
    if (values[i] < threshold) { 
      weightedSum += position;
      sum++;
    }
  }

  if (sum == 0) {
    return 0; 
  }

  return weightedSum / sum; 
}


void readDistance() {
  int sensorValue = analogRead(distanceSensorPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  if (voltage > 0.3) {
    distanceCM = 12.08 * pow(voltage, -1.058);
  } else {
    distanceCM = 80;
  }
}

void checkWallsAndAct() {
  int leftSensor = analogRead(leftWallSensorPin);
  int rightSensor = analogRead(rightWallSensorPin);
  float voltageLeft = leftSensor * (5.0 / 1023.0);
  float voltageRight = rightSensor * (5.0 / 1023.0);

  if (voltageLeft > 0.3) {
    distanceCMleft = 12.08 * pow(voltageLeft, -1.058);
  } else {
    distanceCMleft = 80;
  }

  if (voltageRight > 0.3) {
    distanceCMright = 12.08 * pow(voltageRight, -1.058);
  } else {
    distanceCMright = 80;
  }

  bool wallLeft = distanceCMleft < 10;
  bool wallRight = distanceCMright < 10;

  if (distanceCM <= 10) {
    if (wallLeft && !wallRight) {
      Serial.println("Wall ahead & wall on LEFT => TURN RIGHT");
    } else if (wallRight && !wallLeft) {
      Serial.println("Wall ahead & wall on RIGHT => TURN LEFT");
    } else if (wallRight && wallLeft) {
      Serial.println("Obstacles on all 3 sides");
    } else {
      Serial.println("Obstacle ahead but no side walls => BACKTRACK");
    }
  } else {
    if (wallLeft && wallRight) {
      Serial.println("No obstacle ahead, but between walls => FORWARD");
    } else {
      Serial.println("Free space => FORWARD");
    }
  }
}

void checkKillSwitch() {
  int packetSize = udp.parsePacket();
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
  Wire.begin();

  // motoron setup
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();

  //wifi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());
  udp.begin(localUdpPort);

}


void loop() {
  // wifi stop
  checkKillSwitch();
  if (killSwitch) {
    Serial.println("Kill switch activated! Robot stopped.");
    stopMotors();
    while (true) {
      delay(10000); 
    }
  }


  readDistance();
  Serial.print("Distance: ");
  Serial.print(distanceCM);
  Serial.println(" cm");

  checkWallsAndAct(); 
  delay(500);

  readSensorArray(sensorPinsV, sensorValuesV); // vertical 
  readSensorArray(sensorPinsH, sensorValuesH); // horizontal 

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

  if (allVerticalWhite && allHorizontalWhite) {
    Serial.println("Dead end detected!");
    moveBackward();
  }


  // FORK IDENTIFICATION
  bool leftDetected = sensorValuesH[0] > threshold; 
  bool rightDetected = sensorValuesH[numSensors - 1] > threshold; 
  if (allVerticalWhite && leftDetected && rightDetected) {
    Serial.println("Fork detected!");
    turnRight();
  }


  // SMOOTH CURVE HANDLING
  bool rightCurve = (sensorValuesH[7] > threshold && sensorValuesH[8] > threshold);
  bool leftCurve  = (sensorValuesH[0] > threshold && sensorValuesH[1] > threshold);

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

    moveForward();
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

    moveForward();
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
    moveForward();
  }


  delay(100);
}

