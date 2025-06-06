#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Motoron.h>
#include <Servo.h>
#include <Adafruit_DotStar.h>

Servo myServo;

// LEDs (declaration code taken from adafruit strandtest.ino example in Dotstar library)
#define NUMPIXELS 5
#define DATAPIN 4
#define CLOCKPIN 5
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);  // initialise

// wifi setup
const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";

WiFiUDP udp;
unsigned int localUdpPort = 2390;
char incomingPacket[255];
bool killSwitch = false;

// motoron object
MotoronI2C mc1(0x14);
MotoronI2C mc2(0x10);

// mech kill switch initialisation
const int buttonPin = 2;
bool robotRunning = false;
bool lastButtonReading = HIGH;  // button reading value
bool lastDebouncedState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// distance sensor values
float distanceCM = 0.0;
float distanceCMleft = 0.0;
float distanceCMright = 0.0;

// reflectance sensor values
const int threshold = 400;
const int numSensors = 9;
int lastKnownError = 0;
int lastError = 0;
float D_filtered = 0.0;
float D_alpha = 0.6;
unsigned long deadEndStartTime = 0;  // global or static in loop
bool deadEndActive = false;
unsigned long lastLineSeenTime = 0;
const unsigned long lineLostTimeout = 300;  // milliseconds to tolerate line loss

// ultrasonic sensor pins
const int trigPin = 13;
const int echoPin = 12;

// servo pin
const int servoPin = 43;

// the vertical board
const int sensorPinsV[numSensors] = { 32, 33, 34, 35, 36, 37, 38, 39, 40 };
unsigned int sensorValuesV[numSensors];

// the horizontal board
const int sensorPinsH[numSensors] = { 22, 23, 24, 25, 26, 27, 28, 29, 30 };
unsigned int sensorValuesH[numSensors];

// side sensors
int numSensors2 = 2;
const int sensorPinsRIGHT[2] = { 44, 45 };
const int sensorPinsLEFT[2] = { 46, 47 };
unsigned int sensorValuesRIGHT[2];
unsigned int sensorValuesLEFT[2];
unsigned int leftSideSensorValue = 0;
unsigned int rightSideSensorValue = 0;

void turnLEDOn() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 255, 255, 255);  // set each led in the strip to white (255,255,255)
  }
  strip.show();  // update leds
}


void turnLEDOff() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0, 0, 0);  // set all led off
  }
  strip.show();  // update led
}


// move forward: all motors rotate forward
void moveForward(int speed) {
  mc1.setSpeed(1, speed);  // front left
  mc1.setSpeed(2, speed);  // front right
  mc1.setSpeed(3, speed);  // middle left
  mc2.setSpeed(1, speed);  // middle right
  mc2.setSpeed(2, speed);  // left back
  mc2.setSpeed(3, speed);  // right back
}

// move backward: all motors rotate in reverse
void moveBackward(int speed) {
  mc1.setSpeed(1, -speed);
  mc1.setSpeed(2, -speed);
  mc1.setSpeed(3, -speed);
  mc2.setSpeed(1, -speed);
  mc2.setSpeed(2, -speed);
  mc2.setSpeed(3, -speed);
}

// turn left: front wheels rotate in opposite directions, rear wheel stops
void turnLeft(int speed) {
  mc1.setSpeed(1, -speed);  // front left wheel reverse
  mc1.setSpeed(2, speed);   // front right wheel forward
  mc1.setSpeed(3, -speed);
  mc2.setSpeed(1, speed);
  mc2.setSpeed(2, -speed);
  mc2.setSpeed(3, speed);
}

// turn right: front wheels rotate in opposite directions, rear wheel stops
void turnRight(int speed) {
  mc1.setSpeed(1, speed);   // front left wheel forward
  mc1.setSpeed(2, -speed);  // front right wheel reverse
  mc1.setSpeed(3, speed);
  mc2.setSpeed(1, -speed);
  mc2.setSpeed(2, speed);
  mc2.setSpeed(3, -speed);
}

// stop all wheels
void stopMotors() {
  mc1.setSpeed(1, 0);
  mc1.setSpeed(2, 0);
  mc1.setSpeed(3, 0);
  mc2.setSpeed(1, 0);
  mc2.setSpeed(2, 0);
  mc2.setSpeed(3, 0);
}

// line following control
int calculateLineError() {
  long weightedSum = 0;
  int total = 0;

  for (int i = 0; i < 9; i++) {  // 9 for the reflectance sensor arrays
    int value = sensorValuesH[i] > threshold ? 1000 : 0;
    weightedSum += (long)value * (i * 1000);
    total += value;
  }

  if (total < 5) {
    // no black detected — possibly a gap (dotted line)
    // return the last known direction to keep moving smoothly
    return lastKnownError;
  }

  int position = weightedSum / total;
  int center = ((numSensors - 1) * 1000) / 2;
  lastKnownError = position - center;

  return lastKnownError;  // return the error
}


void followLineWithControl(int baseSpeed) {
  int error = calculateLineError();

  // experimental values through testing
  float Kp = 0.4;
  float Kd = 0.2;
  float Ki = 0.15;

  static float integral = 0;
  integral += error;
  int derivative = error - lastError;
  D_filtered = D_alpha * D_filtered + (1.0 - D_alpha) * derivative;

  // pd implementation
  int correction = Kp * error + Kd * D_filtered;
  //int correction = Kp * error;
  lastError = error;

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, -400, 400);
  rightSpeed = constrain(rightSpeed, -400, 400);

  // set motor speeds
  mc1.setSpeed(1, leftSpeed);   // Front left
  mc1.setSpeed(2, rightSpeed);  // Front right
  mc1.setSpeed(3, leftSpeed);   // Rear left
  mc2.setSpeed(1, rightSpeed);  // Rear right
  mc2.setSpeed(2, leftSpeed);   // Extra left
  mc2.setSpeed(3, rightSpeed);  // Extra right
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
    values[i] = timeout;  // initialise to max timeout
  }

  while ((micros() - startTime) < timeout) {
    for (int i = 0; i < numSensors; i++) {
      if (digitalRead(pins[i]) == LOW && values[i] == timeout) {  // if it finished discharging
        values[i] = micros() - startTime;                         // measuring how long each pin stays high before going low
      }
    }
  }
}


// ultrasonic read
float readUltrasonicDistance() {
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

void checkMechKillSwitch() {
  bool currentReading = digitalRead(buttonPin);
  if (currentReading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > debounceDelay) {
    if (currentReading == LOW && lastButtonReading == HIGH) {
      robotRunning = !robotRunning;
      if (robotRunning) {
        Serial.println("Robot started");
      } else {
        Serial.println("Robot stopped");
        stopMotors();
      }
    }
  }
  lastButtonReading = currentReading;
  if (!robotRunning) {
    stopMotors();
    return;  // Skip all other logic in loop
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("start.");

  // for LED
  strip.begin();
  strip.setBrightness(80);
  strip.show();  // update the leds

  lastError = 0;  // initialise error for pd calculation

  myServo.attach(servoPin);  // for reflectance sensors

  myServo.write(180);  // arm up
  delay(1000);
  myServo.write(70);  // arm down
  delay(1000);


  // motors setup
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  //ultrasonic setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // mech button setup
  pinMode(buttonPin, INPUT_PULLUP);
  stopMotors();
  Serial.println("Waiting for button press...");

  // wifi
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
  checkMechKillSwitch();  // check for mech button
  bool currentReading = digitalRead(buttonPin);

  if (currentReading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (lastDebouncedState != currentReading) {
      lastDebouncedState = currentReading;
      if (currentReading == LOW) {  // button JUST pressed
        robotRunning = !robotRunning;

        if (robotRunning) {
          Serial.println("Robot started");
        } else {
          Serial.println("Robot stopped");
          stopMotors();
        }
      }
    }
  }

  lastButtonReading = currentReading; // update value

  if (!robotRunning) {
    return;  // robot is off, skip all logic
  }

  turnLEDOn();  // self-explanatory

  long distance = readUltrasonicDistance();
  if (distance > 0 && distance < 24) {
    Serial.println("wall detected! Stopping.");
    stopMotors();
    delay(100);
    turnRight(400);
    delay(2500);
    return;
  }

  readSensorArray(sensorPinsV, sensorValuesV);          // vertical
  readSensorArray(sensorPinsH, sensorValuesH);          // horizontal
  readSensorArray(sensorPinsRIGHT, sensorValuesRIGHT);  // right
  readSensorArray(sensorPinsLEFT, sensorValuesLEFT);    // left

  // SPECIAL CASES!!

  // END OF TRACK
  bool allSensorsBlack = true;

  Serial.print("horizontal: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValuesH[i]);
    if (sensorValuesV[i] < threshold) { // if vertical sensors are less than threshold
      allSensorsBlack = false;  
      break;  // get out of loop
    }
  }

  if (allSensorsBlack) {  // if all sensors are triggered
    for (int i = 0; i < numSensors; i++) {
      if (sensorValuesH[i] < threshold) { // if the horizontal sensors are above the threshold
        allSensorsBlack = false;  // set as false
        break;  // get out of loop 
      }
    }
  }

  if (allSensorsBlack) {  // if allSensorsBlack is still true
    Serial.println("END OF TRACK");
    stopMotors();
    delay(1500);
    myServo.write(180);
    return; // skip to start of loop again
  }

  // DEAD END
  bool allVerticalWhite = true;
  bool allHorizontalWhite = true;

  for (int i = 0; i < numSensors; i++) {
    if (sensorValuesV[i] > threshold) allVerticalWhite = false; // if vertical is on black line, set white is false
    if (sensorValuesH[i] > threshold) allHorizontalWhite = false; // if hori is on black line, set white is false
  }

  if (allVerticalWhite && allHorizontalWhite) { // if vertical and horizontal is all white -> dead end
    if (!deadEndActive) {
      deadEndStartTime = millis();
      deadEndActive = true;
    }

    if (millis() - deadEndStartTime >= 3000) {
      Serial.println("Dead end detected for 3 seconds! Stopping.");
      stopMotors();
      delay(300);
      moveBackward(300);
    }

    stopMotors();  // stop during the 3-second check too
    return;
  } else {
    deadEndActive = false;  // reset if no longer at dead end
  }

  // FORK
  bool leftDetected = false;
  bool rightDetected = false;
  // bool allVerticalWhite = false;
  int isWhite = 0;

  for (int i = 0; i < 2; i++) { // checking left and right refl. sensors
    if (sensorValuesLEFT[i] > threshold) {
      Serial.println("left BLACK");
      leftDetected = true;
    } else {
      leftDetected = false;
    }
    if (sensorValuesRIGHT[i] > threshold) {
      Serial.println("right BLACK");
      rightDetected = true;
    } else {
      rightDetected = false;  // make sure both are detected as black since one of the sensors is broken
    }
  }

  for (int i = 0; i < 9; i++) {
    if (sensorValuesH[i] > threshold) { // checks how many hori sensors are white
      isWhite++;
    }
  }

  if (isWhite > 5 && leftDetected && rightDetected) { // if there is more than 5 white in front/vert and black on left and right -> fork
    Serial.println("Fork detected!"); 
  }

  // SHARP 90˚ TURN
  bool allVerticalWhiteForTurn = true;
  bool middleHoriBlack = false;
  bool rightSideSensorBlack = false;
  bool leftSideSensorBlack = false;

  // check if right and left are black
  for (int i = 0; i < 2; i++) {
    if (sensorValuesRIGHT[i] > threshold) {
      rightSideSensorBlack = true;
      Serial.println(sensorValuesRIGHT[i]);
      Serial.println("right is black!!!");
    } else {
      rightSideSensorBlack = false;
    }
  }

  for (int i = 0; i < 2; i++) {
    if (sensorValuesLEFT[i] > threshold) {
      leftSideSensorBlack = true;
      Serial.println(sensorValuesLEFT[i]);
      Serial.println("left is black!!!");
    } else {
      leftSideSensorBlack = false;
    }

    for (int i = 6; i <= 9; i++) {  // if the middle sensors are black
      if (sensorValuesH[i] > threshold) {
        middleHoriBlack = true;
        break;
      }
    }

    int tot_ver_white = 0;

    for (int i = 0; i < 9; i++) { // checks if the vertical sensors are white
      if (sensorValuesV[i] < threshold) {
        tot_ver_white++;
      }
    }

    if (rightSideSensorBlack) { // if right sensor is on line
      Serial.println("right one is black");
    }

    Serial.println(tot_ver_white);
    Serial.println("vertical: ");

    for (int i = 0; i < 9; i++) {
      Serial.print(sensorValuesV[i]);
    }

    Serial.println(middleHoriBlack);
    Serial.println(rightSideSensorBlack);
    
    if (tot_ver_white >= 7 && middleHoriBlack && rightSideSensorBlack) {
      Serial.println("90 DEGREE RIGHT TURN");
      stopMotors();
      delay(150);
      // values chosen to match a 90˚ right turn in practice
      turnRight(400);
      delay(400); 
      stopMotors();
      delay(100);
    }

    if (tot_ver_white >= 7 && middleHoriBlack && leftSideSensorBlack) { // if the vert white is more than 7, middle hori is black, and left is all black -> left turn
      Serial.println("90 DEGREE LEFT TURN");
      stopMotors();
      delay(150);
      // values chosen to match a 90˚ left turn in practice
      turnLeft(400);
      delay(400); 
      stopMotors();
      delay(100);
    }

    //DOTTED LINE DETECTION
    int middleHorizontalWhite = 0;
    for (int i = 3; i <= 5; i++) {
      if (sensorValuesH[i] < threshold) { // check how manby hori sensors are white
        middleHorizontalWhite++;
      }
    }

    int tot_black = 0;

    for (int i = 0; i < 9; i++) {
      if (sensorValuesV[i] > threshold) { // check for how many vert sensors are black
        tot_black++;
      }
    }


    if (tot_black > 3 && middleHorizontalWhite) { // if more than 3 vert are black and middle is off line (white), it is a dotted line -> treat as a normal line
      Serial.println("Dotted line detected - treat as normal line");
      moveForward(600);
      delay(20);
      tot_black = 0;
    }

    // DEFAULT LINE FOLLOWING FOR DASHED/DOTTED LINE
    bool lineDetected = false;
    for (int i = 0; i < numSensors; i++) {
      if (sensorValuesV[i] < threshold) {  // black line
        lineDetected = true;
        break;
      }
    }

    if (lineDetected) {
      lastLineSeenTime = millis();  // reset timer
      followLineWithControl(300);  // have robot follow line w/ pd if no special exceptions triggered
    } else {
      if (millis() - lastLineSeenTime < lineLostTimeout) {
        Serial.println("Line lost briefly - continuing forward");
        followLineWithControl(300);  // assume line will return
      } else {
        Serial.println("Line lost for too long - stopping");
        stopMotors();
      }
    }

    Serial.println("moving");
    delay(50);
  }
}
