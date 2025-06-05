#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Motoron.h>
#include <Servo.h>

// for wifi kill switch
const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";
WiFiUDP udp;
unsigned int localUdpPort = 2390;
char incomingPacket[255];
bool killSwitch = false;

// mech kill switch initialisation
const int buttonPin = 2;
bool robotRunning = false;
bool lastButtonReading = HIGH; // Button reading value
bool lastDebouncedState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// motoron object
MotoronI2C mc1(0x14);
MotoronI2C mc2(0x10);

// sensor values
float distanceCM = 0.0;
float distanceCMfront = 0.0;
float distanceCMback = 0.0;
const int threshold = 400;
const int numSensors = 9;
int lastKnownError = 0;
float previousError = 0;
float integral = 0;

// sensor pins
const int trigPin = 13;
const int echoPin = 13;
const int leftIRFront = A1;
const int leftIRBack = A2;
const int rightIR = A3;
Servo armServo; // Add:
Servo dssServo; // hook
bool lastButtonState = HIGH; // Should match the logic


// servo pin
const int servoPin = 43;

// motor codes
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

// keeping in the middle
void adjustParallelToWallPID() {
  // experimental pid values - no integral term since we found it made the movement worse
  float Kp = 25.0;
  float Ki = 0.0;
  float Kd = 10.0;

  static float previousError = 0;
  static float integral = 0;

  // read the two left sensors
  float frontDist = readIRDistance(leftIRFront);
  float backDist = readIRDistance(leftIRBack);

  // error is the difference between the front and back sensors
  float error = frontDist - backDist;

  integral += error;
  float derivative = error - previousError;
  previousError = error;

  // pd implementation
  float correction = Kp * error + Kd * derivative;

  int baseSpeed = 700; // high to overcome other obstacles in section 2
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  // leftSpeed = constrain(baseSpeed - correction, -100, 100);
  // rightSpeed = constrain(baseSpeed + correction, -100, 100);

  mc1.setSpeed(1, leftSpeed); // front left
  mc1.setSpeed(2, rightSpeed); // front right
  mc1.setSpeed(3, leftSpeed); // middle left
  mc2.setSpeed(1, rightSpeed); // middle right
  mc2.setSpeed(2, leftSpeed); // back left
  mc2.setSpeed(3, rightSpeed); // back right
}

// wall detection
void checkWallsAndAct() {
  // read distance ad ultrasonic sensor values
  distanceCM = readUltrasonicDistance();
  float leftFront = readIRDistance(leftIRFront);
  float leftBack = readIRDistance(leftIRBack);
  float right = readIRDistance(rightIR);

  // if wall is closer than set threshold values - constants chosen by trial and error
  bool wallFront = distanceCM < 23;
  bool wallLeft = leftFront < 5 || leftBack < 5;
  bool wallBothSides = right < 5 && leftFront < 5;

  Serial.print("Front: "); Serial.print(distanceCM);
  Serial.print(" | Left Front: "); Serial.print(leftFront);
  Serial.print(" | Left Back: "); Serial.println(leftBack);
  Serial.print(" | Right: "); Serial.println(right);

  if (wallFront) {// if a wall is in front, if so, turn right (since there are only right turns)
    stopMotors(); // stop so robot can turn
    delay(200);
    Serial.println("wall in da front");
    turnRight(600); // turn right for..
    delay(4000);  // ..4 seconds for a sharp right turn
  } else if (wallLeft) {
    adjustParallelToWallPID();  // adjust wall if no wall in front
  } else {
    moveForward(700); // go forwards otherwise
  }
}



void checkKillSwitch() {  // wifi kill switch
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

void checkMechKillSwitch() {  // mech kill switch
  bool buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    Serial.println("Pressed");
  } else {
    Serial.println("Not pressed");
  }
}


void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("start");

  // set reflectance sensor arm up so it's not in the way
  armServo.attach(servoPin);
  armServo.write(180);

  dssServo.attach(51);  // hook servo

  // motors setup
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  // ultrasonic setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);
  stopMotors(); // ensure motors are stopped initially, so it doesn't start until button pressed

  // mech button setup
  pinMode(buttonPin, INPUT_PULLUP);
  stopMotors();
  Serial.println("Waiting for button press...");

  // wifi chekc
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
  bool currentReading = digitalRead(buttonPin);

  // check if button is pressed every time the loop runs
  if (currentReading != lastButtonReading) {
  lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (lastDebouncedState != currentReading) {
      lastDebouncedState = currentReading;

      if (currentReading == LOW) { // button just pressed
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

  lastButtonReading = currentReading;

  if (!robotRunning) {
  return; // robot is off, skip all logic
  }

  checkWallsAndAct(); // check walls for tuning and turning logic
  delay(100);

}

