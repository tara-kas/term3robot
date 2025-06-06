#include <Servo.h>
#include <Wire.h>
#include <Motoron.h>

// all left side motors -- red wire with B black wire with A
// all right side motors -- red wire with A black wire with B

Servo dssServo;
MotoronI2C mc1(0x14);  // lower motor shied. M1 - Left Front. M2 - Right Front. M3 - Middle Left.
MotoronI2C mc2(0x10);  // upper motor shield. M1 - Middle Right. M2 - Left Back. M3 - Right Back.

// mech kill switch initialisation
const int buttonPin = 2;
bool robotRunning = false;
bool lastButtonReading = HIGH;  // Button reading value
bool lastDebouncedState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void stopMotors() {
  mc1.setSpeed(1, 0);
  mc1.setSpeed(2, 0);
  mc1.setSpeed(3, 0);
  mc2.setSpeed(1, 0);
  mc2.setSpeed(2, 0);
  mc2.setSpeed(3, 0);
}

void setup() {
  Wire.begin();

  // motors setup
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  dssServo.attach(51);  // servo initialism 
}

void moveForward(int speed) {
  mc1.setSpeed(1, speed);
  mc1.setSpeed(2, speed);
  mc1.setSpeed(3, speed);
  mc2.setSpeed(1, speed);
  mc2.setSpeed(2, speed);
  mc2.setSpeed(3, speed);
}

void loop() {
  dssServo.write(90); // hook close
  delay(1000);
  moveForward(700); // run across the lava pit w/ hook on
  delay(10000);
  dssServo.write(150); // hook open
  delay(1000);
}
