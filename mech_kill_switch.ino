#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc(0x10);

const int buttonPin = 2;
bool robotRunning = false;
bool lastButtonReading = HIGH; // Button reading value
bool lastDebouncedState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.disableCommandTimeout();

  pinMode(buttonPin, INPUT_PULLUP);
  stopRobot();
  Serial.println("System initialized. Waiting for button press...");
}

void loop() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // The button state has truly changed
    if (reading != lastDebouncedState) {
      lastDebouncedState = reading;

      if (reading == LOW) {  // Button pressed
        robotRunning = !robotRunning;
        if (robotRunning) {
          Serial.println("Robot started");
          startRobot();
        } else {
          Serial.println("Robot stopped");
          stopRobot();
        }
      }
    }
  }

  lastButtonReading = reading;
}

void startRobot() {
  mc.setSpeed(1, 400);
  mc.setSpeed(2, 400);
  mc.setSpeed(3, 400);
}

void stopRobot() {
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  mc.setSpeed(3, 0);
}