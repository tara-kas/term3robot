#include <Motoron.h>

// Create a Motoron object with address 0x10 (default)
MotoronI2C mc(0x10);

void setup() {
  Wire.begin();  // Initialize I2C
  Serial.begin(9600);

  // Initialize the Motoron controller
  mc.reinitialize();    // Reset the controller
  mc.disableCrc();      // Disable CRC check
  mc.clearResetFlag();  // Clear reset flag

  // If you want to disable the 1500 ms timeout protection, uncomment the line below
  // mc.disableCommandTimeout();
}

// === Control Functions ===

// Move forward: all motors rotate forward
void moveForward(int speed) {
  mc.setSpeed(1, speed);  // Front left wheel
  mc.setSpeed(2, speed);  // Front right wheel
  mc.setSpeed(3, speed);  // Rear wheel
}

// Move backward: all motors rotate in reverse
void moveBackward(int speed) {
  mc.setSpeed(1, -speed);
  mc.setSpeed(2, -speed);
  mc.setSpeed(3, -speed);
}

// Turn left: front wheels rotate in opposite directions, rear wheel stops
void turnLeft(int speed) {
  mc.setSpeed(1, -speed);  // Front left wheel reverse
  mc.setSpeed(2, speed);   // Front right wheel forward
  mc.setSpeed(3, 200);     // Rear wheel (should be stop, but 200 is not 0)
}

// Turn right: front wheels rotate in opposite directions, rear wheel stops
void turnRight(int speed) {
  mc.setSpeed(1, speed);   // Front left wheel forward
  mc.setSpeed(2, -speed);  // Front right wheel reverse
  mc.setSpeed(3, 200);   // Rear wheel (again, not stopped if speed != 0)
}

// Stop all wheels
void stopMotors() {
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  mc.setSpeed(3, 0);
}

// === Main Loop ===
void loop() {
  moveForward(400);
}

