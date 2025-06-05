#include <Motoron.h>

// make a motoron object for each motor shield w/ the default adresses
MotoronI2C mc1(0x14); // lower motor shield: M1 - left front; M2 - right front; M3 - middle left
MotoronI2C mc2(0x10); // upper motor shield. M1 - middle right; M2 - left back; M3 - right back

void setup() {
  Wire.begin();  // initialise I2C
  Serial.begin(9600);

  // initialize motoron controller
  mc1.reinitialize();    // reset the controller
  mc1.disableCrc();      // disable CRC check
  mc1.clearResetFlag();  // clear reset flag

  mc2.reinitialize();    // reset the controller
  mc2.disableCrc();      // Disable CRC check
  mc2.clearResetFlag();  // Clear reset flag

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

void loop() {

  // test forward
  moveForward(700);
  delay(1000);

  stopMotors();
  delay(2000);

  // test backward
  moveBackward(700);
  delay(1000);

  stopMotors();
  delay(2000); 

  // test turning left
  turnLeft(700);
  delay(1000);

  stopMotors();
  delay(2000);

  // test turning right
  turnRight(700);
  delay(1000);

  stopMotors();
  delay(2000);
}

