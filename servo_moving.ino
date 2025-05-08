#include <Servo.h>

Servo myServo;

void setup() {
  Serial.begin(9600); 
  myServo.attach(9); 
}

void loop() {
  for (int angle = 0; angle <= 180; angle++) {
    myServo.write(angle);     
    delay(15);               

    uint16_t val = analogRead(A0);
    double dat = (double) val * 0.47 - 33.4;
    Serial.print("Position: ");
    Serial.print(dat);
    Serial.println(" Degree");
  }

  for (int angle = 180; angle >= 0; angle--) {
    myServo.write(angle);
    delay(15);

    uint16_t val = analogRead(A0);
    double dat = (double) val * 0.47 - 33.4;
    Serial.print("Position: ");
    Serial.print(dat);
    Serial.println(" Degree");
  }
}
