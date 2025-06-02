#include <Adafruit_DotStar.h> // library

#define NUMPIXELS 5 // no. of LEDs in strip
#define DATAPIN 4
#define CLOCKPIN 5
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

void setup() {

  strip.begin(); // initialise pins for output
  strip.setBrightness(80);  // brightness
  strip.show();  // turn all LEDs off for initialisation

}

void loop() {
  if (1) { // to set condition when arm is down
    for (int i=0; i < NUMPIXELS; i++) {
      strip.setPixelColor(i,255,255,255);
    }
  }
}
