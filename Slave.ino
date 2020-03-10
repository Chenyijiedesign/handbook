#include <SoftwareSerial.h>
#include "Adafruit_NeoPixel.h"

// Slave
#define PIN 6
Adafruit_NeoPixel strip = Adafruit_NeoPixel(64, PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial BlueTooth(8,9);
int test;
char string;

void setup() {
  BlueTooth.begin(9600);
  Serial.begin(9600);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  int p;
  if(BlueTooth.available()>0){
  test = BlueTooth.read();
  Serial.println(test);
  light(test);
}
}
      
void light(int test){
  if(test == 0)
    {
        colorWipe(strip.Color(0, 0, 100), 1); // Red
    }
  else if(test == 1)
    {
        colorWipe(strip.Color(100, 0, 0), 1);// Green
        
    }
  else if(test == 2)
    {
        colorWipe(strip.Color(0, 100, 0), 1); // Blue        
    }
  else if(test == 3)
    {
        rainbowCycle(1); // Rainbow       
    }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
