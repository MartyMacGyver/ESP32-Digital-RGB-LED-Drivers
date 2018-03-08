#include "esp32_digital_led_lib.h"

/*
 * Simplest possible example shows a single strand of NeoPixels. See Demo1 for multiple strands and other devices.
 */

strand_t pStrand = {.rmtChannel = 0, .gpioNum = 16, .ledType = LED_WS2812B_V3, .brightLimit = 32, .numPixels = 240,
   .pixels = nullptr, ._stateVars = nullptr};

int stepper = 0;
int colord = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  pinMode (16, OUTPUT);
  digitalWrite (16, LOW);

  if (digitalLeds_initStrands(&pStrand, 1)) {
    Serial.println("Init FAILURE: halting");
    while (true) {};
  }
  
  digitalLeds_resetPixels(&pStrand);
}

void loop()
{
    pStrand.pixels[stepper] = pixelFromRGBW(55, colord, 0, 0);

    stepper++;
    
    if(stepper > 240) {
      stepper = 0;
      colord += 10;
    }

    if(colord > 60) 
      colord = 0;
    
    digitalLeds_updatePixels(&pStrand);
}
