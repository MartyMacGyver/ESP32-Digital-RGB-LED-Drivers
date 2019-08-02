/* 
 * Supplemental function for the ESP32 Digital LED Library
 *
 * Copyright (c) 2019 Martin F. Falatic
 *
 * Rainbow animation is based on public domain code created 19 Nov 2016
 * by Chris Osborn <fozztexx@fozztexx.com> http://insentricity.com
 *
 */

/* 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#elif defined(ARDUINO) // pre-1.0
  // No extras
#elif defined(ESP_PLATFORM)
  #include "../../main/arduinoish.hpp"
#endif

#include "esp32_digital_led_lib.h"
#include "esp32_digital_led_funcs.h"


#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

//**************************************************************************//
void simpleStepper(strand_t * strands [], int numStrands, unsigned long delay_ms, unsigned long timeout_ms)
{
  int stepper = 0;
  int colord = 0;
  int highLimit = 32;
  unsigned long start_ms = millis();
  while (timeout_ms == 0 || (millis() - start_ms < timeout_ms)) {
    strand_t * strand = strands[0];
    strand->pixels[stepper] = pixelFromRGBW(colord, colord, colord, 0);

    stepper++;
    if(stepper > strand->numPixels) {
      stepper = 0;
      colord += 2;
    }
  
    if(colord > highLimit) 
      colord = 0;
    
    digitalLeds_drawPixels(strands, numStrands);
    delay(delay_ms);
  }
  digitalLeds_resetPixels(strands, numStrands);
}


//**************************************************************************//
void randomStrands(strand_t * strands[], int numStrands, unsigned long delay_ms, unsigned long timeout_ms)
{
  Serial.print("DEMO: random colors, delay = ");
  Serial.println(delay_ms);
  uint32_t dimmer = 0x0F0F0F0F;
  unsigned long start_ms = millis();
  while (timeout_ms == 0 || (millis() - start_ms < timeout_ms)) {
    for (int n = 0; n < numStrands; n++) {
      strand_t * pStrand = strands[n];
      for (uint16_t i = 0; i < pStrand->numPixels; i++) {
        pStrand->pixels[i].raw32 = (esp_random() & dimmer);
      }
    }
    digitalLeds_drawPixels(strands, numStrands);
    delay(delay_ms);
  }
}


//**************************************************************************//
class Scannerer {
  private:
    strand_t * pStrand;
    pixelColor_t minColor;
    pixelColor_t maxColor;
    int prevIdx;
    int currIdx;
  public:
    Scannerer(strand_t *, pixelColor_t);
    void prepareNext();
};


Scannerer::Scannerer(strand_t * pStrandIn, pixelColor_t maxColorIn)
{
  pStrand = pStrandIn;
  minColor = pixelFromRGBW(0, 0, 0, 0);
  maxColor = maxColorIn;
  prevIdx = 0;
  currIdx = 0;
}


void Scannerer::prepareNext()
{
  pStrand->pixels[prevIdx] = minColor;
  pStrand->pixels[currIdx] = maxColor;
  prevIdx = currIdx;
  currIdx++;
  if (currIdx >= pStrand->numPixels) {
    currIdx = 0;
  }
}


void scanners(strand_t * strands[], int numStrands, unsigned long delay_ms, unsigned long timeout_ms)
{
  //Scannerer scan(pStrand); Scannerer * pScanner = &scan;
  Scannerer * pScanner[numStrands];
  int i;
  uint8_t c = strands[0]->brightLimit; // TODO: improve
  pixelColor_t scanColors [] = {
    pixelFromRGBW(c, 0, 0, 0),
    pixelFromRGBW(0, c, 0, 0),
    pixelFromRGBW(c, c, 0, 0),
    pixelFromRGBW(0, 0, c, 0),
    pixelFromRGBW(c, 0, c, 0),
    pixelFromRGBW(0, c, c, 0),
    pixelFromRGBW(c, c, c, 0),
    pixelFromRGBW(0, 0, 0, c),
  };
  Serial.print("DEMO: scanners(");
  for (i = 0; i < numStrands; i++) {
    pScanner[i] = new Scannerer(strands[i], scanColors[i]);
    if (i > 0) {
      Serial.print(", ");
    }
    Serial.print("ch");
    Serial.print(strands[i]->rmtChannel);
    Serial.print(" (0x");
    Serial.print((uint32_t)pScanner[i], HEX);
    Serial.print(")");
    Serial.print(" #");
    Serial.print((uint32_t)scanColors[i].raw32, HEX);
  }
  Serial.print(")");
  Serial.println();
  unsigned long start_ms = millis();
  while (timeout_ms == 0 || (millis() - start_ms < timeout_ms)) {
    for (i = 0; i < numStrands; i++) {
      pScanner[i]->prepareNext();
    }
    digitalLeds_drawPixels(strands, numStrands);
    delay(delay_ms);
  }
  for (i = 0; i < numStrands; i++) {
    delete pScanner[i];
  }
  digitalLeds_resetPixels(strands, numStrands);
}


void scanner(strand_t * pStrand, unsigned long delay_ms, unsigned long timeout_ms)
{
  strand_t * strands [] = { pStrand };
  scanners(strands, 1, delay_ms, timeout_ms);
}


//**************************************************************************//
class Rainbower {
  private:
    strand_t * pStrand;
    const uint8_t color_div = 4;
    const uint8_t anim_step = 1;
    uint8_t anim_max;
    uint8_t stepVal1;
    uint8_t stepVal2;
    pixelColor_t color1;
    pixelColor_t color2;
  public:
    Rainbower(strand_t *);
    void prepareNext();
};


Rainbower::Rainbower(strand_t * pStrandIn)
{
  pStrand = pStrandIn;
  anim_max = pStrand->brightLimit - anim_step;
  stepVal1 = 0;
  stepVal2 = 0;
  color1 = pixelFromRGBW(anim_max, 0, 0, 0);
  color2 = pixelFromRGBW(anim_max, 0, 0, 0);
}


void Rainbower::prepareNext()
{
  color1 = color2;
  stepVal1 = stepVal2;
  for (uint16_t i = 0; i < pStrand->numPixels; i++) {
    pStrand->pixels[i] = pixelFromRGBW(color1.r/color_div, color1.g/color_div, color1.b/color_div, 0);
    if (i == 1) {
      color2 = color1;
      stepVal2 = stepVal1;
    }
    switch (stepVal1) {
      case 0:
      color1.g += anim_step;
      if (color1.g >= anim_max)
        stepVal1++;
      break;
      case 1:
      color1.r -= anim_step;
      if (color1.r == 0)
        stepVal1++;
      break;
      case 2:
      color1.b += anim_step;
      if (color1.b >= anim_max)
        stepVal1++;
      break;
      case 3:
      color1.g -= anim_step;
      if (color1.g == 0)
        stepVal1++;
      break;
      case 4:
      color1.r += anim_step;
      if (color1.r >= anim_max)
        stepVal1++;
      break;
      case 5:
      color1.b -= anim_step;
      if (color1.b == 0)
        stepVal1 = 0;
      break;
    }
  }
}


void rainbows(strand_t * strands[], int numStrands, unsigned long delay_ms, unsigned long timeout_ms)
{
  //Rainbower rbow(pStrand); Rainbower * pRbow = &rbow;
  Rainbower * pRbow[numStrands];
  int i;
  Serial.print("DEMO: rainbows(");
  for (i = 0; i < numStrands; i++) {
    pRbow[i] = new Rainbower(strands[i]);
    if (i > 0) {
      Serial.print(", ");
    }
    Serial.print("ch");
    Serial.print(strands[i]->rmtChannel);
    Serial.print(" (0x");
    Serial.print((uint32_t)pRbow[i], HEX);
    Serial.print(")");
  }
  Serial.print(")");
  Serial.println();
  unsigned long start_ms = millis();
  while (timeout_ms == 0 || (millis() - start_ms < timeout_ms)) {
    for (i = 0; i < numStrands; i++) {
      pRbow[i]->prepareNext();
    }
    digitalLeds_drawPixels(strands, numStrands);
    delay(delay_ms);
  }
  for (i = 0; i < numStrands; i++) {
    delete pRbow[i];
  }
  digitalLeds_resetPixels(strands, numStrands);
}


void rainbow(strand_t * pStrand, unsigned long delay_ms, unsigned long timeout_ms)
{
  strand_t * strands [] = { pStrand };
  rainbows(strands, 1, delay_ms, timeout_ms);
}


//**************************************************************************//
