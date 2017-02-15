/* 
 * Demo code for digital RGB LEDs using the RMT peripheral on the ESP32
 *
 * Modifications Copyright (c) 2017 Martin F. Falatic
 *
 * Based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * The RMT peripheral on the ESP32 provides very accurate timing of
 * signals sent to the WS2812 LEDs.
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

#include "ws2812.h"

#if defined(ARDUINO) && ARDUINO >= 100
  // No extras
#elif defined(ARDUINO) // pre-1.0
  // No extras
#elif defined(ESP_PLATFORM)
  #include "arduinoish.hpp"
#endif

const int DATA_PIN = 18;
const uint16_t NUM_PIXELS = 256;  // <--- modify to suit your configuration
uint8_t MAX_COLOR_VAL = 32;

int pausetime = 500;

rgbVal *pixels;

void displayOff();
void rainbow(unsigned long, unsigned long);
void scanner(unsigned long, unsigned long);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  ws2812_init(DATA_PIN);
  pixels = (rgbVal*)malloc(sizeof(rgbVal) * NUM_PIXELS);
  displayOff();
  Serial.println("Init complete");
  delay(1000);
}

void loop() {
  rainbow(0, 5000);
  scanner(0, 5000);
  displayOff();
}

void displayOff() {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels[i] = makeRGBVal(0, 0, 0);
  }
  ws2812_setColors(NUM_PIXELS, pixels);
}

void scanner(unsigned long delay_ms, unsigned long timeout_ms) {
  int currIdx = 0;
  int prevIxd = 0;
  bool RUN_FOREVER = (timeout_ms == 0 ? true : false);
  unsigned long start_ms = millis();
  while (RUN_FOREVER || (millis() - start_ms < timeout_ms)) {
    pixels[prevIxd] = makeRGBVal(0, 0, 0);
    pixels[currIdx] = makeRGBVal(MAX_COLOR_VAL, MAX_COLOR_VAL, MAX_COLOR_VAL);;
    ws2812_setColors(NUM_PIXELS, pixels);
    prevIxd = currIdx;
    currIdx++;
    if (currIdx >= NUM_PIXELS) {
      currIdx = 0;
    }
    delay(delay_ms);
  }
}

void rainbow(unsigned long delay_ms, unsigned long timeout_ms)
{
  const uint8_t color_div = 4;
  const uint8_t anim_step = 1;
  const uint8_t anim_max = MAX_COLOR_VAL - anim_step;
  rgbVal color = makeRGBVal(anim_max, 0, 0);
  rgbVal color2 = makeRGBVal(anim_max, 0, 0);
  uint8_t stepVal = 0;
  uint8_t stepVal2 = 0;

  bool RUN_FOREVER = (timeout_ms == 0 ? true : false);
  unsigned long start_ms = millis();
  while (RUN_FOREVER || (millis() - start_ms < timeout_ms)) {
    color = color2;
    stepVal = stepVal2;
  
    for (uint16_t i = 0; i < NUM_PIXELS; i++) {
      pixels[i] = makeRGBVal(color.r/color_div, color.g/color_div, color.b/color_div);
  
      if (i == 1) {
        color2 = color;
        stepVal2 = stepVal;
      }
  
      switch (stepVal) {
        case 0:
        color.g += anim_step;
        if (color.g >= anim_max)
          stepVal++;
        break;
        case 1:
        color.r -= anim_step;
        if (color.r == 0)
          stepVal++;
        break;
        case 2:
        color.b += anim_step;
        if (color.b >= anim_max)
          stepVal++;
        break;
        case 3:
        color.g -= anim_step;
        if (color.g == 0)
          stepVal++;
        break;
        case 4:
        color.r += anim_step;
        if (color.r >= anim_max)
          stepVal++;
        break;
        case 5:
        color.b -= anim_step;
        if (color.b == 0)
          stepVal = 0;
        break;
      }
    }
  
    ws2812_setColors(NUM_PIXELS, pixels);
  
    delay(delay_ms);
  }
}

