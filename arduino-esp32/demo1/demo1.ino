/* 
 * Copyright (c) 2017 Martin F. Falatic
 *
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

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define PIXELS 256
#define PIN 18

int currIdx = 0;
int prevIxd = 0;
int pausetime = 500;
uint8_t MAX_COLOR_VAL = 2;
const uint16_t NUM_PIXELS = 256;  // <--- modify to suit your configuration

rgbVal pixels[NUM_PIXELS];

void setup() {
  Serial.begin(115200);
  Serial.println("init");
  for (int i = 0; i < COUNT_OF(pixels); i++) {
    pixels[i] = makeRGBVal(0, 0, 0);
  }
  ws2812_init(PIN);
  //pixels = new rgbVal[NUM_PIXELS];
}

void loop() {
      //Serial.println("loop");
      rgbVal newColor = makeRGBVal(MAX_COLOR_VAL, MAX_COLOR_VAL, MAX_COLOR_VAL);
      pixels[prevIxd] = makeRGBVal(0, 0, 0);
      pixels[currIdx] = newColor;
      ws2812_setColors(COUNT_OF(pixels), pixels);
      prevIxd = currIdx;
      currIdx++;
      if (currIdx >= COUNT_OF(pixels)) {
        currIdx = 0;
      }
      //delay(pausetime);
}

