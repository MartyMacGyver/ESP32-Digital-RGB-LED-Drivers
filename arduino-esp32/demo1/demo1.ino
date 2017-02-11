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

extern "C"
{
#include "ws2812.h"
}

#define PIXELS 256
#define PIN 18

void setup() {
  Serial.begin(115200);
  Serial.println("init");
  delay(2000);
  ws2812_init(PIN);
}

int pausetime = 40;
uint8_t MAX_COLOR_VAL = 2;

rgbVal pixels [3];

void loop() {
      Serial.println("loop");
      rgbVal color = makeRGBVal(16, 0, 0);
      pixels[0] = color;
      pixels[1] = color;
      pixels[2] = color;
      ws2812_setColors(3, pixels);
      delay(pausetime);
}


