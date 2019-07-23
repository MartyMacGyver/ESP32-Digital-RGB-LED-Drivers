/* 
 * Demo code for driving a single digital RGB(W) strand using esp32_digital_led_lib
 *
 * Modifications Copyright (c) 2017-2019 Martin F. Falatic
 *
 * Based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
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

#include "esp32_digital_led_lib.h"
#include "esp32_digital_led_funcs.h"
#include "fireworks_effects.h"


#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"  // It's noisy here with `-Wall`

//strand_t strand = {.rmtChannel = 0, .gpioNum = 26, .ledType = LED_WS2812B_V3, .brightLimit = 32, .numPixels = 64};
strand_t strand = {.rmtChannel = 0, .gpioNum = 27, .ledType = LED_SK6812W_V1, .brightLimit = 64, .numPixels = 144};

strand_t * STRANDS [] = { &strand };

int STRANDCNT = COUNT_OF(STRANDS); 

#pragma GCC diagnostic pop


FireworksEffects * fweffects;

//**************************************************************************//
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  digitalLeds_initDriver();

  // Init unused outputs low to reduce noise
  gpioSetup(14, OUTPUT, LOW);
  gpioSetup(15, OUTPUT, LOW);
  gpioSetup(26, OUTPUT, LOW);
  gpioSetup(27, OUTPUT, LOW);

  gpioSetup(strand.gpioNum, OUTPUT, LOW);
  int rc = digitalLeds_addStrands(STRANDS, STRANDCNT);
  if (rc) {
    Serial.print("Init rc = ");
    Serial.println(rc);
  }

  if (digitalLeds_initDriver()) {
    Serial.println("Init FAILURE: halting");
    while (true) {};
  }
  digitalLeds_resetPixels(STRANDS, STRANDCNT);

  fweffects = new FireworksEffects(&strand);
}


//**************************************************************************//
void loop()
{
  // simpleStepper(STRANDS, STRANDCNT, 0, 0);
  // randomStrands(STRANDS, STRANDCNT, 200, 10000);
  // rainbows(STRANDS, STRANDCNT, 1, 0);
  Serial.println("Rendering");
  fweffects->Render();
}
