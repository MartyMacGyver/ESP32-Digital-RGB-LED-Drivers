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

#ifndef ESP32_DIGITAL_LED_FUNCS_H
#define ESP32_DIGITAL_LED_FUNCS_H

#ifdef __cplusplus
extern "C" {
#endif

void simpleStepper(strand_t * strands [], int numStrands, unsigned long delay_ms, unsigned long timeout_ms);
void randomStrands(strand_t * strands[], int numStrands, unsigned long delay_ms, unsigned long timeout_ms);
void scanners(strand_t * strands[], int numStrands, unsigned long delay_ms, unsigned long timeout_ms);
void scanner(strand_t * pStrand, unsigned long delay_ms, unsigned long timeout_ms);
void rainbows(strand_t * strands[], int numStrands, unsigned long delay_ms, unsigned long timeout_ms);
void rainbow(strand_t * pStrand, unsigned long delay_ms, unsigned long timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_DIGITAL_LED_FUNCS_H */
