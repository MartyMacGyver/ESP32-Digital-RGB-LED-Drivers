#ifndef _COMPONENT_WS2812_H_
#define _COMPONENT_WS2812_H_

#include <stdint.h>

typedef union {
  struct __attribute__ ((packed)) {
    uint8_t r, g, b;
  };
  uint32_t num;
} rgbVal;

#define DEBUG_WS2812_DRIVER 0

#if DEBUG_WS2812_DRIVER
char *    ws2812_debugBuffer;
const int ws2812_debugBufferSz = 1024;
#endif

enum led_types {LED_WS2812, LED_WS2812B, LED_SK6812, LED_WS2813};
extern int  ws2812_init(int gpioNum, int ledType);
extern void ws2812_setColors(uint16_t length, rgbVal *array);

inline rgbVal makeRGBVal(uint8_t r, uint8_t g, uint8_t b)
{
  rgbVal v;
  v.r = r;
  v.g = g;
  v.b = b;
  return v;
}

#endif /* _COMPONENT_WS2812_H_ */
