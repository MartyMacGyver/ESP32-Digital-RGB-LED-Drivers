/* Created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Demo of driving WS2812 RGB LEDs using the RMT peripheral.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include "ws2812.h"

#include <esp_system.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/rmt_struct.h>

#define WS2812_PIN	18

#define delay_ms(ms) vTaskDelay((ms) / portTICK_RATE_MS)

void rainbow(void *pvParameters)
{
  const uint8_t dim_div = 32;
  const uint8_t anim_step = 8;
  const uint8_t anim_max = 256 - anim_step;
  const uint16_t pixel_count = 256; // Number of your "pixels"
  const uint8_t delay = 0; // duration between color changes
  rgbVal color = makeRGBVal(anim_max, 0, 0);
  uint8_t step = 0;
  rgbVal color2 = makeRGBVal(anim_max, 0, 0);
  uint8_t step2 = 0;
  rgbVal *pixels;


  pixels = malloc(sizeof(rgbVal) * pixel_count);

  while (1) {
    color = color2;
    step = step2;

    for (uint16_t i = 0; i < pixel_count; i++) {
      pixels[i] = makeRGBVal(color.r/dim_div, color.g/dim_div, color.b/dim_div);

      if (i == 1) {
        color2 = color;
        step2 = step;
      }

      switch (step) {
      case 0:
        color.g += anim_step;
        if (color.g >= anim_max)
          step++;
        break;
      case 1:
        color.r -= anim_step;
        if (color.r == 0)
          step++;
        break;
      case 2:
        color.b += anim_step;
        if (color.b >= anim_max)
          step++;
        break;
      case 3:
        color.g -= anim_step;
        if (color.g == 0)
          step++;
        break;
      case 4:
        color.r += anim_step;
        if (color.r >= anim_max)
          step++;
        break;
      case 5:
        color.b -= anim_step;
        if (color.b == 0)
          step = 0;
        break;
      }
    }

    ws2812_setColors(pixel_count, pixels);

    delay_ms(delay);
  }
}

void app_main()
{
  nvs_flash_init();

  ws2812_init(WS2812_PIN);
  xTaskCreate(rainbow, "ws2812 rainbow demo", 4096, NULL, 10, NULL);

  return;
}
