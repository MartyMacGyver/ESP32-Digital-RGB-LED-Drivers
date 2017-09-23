#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "ws2812.h"

#include "sdkconfig.h"

static char tag[] = "neosarchizo";

const int DATA_PIN = 17;
const uint16_t NUM_PIXELS = 3;
uint8_t MAX_COLOR_VAL = 32;

rgbVal *pixels;

void app_main() {
	ESP_LOGD(tag, "start");
	ws2812_init(DATA_PIN, LED_SK6812);
	pixels = (rgbVal*)malloc(sizeof(rgbVal) * NUM_PIXELS);

	while(1) {
		for(uint16_t i=0; i<NUM_PIXELS; i++) {
			pixels[i] = makeRGBVal(MAX_COLOR_VAL, 0, 0);
		}
		ws2812_setColors(NUM_PIXELS, pixels);
		vTaskDelay(1000/portTICK_PERIOD_MS);
		for (uint16_t i=0; i<NUM_PIXELS; i++) {
			pixels[i] = makeRGBVal(0, MAX_COLOR_VAL, 0);
		}
		ws2812_setColors(NUM_PIXELS, pixels);
		vTaskDelay(1000/portTICK_PERIOD_MS);
		for (uint16_t i=0; i<NUM_PIXELS; i++) {
			pixels[i] = makeRGBVal(0, 0, MAX_COLOR_VAL);
		}
		ws2812_setColors(NUM_PIXELS, pixels);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
} // task_hmc5883l
