# ESP32 Digital RGB LED Drivers

A digital RGB LED (WS2812/SK6813/APA102/NeoPixel/DotStar) driver for the ESP32 

Based upon the [ESP32 WS2812 driver work by Chris Osborn](https://github.com/FozzTexx/ws2812-demo)

## Work in progress!

For the ESP-IDF SDK, esp-idf/demo1 works

For the Arduino-ESP32, arduino-esp32/demo1 does not work unless you modify the hardware code itself. FOR ADVANCED USERS ONLY!

In hardware/espressif/esp32/cores/esp32/main.cpp:

  - Instead of:

  `xTaskCreatePinnedToCore(loopTask, "loopTask", 4096, NULL, 1, NULL, 1);`

  - Use either this:

  `xTaskCreatePinnedToCore(loopTask, "loopTask", 4096, NULL, 1, NULL, 0);`
    
  - Or this:

  `xTaskCreate            (loopTask, "loopTask", 4096, NULL, 1, NULL   );`



