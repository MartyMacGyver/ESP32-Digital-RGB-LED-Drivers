#!/usr/bin/bash

cp ../../arduino-esp32/demo1/demo1.ino main/main.cpp
cp ../../../src/*.cpp components/esp32_digital_led_lib
mkdir components/esp32_digital_led_lib/include
cp ../../../src/*.h components/esp32_digital_led_lib/include

make menuconfig
# make sdkconfig  # no menu

make clean

make -j 6  ;: # often fails the first time on some "file already exists" issue

# make flash
# make erase_flash flash



