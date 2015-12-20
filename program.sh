#!/bin/bash

# You may need to run this as root if reset.py needs to detach a kernel driver.

make;
python reset.py;
sleep 2;
avrdude -v -p atmega32u4 -c avr109 -P /dev/ttyACM0 -b 57600 -D -U flash:w:Joystick.hex:i;
