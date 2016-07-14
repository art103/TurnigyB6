# Custom Turnigy B6 Compact Firmware

## What?
This is a repalcement firmware for the Turnigy B6 Compact charger:
http://goo.gl/uX2E8J

It supports an I2C OLED display based on the ssd1306 chip like this one: 
http://goo.gl/ApJXRa

## Why?
I did not want 2C charging and I wanted an LCD output. 
I also like a challenge :)

## Features
 - Automatic cell count detection and (continuous) checking
 - Automatic charge start
 - Capacity measurement and set charge current to 1C
 - Cell balancing during charge and after charge completion.

## Status
The custom firmware is fully functional with good accuracy balancing. 
The capacity calculation is still a little inaccurate though, so not ready for daily driver use yet!

## Other Information
I have reverse engineered the schematic. It is closely related to the iMAX B6 but uses FETs in place of BiPolar Transistors. 
It also has an STM8S CPU in place of the Atmel. 
The CPU part of the schematic is accurate, the rest is for indication only and is a remnant of my own design.

Programming pins are brought out and the STM8S-Discovery board works very well as a cheap programmer.
