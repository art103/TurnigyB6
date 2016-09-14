# ChARTurn: Custom Turnigy B6 Compact Firmware

## What?
This is a repalcement firmware for the [Turnigy B6 Compact charger](http://goo.gl/uX2E8J).
The name comes from Ch in Charger, my initials ART and urn from Turnigy.

It supports an (optional) I2C OLED display based on the ssd1306 chip like [this one](http://goo.gl/ApJXRa).

## Why?
I did not want 2C charging and I wanted an LCD output. 
I also like a challenge and don't trust anyone with my batteries :)

## Features
 - Automatic cell count detection and (continuous) checking
 - Automatic charge start
 - Capacity measurement and set charge current to 1C
 - Cell balancing during charge and after charge completion.

## Status
Calibration is currently manual and involves taking accurate readings with a meter and adjusting the parameters in code. 
I have a very accurate bench meter, but I have also used the CellMeter checker as it closely follows my bench meter and iCharger 206b (which I do trust).
The custom firmware is fully functional with good accuracy balancing once calibrated.

## Tool Chain
Code::Blocks: 13.12 
SDCC: 3.4.0

Code::Blocks must be set up to output .rel files:
Select Settings->Compiler
Select "Small Device C Compiler" from the dropdown.
Scroll to "Other Settings" and select "Advanced Options..."
Under "Others" in the "Object file extension" field, replace obj with rel.

## Other Information
I have reverse engineered the schematic. It is closely related to the iMAX B6 but uses FETs in place of BiPolar Transistors. 
It also has an STM8S CPU in place of the Atmel. 
The CPU part of the schematic is accurate, the rest is for indication only and is a remnant of my own design.

Programming pins are brought out and the STM8S-Discovery board works very well as a cheap programmer.

Be very careful with PWM! 
I developed whilst running from a bench power supply with a sensible current limit (1 to 10A depending on the scenario).
Switching to testing with a battery on the input yielded some strange results so I tried altering some of the PWM parameters to compensate. 
This led to a peak current of >20A which released the magic smoke from the input FET (AO4407).
A direct replacement part from RS Components is: IRF7424
