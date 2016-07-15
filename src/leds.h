/*
    ChARTurn: Custom firmware for Turnigy B6 Compact charger.
    Copyright (C) 2016 Richard Taylor <richard@artaylor.co.uk>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/  

#ifndef _LEDS_H_
#define _LEDS_H_

/*
 * Initialise the LED GPIOs
 */
void leds_init(void);

/*
 * red: bitmask of LEDs to set red
 * green:  bitmask of LEDs to set green
 * mask: bitmask of LEDs to set values for
 */
void leds_set(uint8_t red, uint8_t green, uint8_t mask);

#endif // _LEDS_H_
