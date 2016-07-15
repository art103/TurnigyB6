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

#include "stm8s.h"
#include "charger.h"
#include "gpio.h"

/*
 * Mapping of led numbers to GPIO pins.
 */
static const struct _pin leds[NUM_CHANNELS] =
{ {GPIOB, GPIO_PIN_7},
  {GPIOB, GPIO_PIN_6},
  {GPIOB, GPIO_PIN_3},
  {GPIOE, GPIO_PIN_5},
  {GPIOC, GPIO_PIN_2},
  {GPIOC, GPIO_PIN_3}
};

/*
 * Initialise the LED GPIOs
 */
void leds_init(void)
{
    uint8_t i;

    for (i=0; i<NUM_CHANNELS; ++i)
    {
        GPIO_Input(leds[i].port, leds[i].pin);
    }
}

/*
 * red: bitmask of LEDs to set red
 * green:  bitmask of LEDs to set green
 * mask: bitmask of LEDs to set values for
 */
void leds_set(uint8_t red, uint8_t green, uint8_t mask)
{
    uint8_t i;

    for (i=0; i<NUM_CHANNELS; ++i)
    {
        if (mask & (1 << i) != 0)
        {
            if (red & (1 << i) != 0)
            {
                GPIO_Output(leds[i].port, leds[i].pin, 0);
            }
            else if (green & (1 << i) != 0)
            {
                GPIO_Output(leds[i].port, leds[i].pin, 1);
            }
            else
            {
                GPIO_Input(leds[i].port, leds[i].pin);
            }
        }
    }
}
