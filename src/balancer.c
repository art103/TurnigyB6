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

// core header file from our library project:
#include "stm8s.h"
#include "charger.h"
#include "gpio.h"

/*
 * Mapping of balancer channels to GPIOs
 */
static const struct _pin balancer[NUM_CHANNELS] =
{ {GPIOD, GPIO_PIN_0},
  {GPIOC, GPIO_PIN_7},
  {GPIOC, GPIO_PIN_6},
  {GPIOC, GPIO_PIN_5}
};

/*
 * Initialise the balancer GPIOs
 */
void balancer_init(void)
{
    uint8_t i;

    for (i=0; i<NUM_CHANNELS; ++i)
    {
        GPIO_Output(balancer[i].port, balancer[i].pin, 0);
    }
}

/*
 * channels: bitmask of channel values (1 = on, 0 = off)
 * mask: bitmask of channels to set values for
 */
void balancer_set(uint8_t channels, uint8_t mask)
{
    uint8_t i;

    for (i=0; i<NUM_CHANNELS; ++i)
    {
        if (mask & (1 << i) != 0)
        {
            if (channels & (1 << i) != 0)
            {
                GPIO_WriteHigh(balancer[i].port, balancer[i].pin);
            }
            else
            {
                GPIO_WriteLow(balancer[i].port, balancer[i].pin);
            }
        }
    }
}

/*
 * Turn off the balancer
 */
void balancer_off(void)
{
    uint8_t i;
    for (i=0; i<NUM_CHANNELS; ++i)
    {
        GPIO_WriteLow(balancer[i].port, balancer[i].pin);
    }
    balancing = 0;
}


/*
 * Called every 5s.
 * Each cell ADC step ~= 10mV
 */
void balance_pack(void)
{
    static uint8_t call_count = 0;
    uint8_t i;

    call_count++;

    // Only balance during charging or after completion
    if (state >= STATE_CHARGING)
	{
		if (call_count == 3)
		{
			// Cells have had time to settle, check them again.
			if ((cell_max - cell_min) > BALANCE_THRESHOLD)
			{
				// Work out which cells need reducing.
				for (i=0; i<NUM_CHANNELS; ++i)
				{
					if (balance_avg[i] > (cell_min + BALANCE_THRESHOLD))
					{
						balancing |= (1 << i);
					}
				}
				balancer_set(balancing, 0x3F);
			}
			else
			{
				// All ok, check again later
				call_count = 0;
			}
		}
		else if (call_count >= 9)
		{
			// let the cells settle
			balancer_off();
			call_count = 0;
		}
    }
}
