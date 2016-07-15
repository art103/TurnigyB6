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
#include "pwm.h"
#include "leds.h"
#include "balancer.h"
#include "error.h"

/*
 * Flash the LEDs and beep out the error code.
 */
void error(uint8_t error_code)
{
    uint8_t i;

    // Turn off the PWM
    pwm_enable(FALSE);
    balancer_off();

    // Beep the error code out
    for (i=0; i<error_code; ++i)
    {
        // Buzzer
        buzzer_on(BEEP_FREQUENCY_2KHZ);
        delay_ms(10);
        buzzer_off();
        delay_ms(100);
    }

    if (error_code == ERROR_DONE)
    {
        leds_set(0x00, 0x3F, 0x3F);
    }
    else
    {
        state = STATE_ERROR;
        for (i=0; i<6; ++i)
        {
            // Set Red and Green alternating pattern
            leds_set(0x2A, 0x15, 0x3F);
            delay_ms(250);
            leds_set(0x15, 0x2A, 0x3F);
            delay_ms(250);
        }
    }
}
