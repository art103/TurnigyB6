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
		delay_ms(100);
		buzzer_off();
		delay_ms(100);
	}

    if (error_code == ERROR_DONE)
    {
        leds_set(0x3F, 0x00, 0x3F);
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
