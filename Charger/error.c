#include "stm8s.h"
#include "charger.h"
#include "pwm.h"
#include "leds.h"

/*
 * Flash the LEDs and beep out the error code.
 */
void error(uint8_t error_code)
{
	uint8_t i;

	// Turn off the PWM
	pwm_init();

	// Disable the battery FET
	GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_SLOW);

	while (1)
	{
		leds_set(0x2A, 0x15, 0x3F);

		// Beep the error code out
        BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
        BEEP->CSR |= BEEP_FREQUENCY_2KHZ;

		for (i=0; i<error_code; ++i)
		{
			// Buzzer
            BEEP->CSR |= BEEP_CSR_BEEPEN;
			delay_ms(100);
            BEEP->CSR &= ~BEEP_CSR_BEEPEN;
			delay_ms(100);
		}

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
