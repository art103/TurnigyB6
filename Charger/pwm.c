#include "stm8s.h"
#include "charger.h"
#include "error.h"
#include "gpio.h"
#include "pwm.h"

#define PWM_TIMER_BASE	((HSI_VALUE / 10000) - 1)	// PWM Timer reload value (40KHz)

/*
 * Initialize the Buck / Boost converter PWM outputs.
 */
void pwm_init(void)
{
    // Configure Timer 1 as 40KHz PWM timer (disabled).
	/* Set the Autoreload value */
	TIM1->ARRH = (uint8_t)(PWM_TIMER_BASE >> 8);
	TIM1->ARRL = (uint8_t)(PWM_TIMER_BASE);

	/* Set the Prescaler value */
	TIM1->PSCRH = 0;
	TIM1->PSCRL = 0;

	/* Select the Counter Mode */
	TIM1->CR1 = (uint8_t)((uint8_t)(TIM1->CR1 & (uint8_t)(~(TIM1_CR1_CMS | TIM1_CR1_DIR)))
						| (uint8_t)(TIM1_COUNTERMODE_UP));

	/* Set the Repetition Counter value */
	TIM1->RCR = 0;

	// Setup OC1.
    TIM1->CCER1 = 0x03;
	TIM1->CCMR1 = TIM1_OCMODE_PWM2;

	// Setup OC4
	TIM1->CCER2 = 0x30;
	TIM1->CCMR4 = TIM1_OCMODE_PWM2;

    TIM1->OISR = 0x41;

	pwm_enable(FALSE);
}

/*
 * Enable or Disable the PWM output.
 */
void pwm_enable(bool enable)
{
    if (enable)
    {
        /* Enable timer and PWM Outputs */
        TIM1->CR1 |= TIM1_CR1_CEN;
        TIM1->BKR |= TIM1_BKR_MOE;

		// Enable the battery FET
		//GPIO_Output(GPIOB, GPIO_PIN_4, 0);
    }
    else
    {
		/* Disable timer and PWM outputs */
		TIM1->CR1 &= ~TIM1_CR1_CEN;
		TIM1->BKR &= (uint8_t)(~TIM1_BKR_MOE);

		// Configure BUCK and BOOST outputs as GPIO, Low (Off).
		GPIO_Output(GPIOC, GPIO_PIN_1, 0);	// Buck
		GPIO_Output(GPIOC, GPIO_PIN_4, 0);	// Boost

		// Disable the battery FET
		//GPIO_Output(GPIOB, GPIO_PIN_4, 1);
    }
}

/*
 * Set the PWM values.
 */
void pwm_set(uint16_t buck, uint16_t boost)
{
	if (buck > PWM_TIMER_BASE)
	{
		buck = PWM_TIMER_BASE;
	}

	if (boost > PWM_TIMER_BASE)
	{
		boost = PWM_TIMER_BASE;
	}

	// Set OC1 Values
	TIM1->CCR1H = (uint8_t)(buck >> 8);
	TIM1->CCR1L = (uint8_t)buck;

	// Set OC4 Values
	TIM1->CCR4H = (uint8_t)(boost >> 8);
	TIM1->CCR4L = (uint8_t)boost;
}

uint16_t target_current = 0;
static uint16_t buck_val = 0;
static uint16_t boost_val = 0;

/*
 * Set the current to current mA (0-5000)
 */
void pwm_set_current(uint16_t current)
{
	pwm_enable(TRUE);

    if (current > battery_capacity)
        current = battery_capacity;

	target_current = current;
	if (battery_voltage > input_voltage)
	{
		buck_val = PWM_TIMER_BASE;
	}
	else
	{
		buck_val = PWM_TIMER_BASE * battery_voltage / input_voltage;
	}

	pwm_run_pid();
}

/*
 * Check the measured current and adjust
 * the PWM accordingly.
 */
void pwm_run_pid(void)
{
	if (target_current > battery_current + (5 * (uint32_t)target_current / 100))	// Stop oscillations.
	{
		if (buck_val == PWM_TIMER_BASE)
		{
			boost_val++;

			// Limit the boost value to avoid short circuit.
			if (boost_val > (PWM_TIMER_BASE / 2))
			{
				boost_val = (PWM_TIMER_BASE / 2);
			}
		}
		else
		{
			buck_val++;
			if (buck_val > PWM_TIMER_BASE)
			{
				buck_val = PWM_TIMER_BASE;
			}
		}
	}
	else if (target_current < battery_current)
	{
		if (buck_val == PWM_TIMER_BASE)
		{
			if (boost_val > 0)
			{
				boost_val--;
			}
			else
			{
				buck_val--;
			}
		}
		else
		{
			if (buck_val > 0)
			{
				buck_val--;
			}
		}
	}

	pwm_set(buck_val, boost_val);
}

void buzzer_init(void)
{
    BEEP->CSR &= ~BEEP_CSR_BEEPDIV;
    BEEP->CSR |= BEEP_CALIBRATION_DEFAULT;

    BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
    BEEP->CSR |= BEEP_FREQUENCY_1KHZ;

	BEEP->CSR |= BEEP_CSR_BEEPEN;
	delay_ms(150);
    BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
    BEEP->CSR |= BEEP_FREQUENCY_2KHZ;

	delay_ms(100);
    BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
    BEEP->CSR |= BEEP_FREQUENCY_4KHZ;
	delay_ms(50);

	BEEP->CSR &= ~BEEP_CSR_BEEPEN;
}
