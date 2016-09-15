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
#include "error.h"
#include "gpio.h"
#include "pwm.h"

// PWM Timer reload value (10KHz)
#define PWM_TIMER_HZ	15000
#define PWM_TIMER_BASE  ((HSI_VALUE / PWM_TIMER_HZ) - 1)

#if (PWM_TIMER_HZ < 500)
	#error "PWM: Need to re-consider variable storage sizes"
#endif

#define HISTORY_COUNT   (2)     // * 100ms.

uint16_t target_current = 0;
static int16_t buck_val = 0;
static int16_t boost_val = 0;

static int16_t last_delta = 0;
static uint8_t slow = 0;

/*
 * Initialize the Buck / Boost converter PWM outputs.
 */
void pwm_init(void)
{
    // Configure Timer 1 as 40KHz PWM timer (disabled).
    // Set the Autoreload value
	TIM1->ARRH = (uint8_t)(PWM_TIMER_BASE >> 8);
	TIM1->ARRL = (uint8_t)(PWM_TIMER_BASE);

    // Set the Prescaler value
    TIM1->PSCRH = 0;
    TIM1->PSCRL = 0;

    // Select the Counter Mode
	TIM1->CR1 = (uint8_t)((uint8_t)(TIM1->CR1 & (uint8_t)(~(TIM1_CR1_CMS | TIM1_CR1_DIR)))
						| (uint8_t)(TIM1_COUNTERMODE_UP));

    // Set the Repetition Counter value
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
        // Enable timer and PWM Outputs
        TIM1->CR1 |= TIM1_CR1_CEN;
        TIM1->BKR |= TIM1_BKR_MOE;

        // Enable the battery FET
        //GPIO_Output(GPIOB, GPIO_PIN_4, 0);
    }
    else
    {
        // Disable timer and PWM outputs
        TIM1->CR1 &= ~TIM1_CR1_CEN;
        TIM1->BKR &= (uint8_t)(~TIM1_BKR_MOE);

        // Configure BUCK and BOOST outputs as GPIO, Low (Off).
        GPIO_Output(GPIOC, GPIO_PIN_1, 0);  // Buck
        GPIO_Output(GPIOC, GPIO_PIN_4, 0);  // Boost

        buck_val = 0;
        boost_val = 0;

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

/*
 * Set the current to current mA (0-5000)
 */
void pwm_set_current(uint16_t current)
{
    if (current > 0 && target_current == 0)
    {
        pwm_enable(TRUE);
    }
    else
    {
        pwm_enable(FALSE);
        last_delta = 0;
        slow = 0;
        target_current = 0;
        return;
    }


    if (current > battery_capacity * CHARGE_RATE)
        current = battery_capacity * CHARGE_RATE;
    if (current > MAX_CHARGE_CURRENT)
        current = MAX_CHARGE_CURRENT;

    target_current = current;

    pwm_run_pid();
}

/*
 * Check the measured current and adjust
 * the PWM accordingly. This should be a PI, maybe D,
 * but is P only due to flash space.
 */

void pwm_run_pid(void)
{
    static int16_t delta_pid = 0;
    int16_t delta;
    uint32_t calc;

    if (target_current == 0)
    {
        return;
    }

    // Make sure we stay within the 50W power limit.
    calc = target_current;
    calc *= pwm_vol;
    if (calc > MAX_CHARGE_POWER)
    {
        calc = MAX_CHARGE_POWER / pwm_vol;
        target_current = calc;
    }

	delta = (int16_t)target_current - (int16_t)pwm_curr;

#if 0
    // Don't take too long to ramp through the BUCK phase if input < output voltage.
	if (battery_voltage > input_voltage && delta > 1000 && buck_val != PWM_TIMER_BASE)
	{
       buck_val = PWM_TIMER_BASE;
	}
#endif

    if (delta < -500)
    {
        error(ERROR_OVER_CURRENT);
    }

    if (delta > 0)
    {
        if (delta_pid < 0)
            delta_pid = 0;

        if ((last_delta >= delta) && ((last_delta - delta) < 7))
            delta_pid++;
        else
            delta_pid = 0;
    }
    else
    {
        if (delta_pid > 0)
            delta_pid = -delta_pid;
        else
            delta_pid--;
    }

    last_delta = delta;

    if (delta < 0)
    {
        delta = -delta;
    }

	if (delta >= 25)
	{
        if (buck_val == PWM_TIMER_BASE)
        {
            boost_val += delta_pid;

            // Limit the boost value to avoid short circuit.
            if (boost_val > (PWM_TIMER_BASE / 2))
            {
                boost_val = (PWM_TIMER_BASE / 2);
            }
            else if (boost_val < 0)
            {
				buck_val += boost_val;
				boost_val = 0;
			}
        }
        else
        {
            buck_val += delta_pid;
            if (buck_val > PWM_TIMER_BASE)
            {
                buck_val = PWM_TIMER_BASE;
            }
        }

        if (boost_val < 0)
        {
            boost_val = 0;
        }

        if (buck_val < 0)
        {
            buck_val = 0;
        }
    }

    pwm_set(buck_val, boost_val);
}

/*
 * Initialize the BEEP Peripheral
 */
void buzzer_init(void)
{
    BEEP->CSR &= ~BEEP_CSR_BEEPDIV;
    BEEP->CSR |= BEEP_CALIBRATION_DEFAULT;
}

/*
 * Turn on the buzzer at freq (Enum).
 */
void buzzer_on(uint8_t freq)
{
    BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
    BEEP->CSR |= freq;
    BEEP->CSR |= BEEP_CSR_BEEPEN;
}

/*
 * Turn off the buzzer.
 */
void buzzer_off(void)
{
    BEEP->CSR &= ~BEEP_CSR_BEEPEN;
}
