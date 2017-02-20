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

#ifndef _PWM_H_
#define _PWM_H_

// PWM Timer reload value (10KHz)
#define PWM_TIMER_HZ	15000
#define PWM_TIMER_BASE  ((HSI_VALUE / PWM_TIMER_HZ) - 1)

/*
 * Initialize the Buck / Boost converter PWM outputs.
 */
void pwm_init(void);

/*
 * Enable or Disable the PWM output.
 */
void pwm_enable(bool enable);

/*
 * Set the PWM values.
 */
void pwm_set(uint16_t buck, uint16_t boost);

/*
 * Set the current to current mA (0-5000)
 */
void pwm_set_current(uint16_t current);

/*
 * Check the measured current and adjust
 * the PWM accordingly.
 */
void pwm_run_pid(void);

/*
 * Initialize the BEEP Peripheral
 */
void buzzer_init(void);

/*
 * Turn on the buzzer at freq (Enum).
 */
void buzzer_on(uint8_t freq);

/*
 * Turn off the buzzer.
 */
void buzzer_off(void);

#endif // _PWM_H_
