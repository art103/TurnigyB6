#ifndef _PWM_H_
#define _PWM_H_
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

void buzzer_init(void);
void buzzer_on(uint8_t freq);
void buzzer_off(void);

#endif // _PWM_H_
