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

#endif // _PWM_H_
