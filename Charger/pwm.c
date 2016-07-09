#include "stm8s.h"
#include "charger.h"

/*
 * Initialize the Buck / Boost converter PWM outputs.
 */
void pwm_init(void)
{
    // Configure Timer 1 as 40KHz PWM timer (disabled).
	TIM1_DeInit();
	TIM1_TimeBaseInit(0, TIM1_COUNTERMODE_UP, PWM_TIMER_BASE, 0);

	// Configure BUCK and BOOST outputs as GPIO, Low (Off).
	GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);	// Buck
	GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);	// Boost
}

/*
 * Enable or Disable the PWM output.
 */
void pwm_enable(bool enable)
{
    if (enable)
    {
        /* Enable timer and PWM Outputs */
        TIM1_Cmd(ENABLE);
        TIM1_CtrlPWMOutputs(ENABLE);
    }
    else
    {
        /* Reset timer */
        pwm_init();
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

    TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
               buck, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
               TIM1_OCNIDLESTATE_RESET);

    TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, boost, TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_SET);
}
