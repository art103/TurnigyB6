// core header file from our library project:
#include "stm8s.h"

#define NUM_CHANNELS 6

struct _pin {
	GPIO_TypeDef* port;
	GPIO_Pin_TypeDef pin;
};

static const struct _pin balancer[NUM_CHANNELS] =
{ {GPIOD, GPIO_PIN_3},
  {GPIOD, GPIO_PIN_2},
  {GPIOD, GPIO_PIN_0},
  {GPIOC, GPIO_PIN_7},
  {GPIOC, GPIO_PIN_6},
  {GPIOC, GPIO_PIN_5}
};

static const struct _pin leds[NUM_CHANNELS] =
{ {GPIOB, GPIO_PIN_7},
  {GPIOB, GPIO_PIN_6},
  {GPIOB, GPIO_PIN_3},
  {GPIOF, GPIO_PIN_4},
  {GPIOC, GPIO_PIN_2},
  {GPIOC, GPIO_PIN_3}
};

volatile uint32_t g_timer_tick = 0;

/*
 * System timer ISR
 */
void tim6_isr(void) __interrupt(23)
{
    TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	g_timer_tick++;
}

void delay_ms(uint32_t ms)
{
	uint32_t start = g_timer_tick;
	while (g_timer_tick < start + ms);
}

/*
 * Initialise the balancer GPIOs
 */
void balancer_init(void)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; ++i)
	{
		GPIO_Init(balancer[i].port, balancer[i].pin, GPIO_MODE_OUT_PP_LOW_SLOW);
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
}

/*
 * Initialise the LED GPIOs
 */
void leds_init(void)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; ++i)
	{
		GPIO_Init(leds[i].port, leds[i].pin, GPIO_MODE_IN_FL_NO_IT);
	}
}

/*
 * red: bitmask of LEDs to set red
 * green:  bitmask of LEDs to set green
 * mask: bitmask of LEDs to set values for
 */
void leds_set(uint8_t red, uint8_t green, uint8_t mask)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; ++i)
	{
		if (mask & (1 << i) != 0)
		{
			if (red & (1 << i) != 0)
			{
				GPIO_Init(leds[i].port, leds[i].pin, GPIO_MODE_OUT_PP_HIGH_SLOW);
			}
			else if (green & (1 << i) != 0)
			{
				GPIO_Init(leds[i].port, leds[i].pin, GPIO_MODE_OUT_PP_LOW_SLOW);
			}
			else
			{
				GPIO_Init(leds[i].port, leds[i].pin, GPIO_MODE_IN_FL_NO_IT);
			}
		}
	}
}

#define MUX_VALUES	8

volatile uint16_t adc_values[MUX_VALUES + 3];
uint8_t adc_input = 0;

void adc_isr(void) __interrupt(22)
{
	adc_values[adc_input] = ADC1_GetConversionValue();

	ADC1_ClearFlag(ADC1_FLAG_EOC);

	if (adc_input < MUX_VALUES)
	{
		// Set Mux Input
		GPIO_Write(GPIOA, adc_input);
		// Read Mux Value
		ADC1_ConversionConfig(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_6, ADC1_ALIGN_RIGHT);
	}
	else
	{
		switch (adc_input - MUX_VALUES)
		{
			case 0:
				// Read Batt Current
				ADC1_ConversionConfig(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_0, ADC1_ALIGN_RIGHT);
			break;

			case 1:
				// Read Batt Voltage
				ADC1_ConversionConfig(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_1, ADC1_ALIGN_RIGHT);
			break;

			case 2:
				// Read Input Voltage
				ADC1_ConversionConfig(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_2, ADC1_ALIGN_RIGHT);
			break;
		}
	}

	adc_input++;
	if (adc_input > MUX_VALUES + 2)
	{
		adc_input = 0;
	}

	ADC1_StartConversion();
}

void mux_init(void)
{
	// ADC Input pin (MUX Output)
	GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);

	// MUX Control pins
	GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_SLOW);
	GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW);
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_SLOW);


	ADC1_DeInit();
	ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_6, ADC1_PRESSEL_FCPU_D2, \
			ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL6,\
			DISABLE);

	/* Enable EOC interrupt */
	ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);

	/* Start Conversion */
	ADC1_StartConversion();
}

void pwm_init(void)
{
	// Configure BUCK and BOOST outputs as GPIO, Low.
	GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);	// Buck
	GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);	// Boost
}

void pwm_set(void)
{
	TIM1_DeInit();
	/* Time Base configuration */
	/*
	TIM1_Period = 4095
	TIM1_Prescaler = 0
	TIM1_CounterMode = TIM1_COUNTERMODE_UP
	TIM1_RepetitionCounter = 0
	*/

	TIM1_TimeBaseInit(0, TIM1_COUNTERMODE_UP, 399, 0);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */

	/*
	TIM1_OCMode = TIM1_OCMODE_PWM2
	TIM1_OutputState = TIM1_OUTPUTSTATE_ENABLE
	TIM1_OutputNState = TIM1_OUTPUTNSTATE_ENABLE
	TIM1_Pulse = CCR1_Val
	TIM1_OCPolarity = TIM1_OCPOLARITY_LOW
	TIM1_OCNPolarity = TIM1_OCNPOLARITY_HIGH
	TIM1_OCIdleState = TIM1_OCIDLESTATE_SET
	TIM1_OCNIdleState = TIM1_OCIDLESTATE_RESET

	*/
	TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
			   255, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
			   TIM1_OCNIDLESTATE_RESET);

	TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, 127, TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_SET);

	/* TIM1 counter enable */
	TIM1_Cmd(ENABLE);

	/* TIM1 Main Output Enable */
	TIM1_CtrlPWMOutputs(ENABLE);
}

int main(void)
{
    /* Reset GPIO ports to a default state */
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);
    GPIO_DeInit(GPIOE);
    GPIO_DeInit(GPIOF);

    /* Set CPU to 16MHz internal RC clock */
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
    CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);

	/* Configure the system timer (1ms ticks) */
	TIM4_DeInit();
	TIM4_TimeBaseInit(TIM4_PRESCALER_64, HSI_VALUE / 64 / 1000);
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
	TIM4_Cmd(ENABLE);


	/* Enable general interrupts */
	enableInterrupts();

	pwm_init();
    leds_init();
    mux_init();
    balancer_init();

    // Set Red and Green alternating pattern
    leds_set(0x2A, 0x15, 0x3F);

	// Disable the battery FET
	GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_SLOW);
	pwm_init();

	// Buzzer
	BEEP_Init(BEEP_FREQUENCY_4KHZ);
	BEEP_Cmd(ENABLE);
	BEEP_Cmd(DISABLE);

    // The main loop
    while(1)
    {
		delay_ms(1000);
        // Toggle the output pin
        GPIO_WriteReverse(GPIOD, GPIO_PIN_0);
    }
}
