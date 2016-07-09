#include "stm8s.h"
#include "charger.h"

static const struct _pin leds[NUM_CHANNELS] =
{ {GPIOB, GPIO_PIN_7},
  {GPIOB, GPIO_PIN_6},
  {GPIOB, GPIO_PIN_3},
  {GPIOE, GPIO_PIN_5},
  {GPIOC, GPIO_PIN_2},
  {GPIOC, GPIO_PIN_3}
};

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
