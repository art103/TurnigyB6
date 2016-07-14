// core header file from our library project:
#include "stm8s.h"
#include "charger.h"
#include "gpio.h"

/*
 * Mapping of balancer channels to GPIOs
 */
static const struct _pin balancer[NUM_CHANNELS] =
{ {GPIOD, GPIO_PIN_3},
  {GPIOD, GPIO_PIN_2},
  {GPIOD, GPIO_PIN_0},
  {GPIOC, GPIO_PIN_7},
  {GPIOC, GPIO_PIN_6},
  {GPIOC, GPIO_PIN_5}
};

/*
 * Initialise the balancer GPIOs
 */
void balancer_init(void)
{
    uint8_t i;

    for (i=0; i<NUM_CHANNELS; ++i)
    {
        GPIO_Output(balancer[i].port, balancer[i].pin, 0);
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
    balancing = 0;
}


/*
 * Called every 5s.
 * Each cell ADC step ~= 10mV
 */
void balance_pack(void)
{
    static uint8_t call_count = 0;
    uint8_t i;

    call_count++;

    // Only balance during charging or after completion
    if (state >= STATE_CHARGING)
	{
		if (call_count == 2)
		{
			// Cells have had time to settle, check them again.
			if ((cell_max - cell_min) > BALANCE_THRESHOLD)
			{
				// Work out which cells need reducing.
				for (i=0; i<NUM_CHANNELS; ++i)
				{
					if (balance_avg[i] > (cell_min + BALANCE_THRESHOLD))
					{
						balancing |= (1 << i);
					}
				}
				balancer_set(balancing, 0x3F);
			}
			else
			{
				// All ok, check again later
				call_count = 0;
			}
		}
    }
    else if (call_count >= 7)
    {
        // let the cells settle
        balancer_off();
        call_count = 0;
    }
}
