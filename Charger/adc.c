// core header file from our library project:
#include "stm8s.h"
#include "charger.h"
#include "error.h"
#include "gpio.h"
#include "adc.h"

#define NUMER_OF_SAMPLES    (16)
#define ADC_SETTLING_TIME	(0)

/*
0 - Cell 4
1 - Cell 5
2 - Cell 6
3 - Cell 3
4 - Cell 2
5 - Cell 2 NC?
6 - Cell 1
7 - Cell 1 NC?
*/
static const uint8_t cell_mapping[MUX_VALUES] = {6, 4, 3, 0, 1, 2};

volatile uint16_t adc_values[MUX_VALUES + 3];   // Values read from ADC
volatile uint8_t adc_done = 0;
static volatile uint16_t conversions = 0;
static volatile uint32_t sample_total = 0;

void ADC1_ClearFlag(ADC1_Flag_TypeDef Flag)
{
    ADC1->CSR &= (~Flag);
}

/*
 * ADC ISR
 */
void adc_isr(void)
{
	sample_total += ADC1->DRL | (ADC1->DRH << 8);
	conversions++;

	// Clear Flag
    //ADC1->CSR &= (~ADC1_FLAG_EOC);
	ADC1_ClearFlag(ADC1_FLAG_EOC);	// WHY! Does only this work???

	if (conversions >= NUMER_OF_SAMPLES)
    {
		adc_done = 1;
    }
    else
    {
    	ADC1->CR1 |= ADC1_CR1_ADON;
    }
}

/*
 * Initialize the MUX and ADC for reading balance channels,
 * Input Voltage, Battery Voltage and Battery Current.
 */
void adc_init(void)
{
	// ADC Input pin (MUX Output)
	GPIO_Input(GPIOD, GPIO_PIN_6);

	// MUX Control pins
	GPIO_Output(GPIOA, GPIO_PIN_1, 0);
	GPIO_Output(GPIOA, GPIO_PIN_2, 0);
	GPIO_Output(GPIOA, GPIO_PIN_3, 0);

	ADC1->CR2 = ADC1_ALIGN_RIGHT;
	ADC1->CR1 = ADC1_PRESSEL_FCPU_D4;
	ADC1->TDRL = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 6);	// Disable Schmidt Triggers

	// Turn on the ADC.
	ADC1->CR1 |= ADC1_CR1_ADON;

    // Enable ADC interrupts
    ADC1->CSR |= ADC1_IT_EOCIE;

	adc_sweep();
}

/*
 * This is quite the hack.
 * For some reason the ADC doesn't like it's inputs changed within the ISR.
 * So move all of that logic out to a blocking function... :(
 */
void adc_sweep(void)
{
	uint8_t adc_input;
    uint32_t tmp;

	adc_done = 0;

	for (adc_input=0; adc_input<MUX_VALUES + 3; adc_input++)
	{
		ADC1->CSR &= ~ADC1_CSR_CH;
		if (adc_input < MUX_VALUES)
		{
			// Set Mux Input
			if (cell_mapping[adc_input] & 0x01)
				GPIO_WriteHigh(GPIOA, GPIO_PIN_1);
			else
				GPIO_WriteLow(GPIOA, GPIO_PIN_1);

			if (cell_mapping[adc_input] & 0x02)
				GPIO_WriteHigh(GPIOA, GPIO_PIN_2);
			else
				GPIO_WriteLow(GPIOA, GPIO_PIN_2);

			if (cell_mapping[adc_input] & 0x04)
				GPIO_WriteHigh(GPIOA, GPIO_PIN_3);
			else
				GPIO_WriteLow(GPIOA, GPIO_PIN_3);

			// Read Mux Value
			ADC1->CSR |= ADC1_CHANNEL_6;
		}
		else
		{
			switch (adc_input - MUX_VALUES)
			{
				case 0:
					// Read Batt Current
					ADC1->CSR |= ADC1_CHANNEL_0;
				break;

				case 1:
					// Read Batt Voltage
					ADC1->CSR |= ADC1_CHANNEL_1;
				break;

				case 2:
					// Read Input Voltage
					ADC1->CSR |= ADC1_CHANNEL_2;
				break;
			}
		}
		delay_ms(ADC_SETTLING_TIME);

		// Trigger the ADC and wait
		adc_done = 0;		// Work around spurious trigger.
        sample_total = 0;
        conversions = 0;
		ADC1->CR1 |= ADC1_CR1_ADON;
		while (adc_done == 0);

		adc_done = 0;

        adc_values[adc_input] = sample_total / NUMER_OF_SAMPLES;
        sample_total = 0;
        conversions = 0;

		// Process the data when it is valid
		switch (adc_input)
		{
			// All 6 Balance inputs are ready.
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
				// Apply per-channel calibration values
				tmp = (uint32_t)adc_values[adc_input] * (uint32_t)calibration[adc_input] / 100;

				// Only report cells that are above our detection threshold
				if (tmp < CELL_PRESENT_V)
				{
					tmp = 0;
				}
				balance_avg[adc_input] += tmp;
				adc_values[adc_input] = tmp;
			break;

			// Battery Current
			case MUX_VALUES:
                tmp = adc_values[MUX_VALUES];
                tmp *= calibration[MUX_VALUES];
                pwm_curr_avg += tmp / 100;
                batt_curr_avg += tmp / 100;
			break;

			// Battery Voltage
			case MUX_VALUES + 1:
                tmp = adc_values[MUX_VALUES + 1];
                tmp *= calibration[MUX_VALUES + 1];
                batt_vol_avg += tmp / 1000;
			break;

			// Input Voltage
			case MUX_VALUES + 2:
                tmp = adc_values[MUX_VALUES + 2];
                tmp *= calibration[MUX_VALUES + 2];
                input_vol_avg += tmp / 100;
			break;

			default:
			break;
		}
	}
	
	// Increment average counters
	pwm_curr_count++;
	average_count++;
}
