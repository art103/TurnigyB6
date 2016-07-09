// core header file from our library project:
#include "stm8s.h"
#include "charger.h"
#include "error.h"
#include "adc.h"

#define NUMER_OF_SAMPLES    (256)
#define ADC_SETTLING_TIME	(10)

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
	GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);

	// MUX Control pins
	GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_SLOW);
	GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW);
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_SLOW);

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

	adc_done = 0;

	for (adc_input=0; adc_input<MUX_VALUES + 3; adc_input++)
	{
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
			if (adc_input == 0)
			{
				ADC1->CSR &= ~ADC1_CSR_CH;
				ADC1->CSR |= ADC1_CHANNEL_6;
			}
			delay_ms(ADC_SETTLING_TIME);
		}
		else
		{
			switch (adc_input - MUX_VALUES)
			{
				case 0:
					// Read Batt Current
					ADC1->CSR &= ~ADC1_CSR_CH;
					ADC1->CSR |= ADC1_CHANNEL_0;
					delay_ms(ADC_SETTLING_TIME);
				break;

				case 1:
					// Read Batt Voltage
					ADC1->CSR &= ~ADC1_CSR_CH;
					ADC1->CSR |= ADC1_CHANNEL_1;
					delay_ms(ADC_SETTLING_TIME);
				break;

				case 2:
					// Read Input Voltage
					ADC1->CSR &= ~ADC1_CSR_CH;
					ADC1->CSR |= ADC1_CHANNEL_2;
					delay_ms(ADC_SETTLING_TIME);
				break;
			}
		}

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
			case MUX_VALUES - 1:
				process_balance_inputs();
			break;

			// Battery Current
			case MUX_VALUES:
				process_battery_current();
			break;

			// Battery Voltage
			case MUX_VALUES + 1:
				process_battery_voltage();
			break;

			// Input Voltage
			case MUX_VALUES + 2:
				process_input_voltage();
			break;

			default:
			break;
		}
	}
}
