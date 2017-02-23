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
#include "adc.h"

#define NUMER_OF_SAMPLES    (16)

volatile uint16_t adc_values[NUM_CHANNELS + 3];   // Values read from ADC
volatile uint8_t adc_done = 0;                  // ISR variable on completion
static volatile uint16_t conversions = 0;       // Conversions done
static volatile uint32_t sample_total = 0;      // Conversion average total.

/*
 * Hack function as this seems the only way to clear the flag.
 */
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
    ADC1_ClearFlag(ADC1_FLAG_EOC);  // WHY! Does only this work???

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
    // ADC Input pins
    GPIO_Input(GPIOD, GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6);
    GPIO_Input(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    ADC1->CR2 = ADC1_ALIGN_RIGHT;
    ADC1->CR1 = ADC1_PRESSEL_FCPU_D4;
    ADC1->TDRL = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 6); // Disable Schmidt Triggers

    // Turn on the ADC.
    ADC1->CR1 |= ADC1_CR1_ADON;

    // Enable ADC interrupts
    ADC1->CSR |= ADC1_IT_EOCIE;

    // Run the first sweep
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

    for (adc_input=0; adc_input<NUM_CHANNELS + 3; adc_input++)
    {
        ADC1->CSR &= ~ADC1_CSR_CH;
        switch (adc_input)
        {
            case 0:
                ADC1->CSR |= ADC1_CHANNEL_6;
            break;

            case 1:
                ADC1->CSR |= ADC1_CHANNEL_5;
            break;

            case 2:
                ADC1->CSR |= ADC1_CHANNEL_4;
            break;

            case 3:
                ADC1->CSR |= ADC1_CHANNEL_3;
            break;

            case 4:
                // Read Batt Current
                ADC1->CSR |= ADC1_CHANNEL_0;
            break;

            case 5:
                // Read Batt Voltage
                ADC1->CSR |= ADC1_CHANNEL_1;
            break;

            case 6:
                // Read Input Voltage
                ADC1->CSR |= ADC1_CHANNEL_2;
            break;
        }

        // Trigger the ADC and wait
        adc_done = 0;       // Work around spurious trigger.
        sample_total = 0;
        conversions = 0;
        ADC1->CR1 |= ADC1_CR1_ADON;
        while (adc_done == 0);

        adc_done = 0;

        adc_values[adc_input] = sample_total;
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
                // Apply per-channel calibration values
                tmp = (uint32_t)adc_values[adc_input] * (uint32_t)calibration[adc_input] / 100 / NUMER_OF_SAMPLES;

                // Only report cells that are above our detection threshold
                if (tmp < CELL_PRESENT_V)
                {
                    tmp = 0;
                }
                balance_avg[adc_input] += tmp;
                adc_values[adc_input] = tmp;
            break;

            // Battery Current
            case NUM_CHANNELS:
                tmp = adc_values[NUM_CHANNELS];
                tmp *= calibration[NUM_CHANNELS];
                pwm_curr = tmp / 100 / NUMER_OF_SAMPLES;
                batt_curr_avg += pwm_curr;
            break;

            // Battery Voltage
            case NUM_CHANNELS + 1:
                tmp = adc_values[NUM_CHANNELS + 1];
                tmp *= calibration[NUM_CHANNELS + 1];
                pwm_vol = tmp / 100 / NUMER_OF_SAMPLES;
                batt_vol_avg += pwm_vol;
            break;

            // Input Voltage
            case NUM_CHANNELS + 2:
                tmp = adc_values[NUM_CHANNELS + 2];
                tmp *= calibration[NUM_CHANNELS + 2];
                input_vol_avg += tmp / 100 / NUMER_OF_SAMPLES;
            break;

            default:
            break;
        }
    }

    // Increment average counter
    average_count++;
}
