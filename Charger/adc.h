#ifndef _ADC_H_
#define _ADC_H_

/*
 * Values read from ADC
 */
extern volatile uint16_t adc_values[MUX_VALUES + 3];

/*
 * ADC ISR
 * This ISR will cycle through all of the ADC inputs and MUX settings
 * storing the results in the adc_values[] array.
 */
void adc_isr(void);

/*
 * Initialize the MUX and ADC for reading balance channels,
 * Input Voltage, Battery Voltage and Battery Current.
 */
void adc_init(void);

/*
 * Sweep through all of the ADC inputs gathering over sampled
 * values.
 */
void adc_sweep(void);

#endif // _ADC_H_
