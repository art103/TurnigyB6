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

#ifndef _ADC_H_
#define _ADC_H_

/*
 * Values read from ADC
 */
extern volatile uint16_t adc_values[NUM_CHANNELS + 3];

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
