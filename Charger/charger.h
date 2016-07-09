#ifndef _CHARGER_H_
#define _CHARGER_H_

#define NUM_CHANNELS 	(6)		// Number of balance / LED channels.
#define MUX_VALUES		(6)   	// Number of values to read from Analogue Mux.

#define PWM_TIMER_BASE	((HSI_VALUE / 40000) - 1)	// PWM Timer reload value (40KHz)

struct _pin {
	GPIO_TypeDef* port;
	GPIO_Pin_TypeDef pin;
};

extern volatile uint16_t adc_values[MUX_VALUES + 3];   // Values read from ADC

void delay_ms(uint32_t ms);
void process_balance_inputs(void);
void process_battery_current(void);
void process_battery_voltage(void);
void process_input_voltage(void);


#endif // _CHARGER_H_
