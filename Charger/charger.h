#ifndef _CHARGER_H_
#define _CHARGER_H_

#define NUM_CHANNELS 	(6)		// Number of balance / LED channels.
#define MUX_VALUES		(6)   	// Number of values to read from Analogue Mux.

//#define ENABLE_EXTRA_LCD_INFO	// Nice output, but may blow the flash limit.

struct _pin {
	GPIO_TypeDef* port;
	GPIO_Pin_TypeDef pin;
};

typedef enum
{
	STATE_ERROR,
	STATE_CHECKING,
	STATE_MEASURING,
	STATE_CHARGING,
} State;

extern State state;

extern volatile uint16_t adc_values[MUX_VALUES + 3];   // Values read from ADC
extern uint16_t battery_current;
extern uint16_t battery_voltage;
extern uint16_t input_voltage;


void delay_ms(uint32_t ms);
void process_balance_inputs(void);
void process_battery_current(void);
void process_battery_voltage(void);
void process_input_voltage(void);

void adc_sweep(void);
void update_lcd_info(void);


#endif // _CHARGER_H_
