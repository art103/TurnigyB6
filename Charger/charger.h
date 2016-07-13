#ifndef _CHARGER_H_
#define _CHARGER_H_

#define NUM_CHANNELS 	(6)		// Number of balance / LED channels.
#define MUX_VALUES		(6)   	// Number of values to read from Analogue Mux.

#define MAX_CELL_V		(4200)	// This is the termination voltage
#define MIN_CELL_V		(2900)	// This is the minimum voltage to register a cell as usable
#define CELL_PRESENT_V	(1500)	// This is the value to register a cell as present
#define PACK_PRESENT_V	(2000)	// Value above which to register the main connector present.

#define INITIAL_CHARGE_CURRENT	(500) // This is the charge rate in mA before calculating the battery capacity.
#define CHARGE_RATE		(1)		// This is the charge rate in C after calculating the battery capacity.

#define BALANCE_CHECK		(5000)
#define BALANCE_THRESHOLD	(5)	// Threshold in mV from the minimum cell value before balancing.

#define BATTERY_MEASURE_TIME	(3000)	// Number of ms to measure the battery voltage increase over.
#define BATTERY_MEASURE_DELAY	(3000)

#define ENABLE_EXTRA_LCD_INFO	// Nice output, but may blow the flash limit.

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
	STATE_DONE
} State;

extern State state;

extern volatile uint16_t adc_values[MUX_VALUES + 3];   // Values read from ADC
extern uint16_t input_voltage;
extern uint16_t battery_voltage;
extern uint16_t battery_current;
extern uint32_t pwm_curr_avg;
extern uint16_t pwm_curr_count;
extern uint32_t batt_vol_avg;
extern uint16_t batt_vol_cnt;
extern uint16_t balance_avg_count;
extern uint32_t balance_avg[NUM_CHANNELS];
extern uint8_t balancing;
extern uint8_t num_cells;
extern uint16_t cell_min;
extern uint16_t cell_max;
extern uint16_t battery_capacity;
extern uint16_t target_current;

extern uint16_t balance_cal[NUM_CHANNELS];



void delay_ms(uint32_t ms);
void adc_sweep(void);
void update_lcd_info(void);

#endif // _CHARGER_H_
