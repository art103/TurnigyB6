#ifndef _CHARGER_H_
#define _CHARGER_H_

#define NUM_CHANNELS 	(6)		// Number of balance / LED channels.
#define MUX_VALUES		(6)   	// Number of values to read from Analogue Mux.

#define MAX_CELL_V		(4250)	// This is the abort voltage
#define MAX_CELL_V_CHG	(4170)	// This is the termination voltage
#define MIN_CELL_V		(2900)	// This is the minimum voltage to register a cell as usable
#define CELL_PRESENT_V	(1500)	// This is the value to register a cell as present
#define PACK_PRESENT_V	(2000)	// Value above which to register the main connector present.

#define INITIAL_CHARGE_CURRENT	(500) // This is the charge rate in mA before calculating the battery capacity.
#define MAX_CHARGE_CURRENT		(5000)
#define MAX_CHARGE_POWER		(50000000)
#define CHARGE_RATE		(1)		// This is the charge rate in C after calculating the battery capacity.

#define BALANCE_CHECK		(5000)
#define BALANCE_THRESHOLD	(5)	// Threshold in mV from the minimum cell value before balancing.

#define BATTERY_MEASURE_TIME	(30000)	// Number of ms to measure the battery voltage increase over.
#define BATTERY_MEASURE_DELAY	(10000)
#define BATTERY_CHECK_PERIOD	(120000)	// How often to stop balancing and check the capacity.

#define CHARGE_TIMEOUT			(5400000)	// How long to charge before aborting.

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

// Calculated from ADC channels every 100ms
extern uint8_t num_cells;

// Fast average (100ms)
extern uint32_t pwm_curr_avg;
extern uint16_t pwm_curr_count;

// Slow average (1s)
extern uint16_t average_count;
extern uint32_t batt_curr_avg;
extern uint32_t batt_vol_avg;
extern uint32_t input_vol_avg;
extern uint32_t balance_avg[NUM_CHANNELS];

extern uint16_t cell_min;
extern uint16_t cell_max;

// Calculated battery capacity.
extern uint16_t battery_capacity;

// Charger logic state
extern uint8_t balancing;
extern uint16_t target_current;

extern const uint16_t calibration[MUX_VALUES + 3];

void delay_ms(uint32_t ms);
void adc_sweep(void);
void update_lcd_info(void);

#endif // _CHARGER_H_
