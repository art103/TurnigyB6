// core header file from our library project:
#include "stm8s.h"
#include "charger.h"

#include "adc.h"
#include "pwm.h"
#include "leds.h"
#include "balancer.h"
#include "gpio.h"
#include "error.h"
#include "ssd1306.h"

#define MAX_CELL_V		(4170)	// This is the termination voltage
#define MIN_CELL_V		(2900)	// This is the minimum voltage to register a cell as usable
#define CELL_PRESENT_V	(1500)	// This is the value to register a cell as present
#define PACK_PRESENT_V	(2000)	// Value above which to register the main connector present.

#define INITIAL_CHARGE_CURRENT	(500) // This is the charge rate in mA before calculating the battery capacity.
#define CHARGE_RATE		(1)		// This is the charge rate in C after calculating the battery capacity.

#define BALANCE_CHECK		(5000)
#define BALANCE_THRESHOLD	(8)	// Threshold in mV from the minimum cell value before balancing.

#define BATTERY_MEASURE_TIME	(60000)	// Number of ms to measure the battery voltage increase over.
#define BATTERY_MEASURE_DELAY	(10000)

uint16_t battery_capacity = 0;					// mAh calculated battery capacity.
uint16_t balance_drop[NUM_CHANNELS] = {0}; 		// mV drop when balancer was enabled.
uint16_t battery_current = 0;
uint16_t battery_voltage = 0;

uint8_t balancing;	// Bitmask of cahnnels being balanced.
uint16_t balance_cells[NUM_CHANNELS] = {0};     // mV of the cells

// ToDo: Move this to EEPROM.
uint16_t balance_cal[NUM_CHANNELS] = {978, 983, 988, 1001, 985, 985};

uint16_t input_voltage = 0;

uint32_t last_balance_check = 0;

volatile uint32_t g_timer_tick = 0;     // Global 1ms system timer tick

/*
 * System timer ISR
 */
void system_timer(void) __interrupt(23)
{
#if defined (STM8S903) || defined (STM8AF622x)
    TIM6->SR1 &= ~(TIM6_FLAG_UPDATE);
#else
    TIM4->SR1 &= ~(TIM4_FLAG_UPDATE);
#endif
	g_timer_tick++;
}

/* For some reason ISRs have to be in main??? */
void _i2c_isr(void) __interrupt(19)
{
    i2c_isr();
}

void _adc_isr(void) __interrupt(22)
{
    adc_isr();
}

/*
 * Delay function based on system timer
 */
void delay_ms(uint32_t ms)
{
	uint32_t start = g_timer_tick;
	while (g_timer_tick < start + ms);
}

void system_init(void)
{
    /* Set CPU to 16MHz internal RC clock */
    CLK->CKDIVR &= ~CLK_CKDIVR_HSIDIV;
    CLK->CKDIVR |= CLK_PRESCALER_HSIDIV1;
    CLK->CKDIVR &= ~CLK_CKDIVR_CPUDIV;
    CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;

	/* Configure the system timer (1ms ticks) */
#if defined (STM8S903) || defined (STM8AF622x)
    TIM6->ARR = HSI_VALUE / 64 / 1000;
    TIM6->PSCR = TIM6_PRESCALER_64;
	TIM6->IER |= TIM6_IT_UPDATE;
	TIM6->CR1 |= TIM6_CR1_CEN;
#else
    TIM4->ARR = HSI_VALUE / 64 / 1000;
    TIM4->PSCR = TIM6_PRESCALER_64;
	TIM4->IER |= TIM6_IT_UPDATE;
	TIM4->CR1 |= TIM6_CR1_CEN;
#endif

	/* Enable general interrupts */
	enableInterrupts();
}

void update_lcd_info(void)
{
	int i;
	uint16_t tmp1;
	uint16_t tmp2;

	for (i=0; i<NUM_CHANNELS; ++i)
	{
		tmp1 = balance_cells[i] / 10;
		tmp2 = tmp1 / 100;

		lcd_set_cusor(94, 8 * (i+1));
		lcd_write_string("     ", 1);
		lcd_set_cusor(94, 8 * (i+1));
		lcd_write_digits(tmp2, 1);
		lcd_write_char('.', 1);
		lcd_write_digits(tmp1 - (tmp2 * 100), 1);
		lcd_write_char('V', 1);
		lcd_set_cusor(82, 8 * (i+1));
		if (balancing & (1 << i))
		{
			lcd_write_char('B', 1);
		}
		else
		{
			lcd_write_char(' ', 1);
		}
	}

	// Battery Voltage
	tmp1 = battery_voltage;
	tmp2 = tmp1 / 1000;
	lcd_set_cusor(18, 16);
	lcd_write_digits(tmp2, 1);
	lcd_write_char('.', 1);
	lcd_write_digits(tmp1 - (tmp2 * 1000), 1);
	lcd_write_string("v ", 1);

	// Battery Current
	lcd_set_cusor(18, 24);
	lcd_write_digits(battery_current, 1);
	lcd_write_string("mA   ", 1);

	// Battery Capacity
	lcd_set_cusor(18, 32);
	lcd_write_digits(battery_capacity, 1);
	lcd_write_string("mAh   ", 1);

	// Input Voltage
	tmp1 = input_voltage;
	tmp2 = tmp1 / 1000;
	lcd_set_cusor(18, 40);
	lcd_write_digits(tmp2, 1);
	lcd_write_char('.', 1);
	lcd_write_digits(tmp1 - (tmp2 * 1000), 1);
	lcd_write_string("v ", 1);

#ifdef ENABLE_EXTRA_LCD_INFO
	lcd_set_cusor(0, 56);
	switch (state)
	{
		case STATE_ERROR:
			lcd_write_string("** Error **", 1);
		break;
		case STATE_CHECKING:
			lcd_write_string("Checking   ", 1);
		break;
		case STATE_MEASURING:
			lcd_write_string("Measuring  ", 1);
		break;
		case STATE_CHARGING:
			lcd_write_string("Charging   ", 1);
		break;
	}
#endif
}

void process_balance_inputs(void)
{
	uint8_t i;
	uint16_t min = MAX_CELL_V;
	uint16_t max = 0;

	// Calculate the cell voltages
	for (i=0; i<NUM_CHANNELS; ++i)
	{
		// Apply per-channel calibration values
		balance_cells[i] = (uint32_t)adc_values[i] * (uint32_t)balance_cal[i] / 100;

		// Only report cells that are above our detection threshold
		if (balance_cells[i] > CELL_PRESENT_V)
		{
			if (balance_cells[i] > max)
				max = balance_cells[i];
			if (balance_cells[i] < min)
				min = balance_cells[i];
		}
		else
		{
			balance_cells[i] = 0;
		}
	}
}

void process_battery_current(void)
{
    uint32_t current;
    current = adc_values[MUX_VALUES];
    battery_current = current * 10000 / 1150;
}

void process_battery_voltage(void)
{
    uint32_t voltage;
    voltage = adc_values[MUX_VALUES + 1];
    voltage = voltage * 523 * 50 / 1024;
    battery_voltage = voltage;
}

void process_input_voltage(void)
{
    uint32_t voltage;
    voltage = adc_values[MUX_VALUES + 2];
    voltage *= 700;
    voltage *= 50;
    voltage /= 1024;
    input_voltage = voltage;
}

State state = STATE_CHECKING;
uint8_t num_cells = 0;
uint8_t damaged_cell = 0;
uint32_t charge_start = 0;
uint16_t pack_start = 0;

void monitor_pack(void)
{
	int i;

	// We have detected cells. Check if it is safe to charge.
	if (num_cells >= 2)
	{
		uint8_t ok = 1;	// Assume everything is ok.

		lcd_set_cusor(18, 8);
		lcd_write_digits(num_cells, 1);
		lcd_write_string("S   ", 1);

		// Iterate through the cells, checking them.
		for (i=0; i<num_cells; ++i)
		{
			// Check to see if we have a dead / missing cell
			if (i > 0 && balance_cells[i-1] == 0)
			{
				ok = 0;
				damaged_cell++;
				if (damaged_cell > 3)
				{
					// The cells have been detected as invalid for
					// 3s. Fail with a fatal error code.
					error(ERROR_DAMAGED_CELL);
				}
			}

			// Check to see if any cells are below the safe charge voltage
			if (balance_cells[i] < MIN_CELL_V)
			{
				ok = 0;
				damaged_cell++;
				if (damaged_cell > 3)
				{
					// The cell has been detected as invalid for
					// 3s. Fail with a fatal error code.
					error(ERROR_LOW_VOLTAGE_CELL);
				}
			}
		}

		if (ok)
		{
			// Check to see that our battery voltage is within the
			// correct range for the detected cells.
			if (battery_voltage > MIN_CELL_V * num_cells)
			{
				if (battery_voltage > MAX_CELL_V * num_cells)
				{
					// The battery voltage doesn't match the cell count.
					ok = 0;
					damaged_cell++;
					if (damaged_cell > 3)
					{
						// The cell has been detected as invalid for
						// 3s. Fail with a fatal error code.
						error(ERROR_PACK_VOLTAGE);
					}
				}
				else
				{
					switch (state)
					{
						case STATE_CHECKING:
							// Everything seems ok, reset previous error(s)
							// Glitches when inserting balance lead can show
							// as missing cells.
							damaged_cell = 0;

							// This seems to check out. Show the cell count
							leds_set(0, (1 << num_cells) - 1, 0x3F);

							pwm_set_current(750);

							state = STATE_MEASURING;

							// Delay starting the next measurement cycle whilst the current ramps up.
							charge_start = g_timer_tick + BATTERY_MEASURE_DELAY;
							pack_start = battery_voltage;
							
							// ToDo: Use the balance values!
						break;

						case STATE_CHARGING:
						case STATE_MEASURING:
							// Let the battery settle to the new charge current.
							if (charge_start > g_timer_tick)
							{
								pack_start = battery_voltage;
							}

							// Keep an eye on the pack voltage and measure a 1% increase.
							// From this, calculate the pack mAh.
							if (battery_voltage > pack_start && battery_voltage - pack_start >= 25)
							{
								// Each pack ADC step = 25.54mV (2S: 0.3% - 6S: 0.1%)
								// Each cell ADC step = 9.7mV (0.23%)
								battery_capacity = 36 * (uint32_t)battery_current * 3600000 / (g_timer_tick - charge_start);
								charge_start = g_timer_tick;
								pack_start = battery_voltage;

								// Set charge rate to 0.5C
								//pwm_set_current(5 * battery_capacity / 10);

								// Delay starting the next measurement cycle whilst the current ramps up.
								charge_start = g_timer_tick + BATTERY_MEASURE_DELAY;

								state = STATE_CHARGING;
								leds_set((1 << num_cells) - 1, 0, 0x3F);
							}
						break;
					}
				}
			}
		}
	} // num_cells > 2
	else
	{
		lcd_set_cusor(18, 8);
		lcd_write_string("None ", 1);

		if (state >= STATE_MEASURING)
		{
			error(ERROR_LOW_VOLTAGE_CELL);
		}
	}
}

void balance_pack(void)
{
/*
if ((last_balance_check + BALANCE_CHECK) < g_timer_tick)
{
	if ((last_balance_check + BALANCE_CHECK - 100) > g_timer_tick)
	{
		// Turn off the balancer to re-check cell values.
		balancer_off();
	}
	return;
}
*/
/*
if ((max - min) > BALANCE_THRESHOLD)
{
	// Work out which cells need reducing.
	for (i=0; i<NUM_CHANNELS; ++i)
	{
		if (balance_cells[i] > (min + BALANCE_THRESHOLD))
		{
			balancing |= (1 << i);
			balancer_set(1 << i, 1 << i);
		}
		else
		{
			balancing &= ~(1 << i);
			balancer_set(0, 1 << i);
		}
	}
}
*/
}

/*
 * Entry Point
 */
int main(void)
{
	uint32_t timer_10ms = 0;
	uint32_t timer_100ms = 0;
	uint32_t timer_1000ms = 0;
	int i;

	system_init();
	pwm_init();
    leds_init();
    adc_init();
    balancer_init();

	GPIO_WriteHigh(GPIOD, GPIO_PIN_0);
    lcd_init();

    // Set Red and Green alternating pattern
    leds_set(0x2A, 0x15, 0x3F);

	// Disable the battery FET
	GPIO_Output(GPIOB, GPIO_PIN_4, 1);

	// Buzzer
	buzzer_init();

    // The main loop
	while(1)
	{
		// Tasks every 10ms.
		if (timer_10ms + 10 < g_timer_tick)
		{
			timer_10ms = g_timer_tick;
			adc_sweep();
		}

		// Tasks every 100ms.
		if (timer_100ms + 100 < g_timer_tick)
		{
			timer_100ms = g_timer_tick;

			// Check and adjust current.
			pwm_run_pid();

			// Check to see if we have a balance port connected
			// and count the cells.
			num_cells = 0;
			for (i=0; i<NUM_CHANNELS; ++i)
			{
				if (balance_cells[i] != 0)
				{
					num_cells++;
				}
			}
		}

		// Tasks every 1s.
		if (timer_1000ms + 1000 < g_timer_tick)
		{
			timer_1000ms = g_timer_tick;

			update_lcd_info();
			monitor_pack();
			balance_pack();
		}
	}
}

