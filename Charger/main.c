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

// ToDo: Move this to EEPROM.
uint16_t balance_cal[NUM_CHANNELS] = {968, 971, 975, 975, 975, 975};

volatile uint32_t g_timer_tick = 0;     // Global 1ms system timer tick

uint16_t input_voltage;
uint16_t battery_voltage;
uint16_t battery_current;

// Averages
uint32_t pwm_curr_avg;
uint16_t pwm_curr_count;
uint32_t batt_vol_avg;
uint16_t batt_vol_cnt;
uint16_t balance_avg_count;
uint32_t balance_avg[NUM_CHANNELS];
uint16_t cell_max;
uint16_t cell_min;


uint16_t battery_capacity;					// mAh calculated battery capacity.
uint8_t balancing;	// Bitmask of cahnnels being balanced.

State state = STATE_CHECKING;
uint8_t num_cells;
uint8_t timeout = 0;
uint32_t charge_start;
uint16_t pack_start;

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
	uint8_t i;
	uint16_t tmp1;
	uint16_t tmp2;

	for (i=0; i<NUM_CHANNELS; ++i)
	{
		tmp1 = balance_avg[i];
		tmp2 = tmp1 / 1000;

		lcd_set_cusor(88, 8 * (i+1));
		lcd_write_string("      ", 1);
		lcd_set_cusor(88, 8 * (i+1));
		lcd_write_digits(tmp2, 1);
		lcd_write_char('.', 1);
		lcd_write_digits(tmp1 - (tmp2 * 1000), 1);

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


void monitor_pack(void)
{
	uint8_t i;

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
			// Check to see if any cells are below the safe charge voltage
			if ((i > 0 && adc_values[i-1] == 0) ||
			    (adc_values[i] < MIN_CELL_V))
			{
				ok = 0;
				if (timeout++ > 3) // 3s timeout
				{
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
					if (timeout++ > 3)   // 3s timeout
					{
						error(ERROR_PACK_VOLTAGE);
					}
				}
				else
				{
					switch (state)
					{
						case STATE_CHECKING:
                            if (timeout++ > 3)  // 3s timeout
                            {
                                // This seems to check out. Show the cell count
                                leds_set(0, (1 << num_cells) - 1, 0x3F);

                                // Start charge at measuring current.
                                battery_capacity = INITIAL_CHARGE_CURRENT;
                                pwm_set_current(INITIAL_CHARGE_CURRENT);

                                state = STATE_MEASURING;

                                // Delay starting the next measurement cycle whilst the current ramps up.
                                charge_start = g_timer_tick + BATTERY_MEASURE_DELAY;
                                pack_start = battery_voltage;
                            }
						break;

						//case STATE_CHARGING:
						case STATE_MEASURING:
							// Let the battery settle to the new charge current.
							if (charge_start > g_timer_tick)
							{
                                pack_start = battery_voltage;
                            }

							// Keep an eye on the pack voltage and measure a 50mV/cell increase.
							// From this, calculate the pack mAh.
							if (charge_start + BATTERY_MEASURE_TIME < g_timer_tick)
							{
                                state = STATE_CHARGING;
                                leds_set((1 << num_cells) - 1, 0, 0x3F);
                                pwm_set_current(1400);
							#if 0
								uint32_t calc;
								uint16_t delta = battery_voltage - pack_start;

                                if (delta > 0)
                                {
                                    calc = (4200 - 3200);
                                    calc *= 3600;
                                    calc *= battery_current;
                                    calc /= g_timer_tick - charge_start;
                                    calc /= delta;
                                    battery_capacity = calc;
                                }

								//if (battery_capacity > 2 * battery_current)
								{
									charge_start = g_timer_tick;
									pack_start = battery_voltage;

									// Set charge rate to 1C
									//pwm_set_current(battery_capacity);

									// Delay starting the next measurement cycle whilst the current ramps up.
									charge_start = g_timer_tick + BATTERY_MEASURE_DELAY;

									state = STATE_CHARGING;
									leds_set((1 << num_cells) - 1, 0, 0x3F);
								}
								#endif
							}
						break;

                        case STATE_CHARGING:
                            if (cell_max >= 3950)
                            {
                                pwm_set_current(target_current - 100);
                            }
                            else if (battery_current < battery_capacity)
                            {
                                pwm_set_current(target_current + 50);
                            }

                            if (target_current <= battery_capacity / 10)
                                state = STATE_DONE;
                        break;

						case STATE_DONE:
                            error(ERROR_DONE);
						break;
					}
				}
			}
		}
	} // num_cells > 2
	else
	{
#ifdef ENABLE_EXTRA_LCD_INFO
		lcd_set_cusor(18, 8);
		lcd_write_string("None ", 1);
#endif // ENABLE_EXTRA_LCD_INFO

		if (state >= STATE_MEASURING)
		{
			error(ERROR_LOW_VOLTAGE_CELL);
		}
	}
}


/*
 * Entry Point
 */
int main(void)
{
	uint8_t i;
	uint32_t timer_10ms = 0;
	uint32_t timer_100ms = 0;
	uint32_t timer_1000ms = 0;

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
		if (timer_10ms < g_timer_tick)
		{
			timer_10ms = g_timer_tick + 10;
			adc_sweep();
		}

		// Tasks every 100ms.
		if (timer_100ms < g_timer_tick)
		{
			timer_100ms = g_timer_tick + 100;

			// Calculate the current average (for PWM use)
			battery_current = pwm_curr_avg / pwm_curr_count;
			pwm_curr_avg = 0;
			pwm_curr_count = 0;

			// Check and adjust current.
			pwm_run_pid();

			// Check to see if we have a balance port connected
			// and count the cells.
			num_cells = 0;
			for (i=0; i<NUM_CHANNELS; ++i)
			{
				if (adc_values[i] != 0)
				{
					num_cells++;
				}
			}
		}

		// Tasks every 1s.
		if (timer_1000ms < g_timer_tick)
		{
			timer_1000ms = g_timer_tick + 1000;

            // Average the balance cell values
            cell_max = MIN_CELL_V;
            cell_min = MAX_CELL_V;
            for (i=0; i<num_cells; ++i)
            {
                uint16_t avg = balance_avg[i] / balance_avg_count;
                if (avg > cell_max)
                    cell_max = avg;
                if (avg < cell_min)
                    cell_min = avg;
                balance_avg[i] = avg;
            }
            balance_avg_count = 0;

            battery_voltage = batt_vol_avg / batt_vol_cnt;
            batt_vol_avg = 0;
            batt_vol_cnt = 0;

			update_lcd_info();
			monitor_pack();
			balance_pack();
		}
	}
}

