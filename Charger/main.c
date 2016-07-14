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
const uint16_t calibration[MUX_VALUES + 3] = {968, 971, 975, 992, 995, 995, 816, 2550, 3418};

// Global 1ms system timer tick
volatile uint32_t g_timer_tick = 0;

// Calculated from averages every 1s.
uint16_t input_voltage;             // Input voltage (averaged).
uint16_t battery_voltage;           // Battery voltage (averaged).
uint16_t battery_current;           // Battery current (averaged).

// Calculated every 100ms
uint8_t num_cells;                  // Number of cells on the balance port.

// Fast average (100ms)
uint32_t pwm_curr_avg;              // Battery current (100ms average).
uint16_t pwm_curr_count;            // Number of average counts.

// Slow average (1s)
uint16_t average_count;             // Number of average counts.
uint32_t batt_curr_avg;             // Battery current (avg total).
uint32_t batt_vol_avg;              // Battery voltage (avg total).
uint32_t input_vol_avg;             // Input voltage (avg total).
uint32_t balance_avg[NUM_CHANNELS]; // Balance port values (averaged).

uint16_t cell_max;                  // Maximum cell voltage (balance port).
uint16_t cell_min;                  // Minimum cell voltage (balance port).

uint16_t battery_capacity;          // Calculated battery capacity.

// Charger logic state
State state = STATE_CHECKING;       // Charger logic state.
uint32_t timeout = 0;               // Timeout used during pack monitoring.
uint32_t charge_start;              // Time that charge measurement was started.
uint8_t balancing;                  // Bitmask of channels being balanced.

// Very high accuracy averages for capacity calcualtion
uint32_t pack_start_avg;            // Starting voltage of pack during measurement.
uint16_t pack_start_cnt;
uint32_t pack_end_avg;              // Ending  voltage of pack during measurement.
uint16_t pack_end_cnt;
uint16_t pack_start;
uint16_t pack_end;

/*
 * System timer ISR
 * Increments a global timer value for driving time based work.
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

/*
 * Route the I2C ISR from here to LCD module.
 * SDCC seems to require ISR definitions in the same file as main().
 */
void _i2c_isr(void) __interrupt(19)
{
    i2c_isr();
}

/*
 * Route the ADC ISR from here to adc module.
 * SDCC seems to require ISR definitions in the same file as main().
 */
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

/*
 * Initialize the system clocks, timer and interrupts.
 */
void system_init(void)
{
    // Set CPU to 16MHz internal RC clock
    CLK->CKDIVR &= ~CLK_CKDIVR_HSIDIV;
    CLK->CKDIVR |= CLK_PRESCALER_HSIDIV1;
    CLK->CKDIVR &= ~CLK_CKDIVR_CPUDIV;
    CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;

    // Configure the system timer (1ms ticks)
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

    // Enable general interrupts
    enableInterrupts();
}

/*
 * When the LCD is enabled, this formats and displays the cell and
 * pack information on screen.
 */
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
        case STATE_DONE:
            lcd_write_string("Finished   ", 1);
        break;
    }
#endif
}

/*
 * Called once a second.
 * This function monitors the pack state and charges if safe to do so.
 * It is also responsible for measuring the pack capacity to set the
 * charge current.
 */
void monitor_and_charge_pack(void)
{
    uint8_t i;
    uint32_t calc;

    // Check to see if we have at least 2 cells.
    if (num_cells < 2)
    {
#ifdef ENABLE_EXTRA_LCD_INFO
        lcd_set_cusor(18, 8);
        lcd_write_string("None ", 1);
#endif // ENABLE_EXTRA_LCD_INFO

        // Was the balance port removed?
        if (state >= STATE_MEASURING)
        {
            error(ERROR_LOW_VOLTAGE_CELL);
        }
    }
    // We have detected cells. Check if it is safe to charge.
    else
    {
        uint8_t ok = 1; // Assume everything is ok for now.

        // Display the number of detected cells.
        lcd_set_cusor(18, 8);
        lcd_write_digits(num_cells, 1);
        lcd_write_string("S   ", 1);

        // Iterate through the balance connector checking the cells.
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

            // Check to see if we have an over voltage cell
            if (adc_values[i] > MAX_CELL_V)
            {
                ok = 0;
                if (timeout++ > 3) // 3s timeout
                {
                    error(ERROR_HIGH_VOLTAGE_CELL);
                }
            }
        }

        // Balance connector looks ok, now check the pack.
        if (ok)
        {
            // Check to see that our battery voltage is within the
            // correct range for the detected cells.
            if ((battery_voltage > MAX_CELL_V * num_cells)
                //|| ((battery_voltage < MIN_CELL_V * num_cells) && state > STATE_CHECKING) // Checked below before charging.
                )
            {
                // The battery voltage doesn't match the cell count.
                ok = 0;
                if (timeout++ > 3)   // 3s timeout
                {
                    error(ERROR_PACK_VOLTAGE);
                }
            }
            else if (battery_voltage > MIN_CELL_V * num_cells)
            {
                switch (state)
                {
                    case STATE_CHECKING:
                        if (timeout++ > 3)  // 3s timeout
                        {
                            // Start charge at measuring current.
                            battery_capacity = INITIAL_CHARGE_CURRENT;
                            pwm_set_current(INITIAL_CHARGE_CURRENT);

                            // Prepare the measurement cycle.
                            charge_start = g_timer_tick;
                            timeout = 0;
                            state = STATE_MEASURING;
                        }
                    break;

                    case STATE_MEASURING:
                        // Check to see if we hit our termination voltage.
                        if (cell_max >= MAX_CELL_V_CHG)
                        {
                            state = STATE_DONE;
                            error(ERROR_DONE);
                        }

                        // Set the Cell count LEDs to Green.
                        leds_set(0, (1 << num_cells) - 1, 0x3F);

                        // See where we are in the cycle
                        calc = g_timer_tick - charge_start;

                        if (calc < 5000)
                        {
                            // Let the battery settle to the new charge current.
                            pack_start_avg = 0;
                            pack_start_cnt = 0;
                        }
                        else if (calc < 10000)
                        {
                            // Take a 5s average of the voltage
                            pack_start_avg += battery_voltage;
                            pack_start_cnt++;
                        }
                        else if (calc < BATTERY_MEASURE_TIME - 5000)
                        {
                            // Twiddle Thumbs
                            pack_end_avg = 0;
                            pack_end_cnt = 0;
                        }
                        else if (calc < BATTERY_MEASURE_TIME)
                        {
                            // Take a 5s average of the voltage
                            pack_end_avg += battery_voltage;
                            pack_end_cnt++;
                        }
                        else
                        {
                            // Calculate the pack capacity.

                            pack_start_avg = pack_start_avg / pack_start_cnt;
                            pack_end_avg = pack_end_avg / pack_end_cnt;

                            if (pack_end_avg <= pack_start_avg)
                            {
                                // Start again (this result implies a > 40Ah pack!).
                                charge_start = g_timer_tick;
                            }
                            else
                            {
                                calc = (4200 - 3200);
                                calc *= 3600;
                                calc /= g_timer_tick - (charge_start + 15000);
                                calc *= battery_current;
                                calc /= pack_end_avg - pack_start_avg;
                                battery_capacity = calc;

                                charge_start = g_timer_tick;

                                // Set charge rate to 1C
                                //pwm_set_current(battery_capacity * CHARGE_RATE);
                                pwm_set_current(1400);

                                state = STATE_CHARGING;
                            }
                        }
                    break;

                    case STATE_CHARGING:
                        // Set the Cell count LEDs to Red.
                        leds_set((1 << num_cells) - 1, 0, 0x3F);

                        // Only apply the current reduction when balancing
                        // is off. (balancing is turned off regularly).
                        if (balancing == 0)
                        {
                            if (cell_max > MAX_CELL_V_CHG)
                            {
                                pwm_set_current(target_current - 100);
                            }
                            else if (battery_current < battery_capacity)
                            {
                                pwm_set_current(target_current + 10);
                            }
                        }

                        // Complete at 0.1C (or 100mA, whichever is higher)
                        if ((target_current < 100) || (target_current <= battery_capacity / 10))
                        {
                            state = STATE_DONE;
                            error(ERROR_DONE);
                        }

                        // Don't charge for longer than CHARGE_TIMEOUT
                        if (g_timer_tick > CHARGE_TIMEOUT)
                        {
                            error(ERROR_TIMEOUT);
                        }

                        // Re-measure pack regularly.
                        if (timeout++ > BATTERY_CHECK_PERIOD)
                        {
                            balancer_off();
                            // Prepare the measurement cycle.
                            charge_start = g_timer_tick;
                            state = STATE_MEASURING;
                            timeout = 0;
                        }
                    break;

                    case STATE_DONE:
                        // Nothing to see here.
                        state = STATE_DONE;
                    break;
                } // switch(state)
            } // Pack voltage range ok
        } // Balance cells ok
    } // num_cells > 2
}

/*
 * Monitors the input voltage to ensure that the supply (or battery)
 * is not over discharged.
 */
void monitor_input(void)
{
#if 0
    uint8_t input_cells = input_voltage / 3300;

    // Make sure we don't discharge the supply pack too far.
    if (state > STATE_CHECKING && (input_voltage < input_cells * 3500 || input_voltage < 10000))
#else
    if (state > STATE_CHECKING && input_voltage < 10000)
#endif
    {
        error(ERROR_INPUT_VOLTAGE);
    }
}

/*
 * Entry Point
 */
int main(void)
{
    uint8_t i;
    uint8_t scan = 0;
    uint32_t timer_10ms = 0;
    uint32_t timer_100ms = 0;
    uint32_t timer_1000ms = 0;

    system_init();
    pwm_init();
    leds_init();
    adc_init();
    balancer_init();

    // Initialize the LCD
    lcd_init();
#ifdef ENABLE_EXTRA_LCD_INFO
    lcd_set_cusor(0,0);
    lcd_write_string(" B6 Compact+ Charger ", 0);

    // Battery and Input stats
    lcd_set_cusor(0, 8);
    lcd_write_string("Cl:", 1);
    lcd_set_cusor(0, 16);
    lcd_write_string("Vb:", 1);
    lcd_set_cusor(0, 24);
    lcd_write_string("Ib:", 1);
    lcd_set_cusor(0, 32);
    lcd_write_string("Cb:", 1);
    lcd_set_cusor(0, 40);
    lcd_write_string("Vi:", 1);
#endif

    // Disable the battery FET
    GPIO_Output(GPIOB, GPIO_PIN_4, 1);

    // Buzzer
    buzzer_init();
    buzzer_on(BEEP_FREQUENCY_1KHZ);
    delay_ms(150);
    buzzer_on(BEEP_FREQUENCY_2KHZ);
    delay_ms(100);
    buzzer_off();

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

            if (state == STATE_CHECKING)
            {
                // Green Scanning sequence
                leds_set(0, 1 << scan, 0x3F);
                scan++;
                if (scan >= NUM_CHANNELS)
                {
                    scan = 0;
                }
            }
        }

        // Tasks every 1s.
        if (timer_1000ms < g_timer_tick)
        {
            timer_1000ms = g_timer_tick + 1000;

            // Average the balance cell values and find the min/max
            cell_max = MIN_CELL_V;
            cell_min = MAX_CELL_V;
            for (i=0; i<NUM_CHANNELS; ++i)
            {
                uint16_t avg = balance_avg[i] / average_count;
                if (i < num_cells)
                {
                    if (avg > cell_max)
                        cell_max = avg;
                    if (avg < cell_min)
                        cell_min = avg;
                }
                balance_avg[i] = avg;
            }

            // Calculate the current average (for Display use)
            battery_current = batt_curr_avg / average_count;
            batt_curr_avg = 0;

            // Calculate the battery voltage average
            battery_voltage = batt_vol_avg / average_count;
            batt_vol_avg = 0;

            // Calculate the input voltage average
            input_voltage = input_vol_avg / average_count;
            input_vol_avg = 0;

            // Reset the average counter
            average_count = 0;

            update_lcd_info();
            monitor_input();
            monitor_and_charge_pack();
            balance_pack();

            // Flash the cells that are balancing
            if (state >= STATE_CHARGING)
            {
                if ((scan++ % 2) == 0)
                {
                    leds_set(0, 0, balancing);
                }
                else if (state == STATE_DONE)
                {
                    leds_set(0, (1 << num_cells) - 1, 0x3F);
                }
            }
        }
    }
}

