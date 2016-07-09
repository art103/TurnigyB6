// core header file from our library project:
#include "stm8s.h"
#include "charger.h"

#include "adc.h"
#include "pwm.h"
#include "leds.h"
#include "balancer.h"
#include "error.h"
#include "ssd1306.h"

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

#define MAX_CELL_V		(4170)	// This is the termination voltage
#define MIN_CELL_V		(2900)	// This is the minimum voltage to register a cell as usable
#define CELL_PRESENT_V	(750)	// This is the value to register a cell as present

#define INITIAL_CHARGE_CURRENT	(500) // This is the charge rate in mA before calculating the battery capacity.
#define CHARGE_RATE		(1)		// This is the charge rate in C after calculating the battery capacity.

#define BALANCE_CHECK		(5000)
#define BALANCE_THRESHOLD	(8)	// Threshold in mV from the minimum cell value before balancing.

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

void process_balance_inputs(void)
{
	uint8_t i;
	uint32_t calc;
	uint16_t min = MAX_CELL_V;
	uint16_t max = 0;
	
	if ((last_balance_check + BALANCE_CHECK) < g_timer_tick)
	{
		if ((last_balance_check + BALANCE_CHECK - 100) > g_timer_tick)
		{
			// Turn off the balancer to re-check cell values.
			balancer_off();
		}
		return;
	}

	// Calculate the cell voltages
	for (i=0; i<NUM_CHANNELS; ++i)
	{
		// Apply per-channel calibration values
		calc = adc_values[i];
		calc *= balance_cal[i];
		calc /= 100;
		balance_cells[i] = calc;
		
		if (calc > CELL_PRESENT_V)
		{
			if (calc > max)
				max = calc;
			if (calc < min)
				min = calc;
		}
		else
		{
			balance_cells[i] = 0;
		}
	}
	
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
}

void process_battery_current(void)
{
    uint32_t current;
    current = adc_values[MUX_VALUES];
    battery_current = current * 1000 / 1024;
}

void process_battery_voltage(void)
{
    uint32_t voltage;
    voltage = adc_values[MUX_VALUES + 1];
    battery_voltage = voltage * 523 * 5 / 1024;
}

void process_input_voltage(void)
{
    uint32_t voltage;
    voltage = adc_values[MUX_VALUES + 2];
    input_voltage = voltage * 700 * 5 / 1024;
}

/*
 * Entry Point
 */
int main(void)
{
    uint16_t boost = 0;
    /* Reset GPIO ports to a default state */
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);
    GPIO_DeInit(GPIOE);
    GPIO_DeInit(GPIOF);

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
	TIM4_DeInit();
	TIM4_TimeBaseInit(TIM4_PRESCALER_64, HSI_VALUE / 64 / 1000);
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
	TIM4_Cmd(ENABLE);
#endif

	/* Enable general interrupts */
	enableInterrupts();

	pwm_init();
    leds_init();
    adc_init();
    balancer_init();

	GPIO_WriteHigh(GPIOD, GPIO_PIN_0);
    lcd_init();

    // Set Red and Green alternating pattern
    leds_set(0x2A, 0x15, 0x3F);

	// Disable the battery FET
	GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_SLOW);

	// Buzzer
    BEEP->CSR &= ~BEEP_CSR_BEEPDIV;
    BEEP->CSR |= BEEP_CALIBRATION_DEFAULT;

    BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
    BEEP->CSR |= BEEP_FREQUENCY_1KHZ;

	BEEP->CSR |= BEEP_CSR_BEEPEN;
	delay_ms(150);
    BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
    BEEP->CSR |= BEEP_FREQUENCY_2KHZ;

	delay_ms(100);
    BEEP->CSR &= ~BEEP_CSR_BEEPSEL;
    BEEP->CSR |= BEEP_FREQUENCY_4KHZ;
	delay_ms(50);

	BEEP->CSR &= ~BEEP_CSR_BEEPEN;

	//error(5);

	pwm_enable(TRUE);
	pwm_set(PWM_TIMER_BASE / 2, 0);

    // The main loop
    lcd_set_cusor(0,0);
    lcd_write_string(" B6 Compact+ Charger ", 0);

	// Battery and Input stats
    lcd_set_cusor(0, 56);
    lcd_write_string("Vi:", 1);
    lcd_set_cusor(60, 48);
    lcd_write_string("Ib:", 1);
    lcd_set_cusor(60, 56);
    lcd_write_string("Vb:", 1);

    while(1)
    {
        int i;
        uint16_t tmp1;
        uint16_t tmp2;

		for (i=0; i<NUM_CHANNELS; ++i)
		{
            tmp1 = balance_cells[i] / 10;
            tmp2 = tmp1 / 100;

			lcd_set_cusor(0, 8 * (i+1));
			lcd_write_digits(tmp2, 1);
			lcd_write_string(".", 1);
            lcd_write_digits(tmp1 - (tmp2 * 100), 1);
			lcd_write_string("   ", 1);
			lcd_set_cusor(24, 8 * (i+1));
			lcd_write_string("V ", 1);
			
			if (balancing & (1 << i))
			{
				lcd_write_string("Bal", 1);
			}
			else
			{
				lcd_write_string("   ", 1);
			}
		}

        leds_set(0x2A, 0x15, 0x3F);

		lcd_set_cusor(78, 48);
		lcd_write_digits(battery_current, 1);
		lcd_write_string("mA   ", 1);

		tmp1 = input_voltage;
		tmp2 = tmp1 / 100;
		lcd_set_cusor(18, 56);
		lcd_write_digits(tmp2, 1);
		lcd_write_char('.', 1);
		lcd_write_digits(tmp1 - (tmp2 * 100), 1);
		lcd_write_string("v ", 1);

		tmp1 = battery_voltage;
		tmp2 = tmp1 / 100;
		lcd_set_cusor(78, 56);
		lcd_write_digits(tmp2, 1);
		lcd_write_char('.', 1);
		lcd_write_digits(tmp1 - (tmp2 * 100), 1);
		lcd_write_string("v ", 1);

        leds_set(0x15, 0x2A, 0x3F);

		//delay_ms(50);
		adc_sweep();
    }
}

