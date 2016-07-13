#include "stm8s.h"

#include "ssd1306.h"
#include "lcdfont_medium.h"
#include "charger.h"

#define LCD_WIDTH 128
#define LCD_HEIGHT 64

#define LCD_WHITE			0
#define LCD_BLACK			1

#define MAX_BUFFER_SIZE		(8)
#define SLAVE_ADDRESS		(0x3C << 1)

#define CHAR_HEIGHT 7
#define CHAR_WIDTH  5

static bool initialized = FALSE;

static volatile uint8_t i2c_buffer[MAX_BUFFER_SIZE];
static volatile uint8_t i2c_buffer_index = 0;
static volatile uint8_t i2c_bytes = 0;

extern volatile uint32_t g_timer_tick;
extern void delay_ms(uint32_t ms);

I2C_Event_TypeDef I2C_GetLastEvent(void)
{
	if ((I2C->SR2 & I2C_SR2_AF) != 0x00)
	{
		return I2C_EVENT_SLAVE_ACK_FAILURE;
	}
	else
	{
		return (I2C->SR3 << 8 | I2C->SR1);
	}
}

/**
  * @brief  I2C Interrupt routine
  * @param None
  * @retval
  * None
  */
void i2c_isr(void)
{
	switch (I2C_GetLastEvent())
	{
		/* EV5 */
		case I2C_EVENT_MASTER_MODE_SELECT :
			/* Send slave Address for write */
			I2C->DR = SLAVE_ADDRESS |I2C_DIRECTION_TX;
		break;

		/* EV6 */
		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
			if (i2c_bytes != 0)
			{
				/* Send the first Data */
				I2C->DR = i2c_buffer[i2c_buffer_index++];

				/* Decrement number of bytes */
				i2c_bytes--;
			}
			if (i2c_bytes == 0)
			{
				I2C->ITR &= ~I2C_IT_BUF;
			}
		break;

		/* EV8 */
		case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
			/* Transmit Data */
			I2C->DR = i2c_buffer[i2c_buffer_index++];

			/* Decrement number of bytes */
			i2c_bytes--;

			if (i2c_bytes == 0)
			{
				I2C->ITR &= ~I2C_IT_BUF;
			}
		break;

		/* EV8_2 */
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
			/* Send STOP condition */
			I2C->CR2 |= I2C_CR2_STOP;

			I2C->ITR &= ~I2C_IT_EVT;
		break;

		default:
		break;
	}
}

static int ssd1306_data_block(uint8_t *dat, uint16_t len, uint8_t is_data)
{
	uint8_t i;
	uint8_t msg;
	uint32_t timeout = g_timer_tick;

	if (is_data != 0)
	  msg = 0x40;
	else
	  msg = 0x00;

	i2c_bytes = len + 1;
	i2c_buffer_index = 0;
	i2c_buffer[0] = msg;

	for (i=0; i<len; ++i)
	{
		i2c_buffer[1+i] = dat[i];
	}

	/* Enable Buffer and Event Interrupt*/
	I2C->ITR |= I2C_IT_EVT | I2C_IT_BUF;

	/* Send START condition */
	I2C->CR2 |= I2C_CR2_START;

	// Wait for the transfer to complete.
	while (i2c_bytes)
	{
		if (timeout + 50 < g_timer_tick)
			break;
	}
	while ((I2C->SR3 & (I2C_FLAG_BUSBUSY & 0xff)) != 0)
	{
		if (timeout + 50 < g_timer_tick)
			break;
	}

	if (timeout + 50 < g_timer_tick)
		return 1;
	return 0;
}

static int ssd1306_command(uint8_t cmd)
{
	if (initialized)
		return ssd1306_data_block(&cmd, 1, 0);
	return -1;
}

static int ssd1306_data(uint8_t *dat, uint16_t len)
{
	if (initialized)
		return ssd1306_data_block(dat, len, 1);
	return -1;
}

int lcd_init(void)
{
	int tries;
	int ret;

	/* I2C Initialize */
	
	/* Set input freq and disable */
	I2C->FREQR = 16;
	I2C->CR1 &= ~I2C_CR1_PE;

	/* Set rise time */
	I2C->TRISER = 6;

	/* Configure divisor */
	I2C->CCRL = 13;
	I2C->CCRH = I2C_CCRH_FS;

	/* Enable I2C */
	I2C->CR1 |= I2C_CR1_PE;

    /* Enable the acknowledgement */
    I2C->CR2 |= I2C_CR2_ACK;
	/* Configure (N)ACK on current byte */
	I2C->CR2 &= ~I2C_CR2_POS;
	
	/* Own address */
	I2C->OARL = 0xA0;
	I2C->OARH = I2C_ADDMODE_7BIT | I2C_OARH_ADDCONF;
                   
	// 128x64 OLED "Crius"
	initialized = TRUE;

	GPIO_WriteLow(GPIOD, GPIO_PIN_0);

	// Wait up to 1.5s for the LCD to respond
	for (tries = 0; tries < 15; tries++)
	{
		ret = ssd1306_command(SSD1306_CMD_SET_DISPLAY_OFF);
		if (ret == 0)
			break;
		delay_ms(100);
	}

	if (ret != 0)
	{
		initialized = FALSE;
		return ret;
	}

	ssd1306_command(SSD1306_CMD_SET_MULTIPLEX);
	ssd1306_command(0x3F);
	ssd1306_command(SSD1306_CMD_SET_DISPLAY_OFFSET);
	ssd1306_command(0x0);
	ssd1306_command(SSD1306_CMD_SET_DISPLAY_START | 0x0);
	ssd1306_command(SSD1306_CMD_SET_SEG_REMAP);
	ssd1306_command(SSD1306_CMD_SET_COM_SCAN_DEC);
	ssd1306_command(SSD1306_CMD_SET_COM_PINS);
	ssd1306_command(0x12);
	ssd1306_command(SSD1306_CMD_SET_CONTRAST_BANK0);
	ssd1306_command(0x7F);	// 0x7f

	ssd1306_command(SSD1306_CMD_ENTIRE_DISPLAY_RESUME);
	ssd1306_command(SSD1306_CMD_SET_NORMAL);

	// Set Clock Freq
	ssd1306_command(SSD1306_CMD_SET_CLOCK_DIVIDER);
	ssd1306_command(0xF0); // 0x80

	ssd1306_command(SSD1306_CMD_SET_COLUMN);
	ssd1306_command(0);
	ssd1306_command(127);
	ssd1306_command(SSD1306_CMD_SET_PAGE);
	ssd1306_command(0);
	ssd1306_command(7);

	ssd1306_command(SSD1306_CMD_SET_MEMORY_MODE);
	ssd1306_command(0x00); // Horizontal

	ssd1306_command(SSD1306_CMD_SET_PRECHARGE);
	ssd1306_command(0x22);
	ssd1306_command(SSD1306_CMD_SET_VCOM_LEVEL);
	ssd1306_command(0x20);

	ssd1306_command(SSD1306_CMD_SET_FADE_BLINK);
	ssd1306_command(0);

	// Enable Charge Pump
	ssd1306_command(SSD1306_CHARGEPUMP); // 0x8D
	ssd1306_command(0x14);
	// Turn on the Output
	ssd1306_command(SSD1306_CMD_SET_DISPLAY_ON);

	ssd1306_command(SSD1306_CMD_SET_ZOOM_IN);
	ssd1306_command(0x00);

	ssd1306_command(SSD1306_CMD_SET_LOW_COLUMN | 0x00); // low col
	ssd1306_command(SSD1306_CMD_SET_HIGH_COLUMN | 0x00); // hi col
	ssd1306_command(SSD1306_CMD_SET_PAGE_START | 0x00); // row

	lcd_clear(0);

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
	return ret;
}

void lcd_write_char(uint8_t c, uint8_t colour)
{
	uint8_t x;
	uint8_t d;

	c -= 32;

	// Send the character.
	for (x=0; x<CHAR_WIDTH; x++ ) {
		d = font_medium[(c*CHAR_WIDTH)+x];
		if (colour == 0)
			d = ~d;
		ssd1306_data(&d, 1);
	}
	// Set a 1 line space after the character.
	if (colour == 1) d = 0;
	else d = 0xff;
	ssd1306_data(&d, 1);
}

void lcd_write_string(char *s, uint8_t colour)
{
	char *ptr = s;

	for (ptr = s; *ptr != 0; ptr++)
		lcd_write_char(*ptr, colour);
}

uint8_t tth = 0;
uint8_t th = 0;
uint8_t h = 0;
uint8_t t = 0;
int16_t u = 0;

void lcd_write_digits(int16_t val, uint8_t colour)
{
	if (val < 0) u = -val;
	else u = val;
	tth = u / 10000;
	u -= tth * 10000;
	th = u / 1000;
	u -= th * 1000;
	h = u / 100;
	u -= h * 100;
	t = u / 10;
	u -= t * 10;

	if (val < 0) lcd_write_char('-', colour);

	if (tth > 0)
		lcd_write_char(tth + '0', colour);
	if (tth > 0 || th > 0)
		lcd_write_char(th + '0', colour);
	if (tth > 0 || th > 0 || h > 0)
		lcd_write_char(h + '0', colour);
	if (tth > 0 || th > 0 || h > 0 || t > 0)
		lcd_write_char(t + '0', colour);
	lcd_write_char(u + '0', colour);
}

void lcd_clear(uint8_t colour)
{
	uint16_t row, col;
	uint8_t buffer = (colour)?0xff:0;

	// Set CGRAM Location
	ssd1306_command(SSD1306_CMD_SET_LOW_COLUMN); // low col
	ssd1306_command(SSD1306_CMD_SET_HIGH_COLUMN); // hi col
	ssd1306_command(SSD1306_CMD_SET_PAGE_START); // row

	for (row = 0; row < LCD_HEIGHT / 8; ++row)
	{
		for (col = 0; col < LCD_WIDTH; ++col)
		{
			ssd1306_data(&buffer, 1);
		}
	}
}

void lcd_set_cusor(uint8_t x, uint8_t y)
{
    if ((y+CHAR_HEIGHT) >= LCD_HEIGHT) return;
    if ((x+CHAR_WIDTH) >= LCD_WIDTH) return;

	// Set CGRAM Location
	ssd1306_command(SSD1306_CMD_SET_LOW_COLUMN  | (x & 0x0f)); // low col
	ssd1306_command(SSD1306_CMD_SET_HIGH_COLUMN | ((x >> 4) & 0x0f)); // hi col
	ssd1306_command(SSD1306_CMD_SET_PAGE_START  | (y / 8)); // row
}
