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

#ifndef _SSD1306_H_
#define _SSD1306_H_

#define SSD1306_CMD_SET_LOW_COLUMN  		0x00
#define SSD1306_CMD_SET_HIGH_COLUMN 		0x10
#define SSD1306_CMD_SET_MEMORY_MODE 		0x20
#define SSD1306_CMD_SET_COLUMN      		0x21        // Triple byte
#define SSD1306_CMD_SET_PAGE        		0x22
#define SSD1306_CMD_SET_FADE_BLINK  		0x23

#define SSD1306_CMD_HSCROLL_R       		0x26
#define SSD1306_CMD_HSCROLL_L       		0x27
#define SSD1306_CMD_VSCROLL_HSCROLL_R 		0x29
#define SSD1306_CMD_VSCROLL_HSCROLL_L 		0x2A
#define SSD1306_CMD_STOP_SCROLL     		0x2E
#define SSD1306_CMD_START_SCROLL    		0x2F

#define SSD1306_CMD_SET_DISPLAY_START   	0x40

#define SSD1306_CMD_SET_CONTRAST_BANK0  	0x81    // Double byte

#define SSD1306_CMD_CHARGEPUMP          	0x8D

#define SSD1306_CMD_SET_SEG_NO_REMAP    	0xA0
#define SSD1306_CMD_SET_SEG_REMAP   		0xA1

#define SSD1306_CMD_SET_VSCROLL_AREA 		0xA3
#define SSD1306_CMD_ENTIRE_DISPLAY_RESUME   0xA4
#define SSD1306_CMD_ENTIRE_DISPLAY_ON       0xA5
#define SSD1306_CMD_SET_NORMAL      		0xA6
#define SSD1306_CMD_SET_INVERSE     		0xA7
#define SSD1306_CMD_SET_MULTIPLEX   		0xA8        // Double byte

#define SSD1306_CMD_SET_DISPLAY_OFF 		0xAE
#define SSD1306_CMD_SET_DISPLAY_ON  		0xAF

#define SSD1306_CMD_SET_PAGE_START  		0xB0

#define SSD1306_CMD_SET_COM_SCAN_INC    	0xC0
#define SSD1306_CMD_SET_COM_SCAN_DEC    	0xC8

#define SSD1306_CMD_SET_DISPLAY_OFFSET  	0xD3    // Double byte

#define SSD1306_CMD_SET_CLOCK_DIVIDER   	0xD5
#define SSD1306_CMD_SET_ZOOM_IN     		0xD6        // Double byte

#define SSD1306_CMD_SET_PRECHARGE   		0xD9        // Double byte
#define SSD1306_CMD_SET_COM_PINS    		0xDA        // Double Byte
#define SSD1306_CMD_SET_VCOM_LEVEL  		0xDB        // Double Byte

#define SSD1306_CMD_NOP             		0xE3

#define SSD1306_EXTERNALVCC 				0x1
#define SSD1306_SWITCHCAPVCC 				0x2

void i2c_isr(void);

int lcd_init(void);
void lcd_write_char(uint8_t c, uint8_t colour);
void lcd_write_string(char *s, uint8_t colour);
void lcd_write_digits(int16_t val, uint8_t thdp, uint8_t colour);
void lcd_clear(uint8_t colour);
void lcd_set_cusor(uint8_t x, uint8_t y);
void lcd_update(void);

#endif
