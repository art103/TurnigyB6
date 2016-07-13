#ifndef _ERROR_H_
#define _ERROR_H_

enum
{
	ERROR_LOW_VOLTAGE_CELL = 2,
	ERROR_PACK_VOLTAGE,
	ERROR_INPUT_VOLTAGE,
	ERROR_DONE,
};

/*
 * Flash the LEDs and beep out the error code.
 */
void error(uint8_t error_code);


#endif // _ERROR_H_
