#ifndef _ERROR_H_
#define _ERROR_H_

enum
{
	ERROR_DONE = 2,
	ERROR_INPUT_VOLTAGE,
	ERROR_PACK_VOLTAGE,
	ERROR_TIMEOUT,
	ERROR_LOW_VOLTAGE_CELL,
	ERROR_HIGH_VOLTAGE_CELL,
};

/*
 * Flash the LEDs and beep out the error code.
 */
void error(uint8_t error_code);


#endif // _ERROR_H_
