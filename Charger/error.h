#ifndef _ERROR_H_
#define _ERROR_H_

enum
{
	ERROR_DAMAGED_CELL = 2,
	ERROR_LOW_VOLTAGE_CELL,
	ERROR_PACK_VOLTAGE,
};

/*
 * Flash the LEDs and beep out the error code.
 */
void error(uint8_t error_code);


#endif // _ERROR_H_
