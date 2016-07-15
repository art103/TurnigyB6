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

#ifndef _ERROR_H_
#define _ERROR_H_

/*
 * Error codes.
 * These will be beeped out when there is an error.
 */
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
