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

#ifndef _BALANCER_H_
#define _BALANCER_H_

/*
 * Initialise the balancer GPIOs
 */
void balancer_init(void);

/*
 * channels: bitmask of channel values (1 = on, 0 = off)
 * mask: bitmask of channels to set values for
 */
void balancer_set(uint8_t channels, uint8_t mask);

/*
 * Turn off the balancer
 */
void balancer_off(void);

/*
 * Called every 5s.
 * Each cell ADC step ~= 10mV
 */
void balance_pack(void);


#endif // _BALANCER_H_
