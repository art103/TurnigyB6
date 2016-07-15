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

#include "stm8s_gpio.h"

/*
 * Set the Pin to output, with state.
 */
void GPIO_Output(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, uint8_t state)
{
	GPIOx->CR2 &= ~(GPIO_Pin);
    GPIOx->CR1 |= GPIO_Pin;

    if (state)
    {
        GPIOx->ODR |= GPIO_Pin;
    }
    else
    {
		GPIOx->ODR &= (uint8_t)(~(GPIO_Pin));
    }

    // Set Output mode
    GPIOx->DDR |= GPIO_Pin;
}

/*
 * Set the Pin to floating input.
 */
void GPIO_Input(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin)
{
	GPIOx->CR2 &= ~(GPIO_Pin);
    GPIOx->DDR &= ~(GPIO_Pin);
    GPIOx->CR1 &= ~(GPIO_Pin);
}

/*
 * Set the Pin to high state.
 */
void GPIO_WriteHigh(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins)
{
    GPIOx->ODR |= PortPins;
}

/*
 * Set the Pin to low state.
 */
void GPIO_WriteLow(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins)
{
    GPIOx->ODR &= 0xFF ^ PortPins;
}
