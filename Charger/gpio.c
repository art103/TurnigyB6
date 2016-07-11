#include "stm8s_gpio.h"

/*
 * Set the Pin to output, with state.
 */
void GPIO_Output(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, uint8_t state)
{
	GPIOx->CR2 &= ~(GPIO_Pin);

	if (state)
	{
		GPIOx->ODR |= GPIO_Pin;
	}
	else /* Low level */
	{
		GPIOx->ODR &= ~(GPIO_Pin);
	}
	
    /* Set Output mode */
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
	GPIOx->ODR &= ~PortPins;
}
