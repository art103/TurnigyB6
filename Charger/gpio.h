#include "stm8s.h"

#ifndef _GPIO_H_
#define _GPIO_H_

/*
 * Set the Pin to output, with state.
 */
void GPIO_Output(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, uint8_t state);

/*
 * Set the Pin to floating input.
 */
void GPIO_Input(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin);

#endif // _GPIO_H_
