#ifndef _GPIO_H_
#define _GPIO_H_
#include "sys.h"
void gpio_mode_in(GPIO_TypeDef* GPIOx,uint16_t pin);
void gpio_mode_out(GPIO_TypeDef* GPIOx,uint16_t pin);
void gpio_mode_af(GPIO_TypeDef* GPIOx,uint16_t pin);
#endif



