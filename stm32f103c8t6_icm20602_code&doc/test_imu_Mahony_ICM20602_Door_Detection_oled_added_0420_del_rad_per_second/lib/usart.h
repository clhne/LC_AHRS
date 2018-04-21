#ifndef _USART_H
#define _USART_H

#include "sys.h"



void usart1_init(uint32_t baud);
void myputc(uint8_t c);
void myputbuf(uint8_t *buffer, uint16_t len);
void myputs(uint8_t *s);


#endif

