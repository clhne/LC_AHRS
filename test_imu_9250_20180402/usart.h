#ifndef _USART_H
#define _USART_H
#include "sys.h"

#define USART_REC_LEN 200 //定义最大接收字节数 200

extern u8 USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA; //接收状态标记

void uart_init(unsigned int baud);
void uart_send_char(char c);
void uart_send_buffer(unsigned char *buffer, unsigned short len);
#endif

