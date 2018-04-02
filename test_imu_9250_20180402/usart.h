#ifndef _USART_H
#define _USART_H
#include "sys.h"

#define USART_REC_LEN 200 //�����������ֽ��� 200

extern u8 USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA; //����״̬���

void uart_init(unsigned int baud);
void uart_send_char(char c);
void uart_send_buffer(unsigned char *buffer, unsigned short len);
#endif

