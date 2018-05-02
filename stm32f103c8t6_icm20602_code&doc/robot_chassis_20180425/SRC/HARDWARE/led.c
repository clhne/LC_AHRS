#include "led.h"
#define LED PAout(12) // PA12
/**************************************************************************
�������ܣ�led�ӿڳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void led_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��PORTAʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //PA12�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_12); //PA12�����
}

/**************************************************************************
�������ܣ�led��˸
��ڲ�������˸Ƶ��
����  ֵ����
**************************************************************************/
void led_toggle(u16 freq) {
    static int counter = 0;
    if (freq == 0) {
        LED = 0;
    } else {
        if (++counter == freq) {
            LED = ~LED;
            counter = 0;
        }
    }
}