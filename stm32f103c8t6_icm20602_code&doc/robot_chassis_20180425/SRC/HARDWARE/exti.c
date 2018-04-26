#include "exti.h"
/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void exti_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // �ⲿ�жϣ���Ҫʹ��AFIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // ʹ��PB�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
    GPIO_Init(GPIOB, &GPIO_InitStructure); // �����趨������ʼ��GPIOB
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // �½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure); // ����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; // ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	 // �����ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
}


