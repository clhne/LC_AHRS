#include "encoder.h"
#define ENCODER_TIM_PERIOD (u16)(65535) //���ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�
/**************************************************************************
�������ܣ���TIM2,TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void encoder_init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // encoder for left wheel
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // ʹ�ܶ�ʱ��4��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ʹ��PB�˿�ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	// �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // ��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  // �趨�������Զ���װֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ѡ��ʱ�ӷ�Ƶ������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); // ʹ�ñ�����ģʽ3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update); // ���TIM�ĸ��±�־λ
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);

    // encoder for right wheel
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // ʹ�ܶ�ʱ��4��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

/**************************************************************************
�������ܣ�TIM4�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM4_IRQHandler(void) {
    if(TIM4->SR & 0x0001) { // ����ж�
    }
    TIM4->SR &= ~(1 << 0); // ����жϱ�־λ
}

/**************************************************************************
�������ܣ�TIM2�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM2_IRQHandler(void) {
    if (TIM2->SR & 0x0001) { // ����ж�
    }
    TIM2->SR &= ~(1 << 0); // ����жϱ�־λ
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int encoder_read(u8 TIMX) {
    int counter;
    switch(TIMX) {
    case 2:
        counter = (short)TIM2->CNT;
        TIM2->CNT = 0;
        break;
    case 4:
        counter = (short)TIM4->CNT;
        TIM4->CNT = 0;
        break;
    default:
        counter = 0;
    }
    return counter;
}
