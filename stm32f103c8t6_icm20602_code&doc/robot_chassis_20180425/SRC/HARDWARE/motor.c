#include <math.h>
#include "motor.h"

#define LEFT_PWM  TIM1->CCR4 // PA11
#define LEFT_IN1 PBout(13)
#define LEFT_IN2 PBout(12)
#define RIGHT_PWM TIM1->CCR1  // PA8
#define RIGHT_IN1 PBout(14)
#define RIGHT_IN2 PBout(15)

void motor_init(u16 arr, u16 psc) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // TB6612�˿�����AIN2->PB12, AIN1->PB13, BIN1->PB14, BIN1->PB15
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // TB6612 PWM�˿����� PWMA->PA11, PWMB->PA8
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // ʹ��GPIO����ʱ��ʹ��
    // ���ø�����Ϊ�����������, ���TIM1 CH1 CH4��PWM���岨��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11; // TIM_CH1 TIM_CH4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = arr; // ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = psc; // ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // ����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); // ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ѡ��ʱ��ģʽ:TIM������ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // �������:TIM����Ƚϼ��Ը�
    TIM_OC1Init(TIM1, &TIM_OCInitStructure); // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
    TIM_OC4Init(TIM1, &TIM_OCInitStructure); // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

    TIM_CtrlPWMOutputs(TIM1, ENABLE); // MOE�����ʹ��

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��

    TIM_ARRPreloadConfig(TIM1, ENABLE); // ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���

    TIM_Cmd(TIM1, ENABLE);  // ʹ��TIM1
}

u16 ABS(int v) {
    int t;
    if (v < 0) {
        t = -v;
    } else {
        t = v;
    }
    return t;
}

/**************************************************************************
�������ܣ�����PWM��ֵ����ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void motor_set_pwm(int *left_pwm_ptr, int *right_pwm_ptr) {
    int left_pwm = *left_pwm_ptr;
    int right_pwm = *right_pwm_ptr;
    if (left_pwm < -6900) {
        left_pwm = -6900;
    }
    if (left_pwm > 6900) {
        left_pwm = 6900;
    }
    if (right_pwm < -6900) {
        right_pwm = -6900;
    }
    if (right_pwm > 6900) {
        right_pwm = 6900;
    }
    if (left_pwm < 0) {
        LEFT_IN1 = 1;
        LEFT_IN2 = 0;
    } else {
        LEFT_IN1 = 0;
        LEFT_IN2 = 1;
    }
    LEFT_PWM = ABS(left_pwm);
    if (right_pwm < 0) {
        RIGHT_IN1 = 0;
        RIGHT_IN2 = 1;
    } else {
        RIGHT_IN1 = 1;
        RIGHT_IN2 = 0;
    }
    RIGHT_PWM = ABS(right_pwm);
    *left_pwm_ptr = left_pwm;
    *right_pwm_ptr = right_pwm;
}