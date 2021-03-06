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

    // TB6612端口配置AIN2->PB12, AIN1->PB13, BIN1->PB14, BIN1->PB15
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // TB6612 PWM端口配置 PWMA->PA11, PWMB->PA8
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // 使能GPIO外设时钟使能
    // 设置该引脚为复用输出功能, 输出TIM1 CH1 CH4的PWM脉冲波形
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11; // TIM_CH1 TIM_CH4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = arr; // 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; // 设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // 设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // TIM向上计数模式
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM1, &TIM_OCInitStructure); // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx
    TIM_OC4Init(TIM1, &TIM_OCInitStructure); // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx

    TIM_CtrlPWMOutputs(TIM1, ENABLE); // MOE主输出使能

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1预装载使能
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4预装载使能

    TIM_ARRPreloadConfig(TIM1, ENABLE); // 使能TIMx在ARR上的预装载寄存器

    TIM_Cmd(TIM1, ENABLE);  // 使能TIM1
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
函数功能：限制PWM的值并赋值给PWM寄存器
入口参数：PWM
返回  值：无
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
