#include "time.h"

volatile u32 g_ul_ms_ticks = 0;

void time_init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    // ±÷” πƒ‹
    TIM_TimeBaseStructure.TIM_Period = 99;                  //100KHz / (99+1) = 1KHz,  1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 719;              //72MHz/(719+1) = 100KHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update,ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    TIM_Cmd(TIM3, ENABLE); //ENABLE TIM3
    NVIC_Init(&NVIC_InitStructure);
}

u32 millis(void) {
    u32 ms = g_ul_ms_ticks;
    return ms;
}

void TIM3_IRQHandler(void) {
    if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET) {
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        g_ul_ms_ticks++;
    }
}
