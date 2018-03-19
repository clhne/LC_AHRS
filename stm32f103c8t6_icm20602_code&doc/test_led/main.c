#include "stm32f10x.h"

#define CLOCK 72/8 // 72M

void delay_us(unsigned int us) {
	u8 n;		    
	while(us--)for(n=0;n<CLOCK;n++); 	 
}

void delay_ms(unsigned int ms) {
	while(ms--)delay_us(1000);	 
}

int main(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	SystemInit();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	while(1) {
		delay_ms(1000);
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		delay_ms(1000);
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
	}
}
