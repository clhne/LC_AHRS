#include "sys.h"

__asm void MSR_MSP(u32 addr) {
  MSR MSP, r0 			//set Main Stack value
  BX r14
}

//进入待机模式	  
void Sys_Standby(void) {
  SCB->SCR|=1<<2;//使能SLEEPDEEP位 (SYS->CTRL)	   
  RCC->APB1ENR|=1<<28;     //使能电源时钟	    
  PWR->CSR|=1<<8;          //设置WKUP用于唤醒
  PWR->CR|=1<<2;           //清除Wake-up 标志
  PWR->CR|=1<<1;           //PDDS置位		
  __wfi();	 
}

//系统软复位   
void Sys_Soft_Reset(void) {   
  SCB->AIRCR =0X05FA0000|(uint32_t)0x04;	  
}

//JTAG模式设置,用于设置JTAG的模式
//mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;	   
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00		  
void JTAG_Set(uint8_t mode) {
  uint32_t temp;
  temp=mode;
  temp<<=25;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用IO时钟	   
  AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
  AFIO->MAPR|=temp;       //设置jtag模式
}

