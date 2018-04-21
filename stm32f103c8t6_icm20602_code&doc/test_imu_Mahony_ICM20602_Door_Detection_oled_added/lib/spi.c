#include "spi.h"
#include "gpio.h"

void spi2_init() {
  SPI_InitTypeDef SPI_InitStruct;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
  gpio_mode_af(GPIOB,GPIO_Pin_15);//MOSI
  gpio_mode_af(GPIOB,GPIO_Pin_14);//MISO
  gpio_mode_af(GPIOB,GPIO_Pin_13);//SCK
	
	//CS
  gpio_mode_out(GPIOB,GPIO_Pin_12); CS_AK = 1;
  gpio_mode_out(GPIOA,GPIO_Pin_8); CS_ICM = 1;
  gpio_mode_out(GPIOA,GPIO_Pin_11);	CS_SPL = 1;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CRCPolynomial = 7;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_Init(SPI2,&SPI_InitStruct);
  SPI_Cmd(SPI2, ENABLE);
}

uint8_t spi2_read_write_byte(uint8_t txc) {
  while((SPI2->SR&SPI_SR_TXE)==0);
  SPI2->DR = txc;
  while((SPI2->SR&SPI_SR_RXNE)==0);
  return SPI2->DR;	
}

uint8_t spi2_write_reg(uint8_t reg_addr,uint8_t reg_val) {
  spi2_read_write_byte(reg_addr&0x7f);
  spi2_read_write_byte(reg_val);
  return 0;
}

uint8_t spi2_read_reg(uint8_t reg_addr) {
  spi2_read_write_byte(reg_addr|0x80);
  return spi2_read_write_byte(0xff);
}

uint8_t spi2_read_reg_buffer(uint8_t reg_addr,void *buffer,uint16_t len) {
  uint8_t *p = buffer;
  uint16_t i;
  spi2_read_write_byte(reg_addr|0x80);
  for(i=0;i<len;i++) {
    *p++= spi2_read_write_byte(0xff);
  }
  return 0;
}
