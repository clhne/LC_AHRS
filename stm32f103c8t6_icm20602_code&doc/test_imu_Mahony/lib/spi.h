#ifndef _SPI_H_
#define _SPI_H_
#include "sys.h"
#define CS_ICM	PAout(8)
#define CS_AK	PBout(12)
#define CS_SPL	PAout(11)
void spi2_init(void);
uint8_t spi2_write_reg(uint8_t reg_addr,uint8_t reg_val);
uint8_t spi2_read_reg(uint8_t reg_addr);
uint8_t spi2_read_reg_buffer(uint8_t reg_addr,void *buffer,uint16_t len);
#endif
