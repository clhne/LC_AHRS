#include "spl06.h"
#include "delay.h"
#include "spi.h"

#define SP06_PSR_B2		0x00		//气压值
#define SP06_PSR_B1		0x01
#define SP06_PSR_B0		0x02
#define SP06_TMP_B2		0x03		//温度值
#define SP06_TMP_B1		0x04
#define SP06_TMP_B0		0x05

#define SP06_PSR_CFG	0x06		//气压测量配置
#define SP06_TMP_CFG	0x07		//温度测量配置
#define SP06_MEAS_CFG	0x08		//测量模式配置

#define SP06_CFG_REG	0x09
#define SP06_INT_STS	0x0A
#define SP06_FIFO_STS	0x0B

#define SP06_RESET		0x0C
#define SP06_ID			0x0D

#define SP06_COEF		0x10		//-0x21,12个字节
#define SP06_COEF_SRCE	0x28

#define SPL06_ADDRESS	0xEE

/*
SP06_MEAS_CFG：
SENSOR_RDY:		12ms
MEAS_COEF_RDY:	40ms

温度转换速率:	1Hz-128Hz

气压转换速率: 1 -128 Hz
气压转换时间
	低功耗:	5ms
	标准:	28ms
	高精度:	105ms
*/

uint8_t spl06_write_reg(uint8_t reg,uint8_t val) {
  //return myiic_write_reg(SPL06_ADDRESS,reg,val);
  CS_SPL = 0;
  spi2_write_reg(reg,val);
  CS_SPL =1;
  return 0;
}

uint8_t spl06_read_reg(uint8_t reg) {
  //return myiic_read_reg(SPL06_ADDRESS,reg);
  uint8_t res;
  CS_SPL = 0;
  res = spi2_read_reg(reg);
  CS_SPL =1;
  return res;
}

uint8_t spl06_read_buffer(uint8_t reg,void *buffer,uint8_t len) {
  //return myiic_read_reg_buffer(SPL06_ADDRESS,reg,buffer,len);
  CS_SPL = 0;
  spi2_read_reg_buffer(reg,buffer,len);
  CS_SPL = 1;
  return 0;
}

int16_t _C0,_C1,_C01,_C11,_C20,_C21,_C30;
int32_t _C00,_C10;

float _kT,_kP;

uint8_t spl06_init(void) {
  uint8_t coef[18];
  if(spl06_write_reg(SP06_RESET,0x89)) {
    puts("spl06 reset  fail\r\n");
    return 1;
  }

  printf("sol06 id=%x\r\n",spl06_read_reg(SP06_ID));//读取ID

  delay_ms(200);		//复位后系数准备好需要至少40ms
  spl06_read_buffer(SP06_COEF,coef,18);
  _C0 = ((int16_t)coef[0]<<4 ) + ((coef[1]&0xF0)>>4);
  _C0 = (_C0&0x0800)?(0xF000|_C0):_C0;
  _C1 = ((int16_t)(coef[1]&0x0F)<<8 ) + coef[2];
  _C1 = (_C1&0x0800)?(0xF000|_C1):_C1;

  _C00 = ((int32_t)coef[3]<<12 ) + ((uint32_t)coef[4]<<4 ) + (coef[5]>>4);
  _C10 = ((int32_t)(coef[5]&0x0F)<<16 ) + ((uint32_t)coef[6]<<8 ) + coef[7];
  _C00 = (_C00&0x080000)?(0xFFF00000|_C00):_C00;
  _C10 = (_C10&0x080000)?(0xFFF00000|_C10):_C10;

  _C01 = ((int16_t)coef[8]<<8 ) + coef[9];

  _C11 = ((int16_t)coef[10]<<8 ) + coef[11];
  _C11 = (_C11&0x0800)?(0xF000|_C11):_C11;
  _C20 = ((int16_t)coef[12]<<8 ) + coef[13];
  _C20 = (_C20&0x0800)?(0xF000|_C20):_C20;
  _C21 = ((int16_t)coef[14]<<8 ) + coef[15];
  _C21 = (_C21&0x0800)?(0xF000|_C21):_C21;
  _C30 = ((int16_t)coef[16]<<8 ) + coef[17];
  _C30 = (_C30&0x0800)?(0xF000|_C30):_C30;
 
  spl06_config_pressure(PM_RATE_1,PM_PRC_64);
  spl06_config_temperature(TMP_RATE_1,TMP_PRC_64);
 
  printf("_C0=%d\r\n",_C0);
  printf("_C1=%d\r\n",_C1);
  printf("_C00=%d\r\n",_C00);
  printf("_C10=%d\r\n",_C10);
  printf("_C01=%d\r\n",_C01);
  printf("_C11=%d\r\n",_C11);
  printf("_C20=%d\r\n",_C20);
  printf("_C21=%d\r\n",_C21);
  printf("_C30=%d\r\n",_C30);

  puts("spl06 init pass\r\n\r\n");

  spl06_start_T();	//启动温度测量

  return 0;
}

void spl06_config_temperature(uint8_t rate,uint8_t oversampling) {
  switch(oversampling) {
  case TMP_PRC_1:
    _kT = 524288;
    break;
  case TMP_PRC_2:
    _kT = 1572864;
    break;
  case TMP_PRC_4:
    _kT = 3670016;
    break;
  case TMP_PRC_8:
    _kT = 7864320;
    break;
  case TMP_PRC_16:
    _kT = 253952;
    break;
  case TMP_PRC_32:
    _kT = 516096;
    break;
  case TMP_PRC_64:
    _kT = 1040384;
    break;
  case TMP_PRC_128:
    _kT = 2088960;
    break;
	}
  _kT = 1.0f/_kT;

  spl06_write_reg(SP06_TMP_CFG,rate|oversampling|0x80);	//温度每秒128次测量一次
  if(oversampling > TMP_PRC_8) {
    uint8_t temp = spl06_read_reg(SP06_CFG_REG);
    spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_T_SHIFT);
  }
}

void spl06_config_pressure(uint8_t rate,uint8_t oversampling) {
  switch(oversampling){
  case PM_PRC_1:
    _kP = 524288;
    break;
  case PM_PRC_2:
    _kP = 1572864;
    break;
  case PM_PRC_4:
    _kP = 3670016;
    break;
  case PM_PRC_8:
    _kP = 7864320;
    break;
  case PM_PRC_16:
    _kP = 253952;
    break;
  case PM_PRC_32:
    _kP = 516096;
    break;
  case PM_PRC_64:
    _kP = 1040384;
    break;
  case PM_PRC_128:
    _kP = 2088960;
    break;
  }
  _kP = 1.0f/_kP;
  spl06_write_reg(SP06_PSR_CFG,rate|oversampling);
  if(oversampling > PM_PRC_8) {
    uint8_t temp = spl06_read_reg(SP06_CFG_REG);
    spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_P_SHIFT);
	}
}

int32_t spl06_get_pressure_adc() {
  uint8_t buf[3];
  int32_t adc;
  spl06_read_buffer(SP06_PSR_B2,buf,3);
  adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
  adc = (adc&0x800000)?(0xFF000000|adc):adc;
  return adc;
}

int32_t temp_adc;
int32_t spl06_get_temperature_adc() {
  uint8_t buf[3];
  int32_t adc;
  spl06_read_buffer(SP06_TMP_B2,buf,3);
  adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
  adc = (adc&0x800000)?(0xFF000000|adc):adc;
  temp_adc = adc;
  return adc;
}

uint8_t spl06_start_P() {
  return spl06_write_reg(SP06_MEAS_CFG,MEAS_CTRL_PressMeasure);	
}

uint8_t spl06_start_T() {
  return spl06_write_reg(SP06_MEAS_CFG,MEAS_CTRL_TempMeasure);	
}

float spl06_get_pressure() {
  int32_t pressadc;
  float Traw_src,Praw_src;
  float qua2, qua3;
  float press;
  
  pressadc = spl06_get_pressure_adc();
  Praw_src = _kP * pressadc;
  Traw_src = _kT * temp_adc;
  
  qua2 = _C10 + Praw_src * (_C20 + Praw_src* _C30);
  qua3 = Traw_src * Praw_src * (_C11 + Praw_src * _C21);
  
  press = _C00 + Praw_src * qua2 + Traw_src * _C01 + qua3;
  return press;
}

float spl06_get_temperature() {
  int32_t tempadc;
  float Traw_src,temp;

  tempadc = spl06_get_temperature_adc();
  Traw_src = _kT*tempadc;
  temp = 0.5f*_C0 + Traw_src*_C1;
  return temp;
}
