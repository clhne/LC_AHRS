#include "ak8975.h"
#include "delay.h"
#include "spi.h"

/*
*测量时间:典型7.9ms，最大10ms
单位转换：最小0.285 典型0.3 最大0.315 μT/LSB
*/

#define AK8975_WIA     0x00		//Device ID = 0x48

#define AK8975_HXL     0x03
#define AK8975_HXH     0x04
#define AK8975_HYL     0x05
#define AK8975_HYH     0x06
#define AK8975_HZL     0x07
#define AK8975_HZH     0x08

#define AK8975_CNTL    0x0A


#define AK8975_INFO     0x01
#define AK8975_ST1     	0x02
#define AK8975_ST2     	0x09
#define AK8975_ASTC     0x0C
//#define AK8975_TS1     	0x0D
//#define AK8975_TS2     	0x0E
#define AK8975_I2CDIS   0x0F
#define AK8975_ASAX     0x10
#define AK8975_ASAY     0x11
#define AK8975_ASAZ     0x12

#define AK8975_ADDRESS	0x18

uint8_t ak8975_write_reg(uint8_t reg,uint8_t val) {
  CS_AK = 0;
  spi2_write_reg(reg,val);
  CS_AK =1;
  return 0;
}

uint8_t ak8975_read_reg(uint8_t reg) {
  uint8_t res;
  CS_AK = 0;
  res = spi2_read_reg(reg);
  CS_AK =1;
  return res;
}

uint8_t ak8975_read_buffer(uint8_t reg,void *buffer,uint8_t len) {
  CS_AK = 0;
  spi2_read_reg_buffer(reg,buffer,len);
  CS_AK = 1;
  return 0;
}

uint8_t ak8975_init() {
  uint8_t id,info;
	id = ak8975_read_reg(AK8975_WIA);
	info = ak8975_read_reg(AK8975_INFO);

  printf("ak8975 id:%x\r\n",id);
  printf("ak8975 info:0x%x\r\n",info);

  //ak8975_write_reg(AK8975_I2CDIS,0x1B);	//禁止IIC
  if (ak8975_start()) {	//启动一次测量
    printf("error");
    return 1;
  }
  printf("ak8975 init pass\r\n\r\n");
	return 0;
}

uint8_t ak8975_start() {
  return ak8975_write_reg(AK8975_CNTL,AK8975_MODE_SignalMeasure);
}

uint8_t ak8975_get_mag_adc(int16_t *mag) {
  uint8_t buf[6];
  if (ak8975_read_buffer(AK8975_HXL,buf,6))return 1;
  mag[0] = (buf[1]<<8) + buf[0];
  mag[1] = (buf[3]<<8) + buf[2];
  mag[2] = (buf[5]<<8) + buf[4];
  return 0;
}

float _mag_scale = 0.3f;
uint8_t ak8975_get_mag(float *mag) {
  int16_t magadc[3];
  if (ak8975_get_mag_adc(magadc))return 1;
  mag[0] = _mag_scale * magadc[0];
  mag[1] = _mag_scale * magadc[1];
  mag[2] = _mag_scale * magadc[2];
	return 0;
}
