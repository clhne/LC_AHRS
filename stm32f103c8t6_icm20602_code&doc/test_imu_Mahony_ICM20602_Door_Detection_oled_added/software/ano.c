#include "ano.h"
#include "stdarg.h"
#include "stdlib.h"
#include "usart.h"

#define ANO_SetName(_name)	do{ano_txbuffer[2] = (_name);ano_txindex = 4;}while(0)
#define ANO_Put8(_dat)		do{ano_txbuffer[ano_txindex++] = (_dat);}while(0)
#define ANO_Put16(_dat)		do{	ano_txbuffer[ano_txindex++] = BYTE1((_dat));	ano_txbuffer[ano_txindex++] = BYTE0((_dat));}while(0)
#define ANO_Put32(_dat)		do{	ano_txbuffer[ano_txindex++] = BYTE3((_dat));	\
								ano_txbuffer[ano_txindex++] = BYTE2((_dat));	\
								ano_txbuffer[ano_txindex++] = BYTE1((_dat));	\
								ano_txbuffer[ano_txindex++] = BYTE0((_dat));}while(0)

static uint8_t ano_txbuffer[50]={0xAA, 0xAA};	//发送数据缓存
static uint8_t ano_rxbuffer[50] = {0xAA, 0xAA};	//接收数据缓存
static uint8_t ano_txindex=4;

void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length) {
	myputbuf(ano_txbuffer,length);
}

//数据上传
void ANO_Upload(void) {
  uint8_t i;
	uint8_t check=0;
	ano_txbuffer[3] = (ano_txindex - 4);
	for(i=0; i < ano_txindex; i++) {
		check += ano_txbuffer[i];
	}
	ano_txbuffer[ano_txindex++] = check;
	ANO_DT_Send_Data(ano_txbuffer, ano_txindex);
}

//传感器数据
void ANO_DT_Send_Sensor(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z)
{
	ANO_SetName(0x02);
	ANO_Put16(a_x);ANO_Put16(a_y);ANO_Put16(a_z);
	ANO_Put16(g_x);ANO_Put16(g_y);ANO_Put16(g_z);
	ANO_Put16(m_x);ANO_Put16(m_y);ANO_Put16(m_z);	
	
	ANO_Upload();
}

void ANO_RecvData_CallBack(uint8_t cmd,uint8_t *buffer,uint16_t len) {
}


//extern void ANO_RecvData_CallBack(uint8_t cmd, uint8_t *buf, uint8_t len);
void ANO_RecvByte(uint8_t dat) {
	static uint8_t step = 0;
	static uint8_t cmd;
	static uint8_t len;
	static uint8_t counter;
	static uint8_t check;
	switch(step)
	{
	case 0:
		if (dat == 0xAA)
		{
			step++;
			check = dat;
		}
		break;
	case 1:
		if (dat == 0xAF)
		{
			check  += dat;
			step++;
		}
		else step = 0;
		break;
	case 2:
		cmd = dat;
		check += dat;
		step++;
		break;
	case 3:
		len = dat;
		check += dat;
		if (len == 0)step += 2;
		else
		{
			counter = 0;
			step++;
		}
		break;
	case 4:
		ano_rxbuffer[counter] = dat;
		check += dat;
		if (++counter==len)
		{
			step++;
		}
		break;
	case 5:
		if (check ==dat)
		{
			ANO_RecvData_CallBack(cmd,ano_rxbuffer,len);
		}
		step = 0;
		break;
	}
}

void ANO_RecvBytes(uint8_t *dat,uint16_t len){
	uint16_t  i;
	for (i = 0; i < len ; i++) {
		ANO_RecvByte(*dat++);
	}
}




