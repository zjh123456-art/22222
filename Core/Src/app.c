/*
 * app.c
 *
 *  Created on: July 6, 2020
 *      Author: aguang
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "uart.h"
//#include "modbus.h"

#include "adc_4112_cache.h"
#include "clock.h"
#include "AD5422.h"
#include "flashapp.h"
#define extern
#include "adc_4112.h"
#include "app.h"
#undef extern
#include <math.h>
//volatile uint8_t flag_modbus_cmd;				//volatile 禁止編譯器優化省略變量變化
//uint8_t rx_buffer[BUFFER_SIZE];
//uint8_t gobal_channel_num;
//uint8_t gobal_channel_mask;
//#define printf  printf(..)

#define ADC_V_REF_FLASH_PAGE	0
#define ADC_I_FLASH_PAGE	1
#define DAC_V_FLASH_PAGE	2
#define DAC_I_FLASH_PAGE	3
#define ADC_V_FLASH_PAGE	4
#define ADC_I_REF_FLASH_PAGE	5

#define FLASH_VALUE_01MA	0
#define FLASH_VALUE_20MA	1

#define BUFFER_SIZE 256 //缓冲区的长度,可以修改
#define TIMEOOT_COUNT 100
#define CAL_NUM 11 //11校准
#define ARRAY_SIZE(X) (sizeof(X) / sizeof(X[0]))
uint32_t validLen;			//已使用的数据长度
uint8_t *pHead = NULL;		//环形存储区的首地址
uint8_t *pTail = NULL;		//环形存储区的结尾地址
uint8_t *pValid = NULL;		//已使用的缓冲区的首地址
uint8_t *pValidTail = NULL; //已使用的缓冲区的尾地址
RegValDef calset[CAL_NUM];

ad717x_dev *adc_4112_dev1;
ad717x_dev *adc_4112_dev2;
uint8_t I_open, V_open;
uint16_t SendCRC16(uint8_t *vptr, uint8_t len)
{
	uint16_t TCPCRC = 0xFFFF;
	uint16_t POLYNOMIAL = 0xA001;
	uint8_t i, j;

	for (i = 0; i < len; i++)
	{
		TCPCRC ^= vptr[i];
		for (j = 0; j < 8; j++)
		{
			if ((TCPCRC & 0x0001) != 0)
			{
				TCPCRC >>= 1;
				TCPCRC ^= POLYNOMIAL;
			}
			else
			{
				TCPCRC >>= 1;
			}
		}
	}
	return TCPCRC;
}
/*
 * 初始化环形缓冲区
 * 环形缓冲区这里可以是malloc申请的内存,也可以是Flash存储介质
 */
void initRingbuffer(void)
{
	if (pHead == NULL)
	{
		pHead = (uint8_t *)malloc(BUFFER_SIZE);
	}
	pValid = pValidTail = pHead;
	pTail = pHead + BUFFER_SIZE;
	validLen = 0;
	//printf("initRingbuffe\r\n");
}

/*
 * function:向缓冲区中写入数据
 * param:@buffer 写入的数据指针
 *       @addLen 写入的数据长度
 * return:-1:写入长度过大
 *        -2:缓冲区没有初始化
 * */
int wirteRingbuffer(uint8_t *buffer, uint32_t addLen)
{
	//printf("wirteRingbuffer\r\n");
	if (addLen > BUFFER_SIZE)
		return -2;
	if (pHead == NULL)
		return -1;
	//UNUSED(buffer);
	//printf("DataLength=%d\r\n",addLen);
	//将要存入的数据copy到pValidTail处
	if (pValidTail + addLen > pTail) //需要分成两段copy
	{
		int len1 = pTail - pValidTail;
		int len2 = addLen - len1;
		memcpy(pValidTail, buffer, len1);
		memcpy(pHead, buffer + len1, len2);
		pValidTail = pHead + len2; //新的有效数据区结尾指针
	}
	else
	{
		memcpy(pValidTail, buffer, addLen);
		pValidTail += addLen; //新的有效数据区结尾指针
	}

	//需重新计算已使用区的起始位置
	if (validLen + addLen > BUFFER_SIZE)
	{
		int moveLen = validLen + addLen - BUFFER_SIZE; //有效指针将要移动的长度
		if (pValid + moveLen > pTail)				   //需要分成两段计算
		{
			int len1 = pTail - pValid;
			int len2 = moveLen - len1;
			pValid = pHead + len2;
		}
		else
		{
			pValid = pValid + moveLen;
		}
		validLen = BUFFER_SIZE;
	}
	else
	{
		validLen += addLen;
	}

	return 0;
}

/*
 * function:从缓冲区内取出数据
 * param   :@buffer:接受读取数据的buffer
 *          @len:将要读取的数据的长度
 * return  :-1:没有初始化
 *          >0:实际读取的长度
 * */
int readRingbuffer(uint8_t *buffer, uint32_t len)
{
	if (pHead == NULL)
		return -1;

	// UNUSED(buffer);

	if (validLen == 0)
		return 0;

	if (len > validLen)
		len = validLen;

	if (pValid + len > pTail) //需要分成两段copy
	{
		int len1 = pTail - pValid;
		int len2 = len - len1;
		memcpy(buffer, pValid, len1);		//第一段
		memcpy(buffer + len1, pHead, len2); //第二段，绕到整个存储区的开头
		pValid = pHead + len2;				//更新已使用缓冲区的起始
	}
	else
	{
		memcpy(buffer, pValid, len);
		pValid = pValid + len; //更新已使用缓冲区的起始
	}
	validLen -= len; //更新已使用缓冲区的长度

	return len;
}

/*
 * function:获取已使用缓冲区的长度
 * return  :已使用的buffer长度
 * */
uint32_t getRingbufferValidLen(void)
{
	return validLen;
}

/*
 * function:释放环形缓冲区
 * */
void releaseRingbuffer(void)
{
	if (pHead != NULL)
		free(pHead);
	pHead = NULL;
}

int Datasend(SendDataStruct *DataStruct)
{
	unsigned char count = 0, loop;
	unsigned char UartSendBuffer[32];
	char asciidata[10];
	unsigned char *data = (unsigned char *)DataStruct->data;
	//	printf("\r\n**send in**********");
	sprintf(asciidata, "%02x", DataStruct->cmd);
	UartSendBuffer[count++] = asciidata[0];
	UartSendBuffer[count++] = asciidata[1];
	//sprintf(asciidata, "%lf", DataStruct->Regdata.reg[]);
	sprintf(asciidata, "%02x", DataStruct->len);
	UartSendBuffer[count++] = asciidata[0];
	UartSendBuffer[count++] = asciidata[1];
	for (loop = 0; loop < DataStruct->len; loop++)
	{
		sprintf(asciidata, "%02x", *(data + loop));
		UartSendBuffer[count++] = asciidata[0];
		UartSendBuffer[count++] = asciidata[1];
	}
	DataStruct->crc = SendCRC16(UartSendBuffer, count);
	sprintf(asciidata, "%02x", (DataStruct->crc >> 8) & 0xff);
	UartSendBuffer[count++] = asciidata[0];
	UartSendBuffer[count++] = asciidata[1];
	sprintf(asciidata, "%02x", DataStruct->crc & 0xff);
	UartSendBuffer[count++] = asciidata[0];
	UartSendBuffer[count++] = asciidata[1];
	UartSendBuffer[count++] = '\r';
	UartSendBuffer[count++] = '\n';
	HAL_StatusTypeDef status = USART1_Data_Send(UartSendBuffer, count);
	if (status != HAL_OK)
		return -1;
	//if(iStatus!=count)
	//  close(SOCK_TCPS);

	return 0;
}

unsigned char uart_data_process(void)
{
	int count = 0, Hex, ret;
	unsigned char *tmp, *endpoint, *startdatapoint, i;
	static unsigned char strdata[1024], data[32];
	unsigned int DataLength;
	CmdNumCount = 0;
	DataLength = getRingbufferValidLen();
	if (DataLength <= 0)
		return 0;
	ret = readRingbuffer(data, 31);
	printf("DataLength=%d\r\n", ret);
	data[ret] = 0;
	strncat(strdata, data, ret);
	tmp = strdata;
	startdatapoint = strdata;
	endpoint = startdatapoint;
	//if(DataParserFlag==0)
	//	return CmdNumCount;

	while (*startdatapoint)
	{ //printf("startdatapoint=%x\n",*startdatapoint);
		if ((*(startdatapoint) == '\r') && (*(startdatapoint + 1) == '\n'))
		{
			if ((CmdNumCount < CMDLISTNUM))
			{
				for (i = 0; i < startdatapoint - endpoint; i += 2)
				{
					Hex = 0;
					sscanf((char *)(endpoint + i), "%02x", &Hex);
					CmdRecvBuffer[CmdNumCount][count] = Hex;
					count++;
				}
			}
			printf("CmdRecvBuffer[CmdNumCount]=%x\r\n", CmdRecvBuffer[CmdNumCount][0]);
			CmdNumCount++;
			startdatapoint += 2;
			endpoint = startdatapoint;
			tmp = startdatapoint;
			count = 0;
		}
		else
			startdatapoint++;
	}
	if (endpoint != startdatapoint)
	{
		memcpy(strdata, endpoint, startdatapoint - endpoint);
	}
	memset(strdata + (startdatapoint - endpoint), 0,
		   strlen(strdata) - (startdatapoint - endpoint));
	if (CmdNumCount == 0)
	{
		memset(CmdRecvBuffer, 0x00, sizeof(CmdRecvBuffer));
	}
	printf("CmdNumCount=%d\r\n", CmdNumCount);
	return CmdNumCount;
}
double read_Voltage_val(ad717x_dev *adc_4112_dev, uint8_t chanel, uint8_t caluse)
{
	uint32_t ret_data, timeout = 0, i;
	int32_t ret;
	double readvol, diffval, destval;
	ad717x_st_reg *reg_ret;
	switchmode(0);
	if (!V_open)
		return -1;
	do
	{
		timeout++;
		ret = AD4112_ReadData_ByChannel(adc_4112_dev, &ret_data, chanel);
		if (timeout >= TIMEOOT_COUNT)
			break;
		if (ret < 0)
			continue;

		ret = AD717X_ReadRegister(adc_4112_dev, AD717X_STATUS_REG);
		if (ret < 0)
			continue;
		reg_ret = AD717X_GetReg(adc_4112_dev, AD717X_STATUS_REG);
		break;
	} while (1);
	if (timeout < TIMEOOT_COUNT)
	{
		readvol = AD4112_Code_To_Voltage(adc_4112_dev, chanel, ret_data);
		if (caluse)
		{
			i = (int8_t)(readvol) % CAL_NUM;
			//diffval=(i<1 || readvol<0.5)?calset[0].RegFloatVal:calset[i].RegFloatVal-calset[i-1].RegFloatVal;
			if (readvol != 0 && readvol < 10)
			{
				diffval = 1 + calset[i].RegFloatVal - calset[i + 1].RegFloatVal;
				//destval=diffval*(readvol+0.5-i);
				destval = diffval * (readvol - i) + (i - calset[i].RegFloatVal);
			}
			printf("user cal %f,calset[%d]=%f,destval=%f\r\n", readvol, i, calset[i].RegFloatVal, destval);
			return (readvol + destval);
		}
		else
		{
			return readvol;
		}
	}
	else
	{
		return -1;
	}
}
double read_Ampere_val(ad717x_dev *adc_4112_dev, uint8_t chanel)
{
	uint32_t ret_data, timeout = 0;
	int32_t ret;
	ad717x_st_reg *reg_ret;
	switchmode(1);
	if (!I_open)
		return -1;
	do
	{
		timeout++;
		ret = AD4112_ReadData_ByChannel(adc_4112_dev, &ret_data, chanel);
		if (timeout >= TIMEOOT_COUNT)
			break;
		if (ret < 0)
			continue;

		ret = AD717X_ReadRegister(adc_4112_dev, AD717X_STATUS_REG);
		if (ret < 0)
			continue;
		reg_ret = AD717X_GetReg(adc_4112_dev, AD717X_STATUS_REG);
		break;
	} while (1);
	if (timeout < TIMEOOT_COUNT)
	{
		return AD4112_Code_To_Ampere(adc_4112_dev, chanel, ret_data);
	}
	else
	{
		return -1;
	}
}

static void switchmode(uint8_t mode)
{
//	uint32_t value;
	ad717x_st_reg *reg_ret;
	static unsigned char modeflag = 0xff;
	if (modeflag == mode)
		return;
	else
		modeflag = mode;

	if (mode) //电流
	{
		printf("set I\r\n");
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xf) | AD717X_CHMAP_REG_AINNEG(0x8)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xe) | AD717X_CHMAP_REG_AINNEG(0x9)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP2_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xd) | AD717X_CHMAP_REG_AINNEG(0xa)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP3_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xc) | AD717X_CHMAP_REG_AINNEG(0xb)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP4_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP5_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP6_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP7_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xf) | AD717X_CHMAP_REG_AINNEG(0x8)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xe) | AD717X_CHMAP_REG_AINNEG(0x9)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP2_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(2) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP3_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(3) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP4_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP5_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP6_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP7_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		I_open = 1;
		V_open = 0;
	}
	else //电压
	{
		printf("set V\r\n");
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(0) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(1) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP2_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(2) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP3_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(3) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP4_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP5_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP6_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1, AD717X_CHMAP7_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(0) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(1) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP2_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(2) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP3_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(3) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP4_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP5_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP6_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2, AD717X_CHMAP7_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		I_open = 0;
		V_open = 1;
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  //cs
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //cs
	AD4112_remove(adc_4112_dev1);
	AD4112_remove(adc_4112_dev2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //cs
	AD4112_init(&adc_4112_dev1, 1);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //cs
	AD4112_init(&adc_4112_dev2, 2);

	AD717X_ReadRegister(adc_4112_dev1, AD717X_ID_REG);

	reg_ret = AD717X_GetReg(adc_4112_dev1, AD717X_ID_REG);
	printf("adc id 0x%x\r\n", reg_ret->value);
	AD717X_ReadRegister(adc_4112_dev2, AD717X_ID_REG);

	reg_ret = AD717X_GetReg(adc_4112_dev2, AD717X_ID_REG);
	printf("adc id 0x%x\r\n", reg_ret->value);
}

void ADC_V_REF_Calculate(uint8_t channel)	//Flash 第1页，ADDR_FLASH_PAGE_X(0)
{
	uint16_t flash_flag;
	RegValDef tempdata;
	double readval = 0;
	uint16_t tbuffer[CAL_NUM * 2];

	printf("=========ADC_V_REF_Calculate, channel = %d\r\n", channel);
	flash_flag = 0x5c5c;
	VOLTAGE_REF = 2.5000;
	while (1)
	{
		if (channel < 8)
			readval = read_Voltage_val(adc_4112_dev1, channel, 0);
		else
			readval = read_Voltage_val(adc_4112_dev2, (channel - 8), 0);

		if (fabs(readval - 2.5) <= 0.00001)
			break;

		if (readval > 2.5)
			VOLTAGE_REF -= 0.00001;
		else
			VOLTAGE_REF += 0.00001;
	}
	printf("cal:VOLTAGE_REF=%f\r\n", VOLTAGE_REF);
	tempdata.RegFloatVal = VOLTAGE_REF;
	memcpy(&tbuffer[0], tempdata.reg, 4);

	FLASH_Write(ADDR_FLASH_PAGE_X(ADC_V_REF_FLASH_PAGE) + (channel*6), &flash_flag, 1);
	FLASH_Write(ADDR_FLASH_PAGE_X(ADC_V_REF_FLASH_PAGE) + (channel*6) + 2, tbuffer, 2);
}

void ADC_I_REF_Calculate(uint8_t channel)	//Flash 第6页，ADDR_FLASH_PAGE_X(5)
{
	uint16_t flash_flag;
	RegValDef tempdata;
	double readval = 0;
	uint16_t tbuffer[CAL_NUM * 2];

	printf("=========ADC_I_REF_Calculate, channel = %d\r\n", channel);
	flash_flag = 0x5c5c;
	VOLTAGE_REF = 2.5000;
	while (1)
	{
		if (channel < 8)
			readval = read_Voltage_val(adc_4112_dev1, channel, 0);
		else
			readval = read_Voltage_val(adc_4112_dev2, (channel - 8), 0);

		if (fabs(readval - 2.5) <= 0.00001)
			break;

		if (readval > 2.5)
			VOLTAGE_REF -= 0.00001;
		else
			VOLTAGE_REF += 0.00001;
	} 
	printf("cal:VOLTAGE_REF=%f\r\n",VOLTAGE_REF);
	tempdata.RegFloatVal = VOLTAGE_REF;
	memcpy(&tbuffer[0], tempdata.reg, 4);

	FLASH_Write(ADDR_FLASH_PAGE_X(ADC_I_REF_FLASH_PAGE) + (channel*6), &flash_flag, 1);
	FLASH_Write(ADDR_FLASH_PAGE_X(ADC_I_REF_FLASH_PAGE) + (channel*6) + 2, tbuffer, 2);
}

void ADC_V_Calculate(uint8_t channel)	//Flash 第5页，ADDR_FLASH_PAGE_X(4)
{
	double value;
	uint16_t flash_flag;
	RegValDef tempdata;
	uint16_t tbuffer[4];
	uint8_t number;
	uint8_t idatacnt;

	idatacnt = 2;	//ADC_V校准需要写入的数据个数

	flash_flag = 0x5c5c;
	VOLTAGE_REF = 2.5000;
	//2.5V	2.500716V
	//9.5V	9.503080V
	printf(">>>>>>>>ADC_V_Calculate,%d\n", channel);
	if (channel < 8)
		value = read_Voltage_val(adc_4112_dev1, channel, 0);
	else
		value = read_Voltage_val(adc_4112_dev2, (channel - 8), 0);

	printf("Vvalue %lf V\n", value);
	if(value < 9)
	{
		number = 1;
		value = value - 2.5;
	}	
	else
	{
		number = 2;
		value = value - 9.5;
	}

	printf("write number = %d ,Vvalue %lf V\n", number, value);
	number = number - 1;
	tempdata.RegFloatVal = value;
	memcpy(&tbuffer[0], tempdata.reg, 4);
	FLASH_Write(ADDR_FLASH_PAGE_X(ADC_V_FLASH_PAGE) + (channel*idatacnt*6) + (number*6), &flash_flag, 1);
	FLASH_Write(ADDR_FLASH_PAGE_X(ADC_V_FLASH_PAGE) + (channel*idatacnt*6) + (number*6) + 2, tbuffer, 2);

	
/*  	for(i = 0; i < 20; i++)
	{
		FLASH_Read(ADDR_FLASH_PAGE_X(ADC_V_FLASH_PAGE) + (i*2), &flash_flag, 1);
		if(flash_flag != 0xffff)
		{
			printf("flash_flag = 0x%x\n", flash_flag);
			flash_flag = 0xffff;
			FLASH_Write(ADDR_FLASH_PAGE_X(ADC_V_FLASH_PAGE) + (i*2), &flash_flag, 1);
		}
	}  */
}

/*
 *	这里主要获取2个值：
 *	1.外部标准电流源设置为0.1mA时，读取当前的电流值
 *	2.外部标准电流源设置为20mA时，读取当前的电流值
 *	3.6个通道，分别写到对应的Flash地址当中,1个通道需要12字节(2+4+2+4)
 */
void ADC_I_Calculate(uint8_t channel)	//Flash 第2页，ADDR_FLASH_PAGE_X(1)
{
	double value;
	uint16_t flash_flag;
	RegValDef tempdata;
	uint16_t tbuffer[4];

	flash_flag = 0x5c5c;

	printf(">>>>>>>>ADC_I_Calculate,%d\n", channel);
	if (channel < 4)
		value = read_Ampere_val(adc_4112_dev1, channel); //0,1 2,3 四路电流					读取电流值，0,1 2,3属于芯片1；4,5属于芯片2
	else
		value = read_Ampere_val(adc_4112_dev2, (channel - 4)); //0,1 两路路电流
	
	printf("Ivalue %lf A\r\n", value);

	for(int i = 0; i < 2; i++)
	{
		if(i == 0)
			value = 0.000093;						
		else
	 		value = 0.018847;

		value = 1000*value;
		printf("Ivalue %lf mA\r\n", value);
		if(value < 10)
		{
			value = value * 1000;
			printf("Ivalue %lf uA\r\n", value);
			value = -1;
			tempdata.RegFloatVal = value;
			memcpy(&tbuffer[0], tempdata.reg, 4);
			printf("write 0.1mA value......\r\n");
			FLASH_Write(ADDR_FLASH_PAGE_X(ADC_I_FLASH_PAGE) + (channel*12), &flash_flag, 1);
			FLASH_Write(ADDR_FLASH_PAGE_X(ADC_I_FLASH_PAGE) + (channel*12) + 2, tbuffer, 2);
		}
		else
		{
			value = 20 - value;
			value = value * 5;
			value = -1;
			printf("Ivalue %lf mA\r\n", value);
			tempdata.RegFloatVal = value;
			memcpy(&tbuffer[0], tempdata.reg, 4);
			printf("write 20mA value......\r\n");
			FLASH_Write(ADDR_FLASH_PAGE_X(ADC_I_FLASH_PAGE) + (channel*12) + 6, &flash_flag, 1);
			FLASH_Write(ADDR_FLASH_PAGE_X(ADC_I_FLASH_PAGE) + (channel*12) + 8, tbuffer, 2);

		}
	}
}

/*
 *	多段：
 *	1.0V:
 *	2.0.01V-0.1V:
 *	3.0.1V-0.5V:
 *	4.0.5V-0.9V:
 *	5.0.9V-10.0V:
 *	需要写入0V、0.01V、0.1V、0.5V、0.9V、10.0V共6个值,1个值占6字节，
 */
void DAC_V_Calculate(uint8_t channel, uint8_t number, double value)	//Flash 第3页，ADDR_FLASH_PAGE_X(2)
{
	uint16_t flash_flag;
	RegValDef tempdata;
	uint16_t tbuffer[4];

	printf(">>>>>>>>DAC_V_Calculate,%d， num = %d, Vvalue %lf V\n", channel, number, value);

	flash_flag = 0x5c5c;
	switch (number)
	{
		case 1:
			break;
		case 2:
			value = 0.01 - value;
			break;
		case 3:
			value = 0.1 - value;
			break;
		case 4:
			value = 0.5 - value;
			break;
		case 5:
			value = 0.9 - value;
			break;
		case 6:
			value = 10.0 - value;
			break;
		default:
			break;
	}

	printf("Vvalue %lf V\n", value);
	number = number - 1;
	tempdata.RegFloatVal = value;
	memcpy(&tbuffer[0], tempdata.reg, 4);
	FLASH_Write(ADDR_FLASH_PAGE_X(DAC_V_FLASH_PAGE) + (channel*36) + (number*6), &flash_flag, 1);
	FLASH_Write(ADDR_FLASH_PAGE_X(DAC_V_FLASH_PAGE) + (channel*36) + (number*6) + 2, tbuffer, 2);
}

/*
 *	外部读取两个值，再通过自动校准命令输入：
 *	1.设置输出0mA电流时，读取当前的电流表电流
 *	2.设置输出20mA电流时，读取当前的电流表电流
 *	3.2个通道，分别写到对应的Flash地址当中,1个通道需要12字节(2+4+2+4)
 *	需要写入0mA、0.01mA、0.1mA、1mA、10mA、20mA共6个值,1个值占6字节，
 */
void DAC_I_Calculate(uint8_t channel, uint8_t number, double value)	//Flash 第4页，ADDR_FLASH_PAGE_X(3)
{

	uint16_t flash_flag;
	RegValDef tempdata;
	uint16_t tbuffer[4];

	//0---0.011771 mA
	//0.01---0.021988 mA
	//0.1--0.113941 mA
	//1---1.034435 mA
	//10--10.245971 mA
	//20--20.48433 mA
	printf(">>>>>>>>DAC_I_Calculate,%d， num = %d, Ivalue %lf mA\n", channel, number, value);

	flash_flag = 0x5c5c;
	switch (number)
	{
		case 1:
			break;
		case 2:
			value = value - 0.01;
			break;
		case 3:
			value = value - 0.1;
			break;
		case 4:
			value = value - 1;
			break;
		case 5:
			value = value - 10;
			break;
		case 6:
			value = value - 20;
			break;
		default:
			break;
	}

	printf("value %lf mA\n", value);
	number = number - 1;
	tempdata.RegFloatVal = value;
	memcpy(&tbuffer[0], tempdata.reg, 4);
	FLASH_Write(ADDR_FLASH_PAGE_X(DAC_I_FLASH_PAGE) + (channel*36) + (number*6), &flash_flag, 1);
	FLASH_Write(ADDR_FLASH_PAGE_X(DAC_I_FLASH_PAGE) + (channel*36) + (number*6) + 2, tbuffer, 2);
}

void DAC_FLash_Value_Clear(uint8_t flag, uint8_t channel)	//Flash 第4页，ADDR_FLASH_PAGE_X(3)
{

	uint16_t flash_flag;
	uint8_t i;

	printf(">>>>>>>>DAC_FLash_Value_Clear, flag = %d, channel = %d\n", flag, channel);

	flash_flag = 0xffff;

	if(flag == 0)
	{
		for(i = 0; i < 6; i++)
		{
			FLASH_Write(ADDR_FLASH_PAGE_X(DAC_V_FLASH_PAGE) + (channel*36) + (i*6), &flash_flag, 1);
		}
	}
	else
	{
		for(i = 0; i < 6; i++)
		{
			FLASH_Write(ADDR_FLASH_PAGE_X(DAC_I_FLASH_PAGE) + (channel*36)+ (i*6), &flash_flag, 1);
		}
		//for(i = 0; i < 2; i++)
		//{
		//	FLASH_Write(ADDR_FLASH_PAGE_X(DAC_I_FLASH_PAGE) + (channel*12)+ (i*6), &flash_flag, 1);
		//}
	}
}

double GetFlashPage_ChannelVal(uint8_t FlashPageVal, uint8_t Channel, uint8_t datacnt, uint8_t number)
{
	uint16_t flash_flag;
	RegValDef tempdata;
	uint16_t rbuffer[4];
	
	printf(">>>>>>>>GetFlashPage_ChannelVal,%d\n", Channel);
	if(number > datacnt)	//读取的数据超出写入的数据总数
		return -1;

	flash_flag = 0;
	number = number - 1;

	FLASH_Read(ADDR_FLASH_PAGE_X(FlashPageVal) + (Channel*datacnt*6) + (number*6), &flash_flag, 1);
	FLASH_Read(ADDR_FLASH_PAGE_X(FlashPageVal) + (Channel*datacnt*6) + (number*6) + 2, rbuffer, 2);
	
	if (flash_flag != 0x5c5c)
		return -1;
	
	memcpy(tempdata.reg, &rbuffer[0], 4);

	return tempdata.RegFloatVal;
}

void cmd_run(void)
{
	SendDataStruct DataStruct;
	RegValDef regdata;
	uint8_t i;
	double value, range;
	double calvalue, iFlashHead, iFlashTail;
	float calDacIvalue;
	uint8_t channel, num;


	for (i = 0; i < CmdNumCount; i++)
	{
		printf("recvcmd =%x\r\n", CmdRecvBuffer[i][0]);
		/**************************************************** 

-----data struct:{
		[0];cmd
		[1];len
		[2~len+2]:data
		[len+3~len+5]:crc
		+\r\n
	}
	CMD：
     0x30 Vol1 ........    0x35 VOL6 读取6路电压
     0x50 AM1  ......      0x55 AM6  读取6路电流
		 设置ADC
		  0x90,0x91 set V  DA1,DA2
		  0xa0,0xa1 set c  DA1,DA2
		 set  calculate
		 0xc0 
 ****************************************************/
		DataStruct.cmd = CmdRecvBuffer[i][0];
		channel = DataStruct.cmd & 0xf;
		switch (DataStruct.cmd)
		{
			case 0x30:
			case 0x31:
			case 0x32:
			case 0x33:
			case 0x34:
			case 0x35:
			case 0x36:
			case 0x37:
			case 0x38:
			case 0x39:
			case 0x3a:
			case 0x3b:
			case 0x3c:
			case 0x3d:
			case 0x3e:
			case 0x3f:
				VOLTAGE_REF = 2.355720;

				if (channel < 8)
					value = read_Voltage_val(adc_4112_dev1, channel, 0);
				else
					value = read_Voltage_val(adc_4112_dev2, (channel - 8), 0);
				
				printf("value %lfV\r\n", value);

				if (value != -1)
				{
					//ADC电电压校准
					iFlashHead = GetFlashPage_ChannelVal(ADC_V_FLASH_PAGE, channel, 2, 1);
					iFlashTail = GetFlashPage_ChannelVal(ADC_V_FLASH_PAGE, channel, 2, 2);
					printf("iFlashHead = %lf\n", iFlashHead);
					printf("iFlashTail = %lf\n", iFlashTail);
					if((iFlashHead != -1) && (iFlashTail != -1))
					{
						calvalue	= value - iFlashHead + (iFlashTail - iFlashHead)*2.5/7;
						value		= calvalue/(1 + (iFlashTail - iFlashHead)/7);
					}
					printf("value %lfV\r\n", value);

					regdata.RegFloatVal = value;
					DataStruct.len = sizeof(regdata.reg);
					//memcpy((uint8_t*)DataStruct.data,regdata.reg,DataStruct.len);
					DataStruct.data = (uint8_t *)regdata.reg;
				}
				else
				{
					DataStruct.len = 2;
					//memcpy((uint8_t*)DataStruct.data,"ER",2);
					DataStruct.data = (uint8_t *)"ER";
				}
				Datasend(&DataStruct);
				break;
			case 0x50:
			case 0x51:
			case 0x52:
			case 0x53:
			case 0x54:
			case 0x55:
				iFlashHead = GetFlashPage_ChannelVal(ADC_I_REF_FLASH_PAGE, channel, 1, 1);
				//printf("iFlashHead = %lf\n", iFlashHead);
				if(iFlashHead != -1)
				{
					VOLTAGE_REF = iFlashHead;
				}
				else
				{
					VOLTAGE_REF = 2.5000;
					//VOLTAGE_REF = 2.355720;
				}
				printf("VOLTAGE_REF=%f \r\n", VOLTAGE_REF);

				if (channel < 4)
					value = read_Ampere_val(adc_4112_dev1, channel); //0,1 2,3 四路电流
				else
					value = read_Ampere_val(adc_4112_dev2, (channel - 4)); //0,1 两路路电流

				//value = 0.017715;
				
				value = value * 1000;					//	A to mA
				printf("Ivalue %lf mA\r\n", value);
				if (value != -1)
				{
					//ADC电流校准	
					iFlashHead = GetFlashPage_ChannelVal(ADC_I_FLASH_PAGE, channel, 2, 1);
					iFlashTail = GetFlashPage_ChannelVal(ADC_I_FLASH_PAGE, channel, 2, 2);
					printf("iFlashHead = %lf\n", iFlashHead);
					printf("iFlashTail = %lf\n", iFlashTail);

					if(1/*(iFlashHead != -1) && (iFlashTail != -1)*/)	//校验2个值并写到Flash中
					{
						printf("Get_ADC_I_FlashValue Success!\n");
						if(value <= 0.1)										//0~0.1mA
						{
							calvalue = (value * iFlashTail)/iFlashHead;
						}
						else if(value >0.1 || value <=1)		//0.1~1mA
						{
							value = value + 0.003162;
						}
						else if(value >1 || value <=10)			//1~10mA
						{
							value =value + 0.023169;
						}
						else																//10~20mA
						{
							value = value + 0.235413;
							//calvalue = (value * iFlashTail)/(iFlashHead + 1);
						}
						//value	= value + calvalue;
					}
					else
					{
						printf("Get_ADC_I_FlashValue Invalid!\n");
					}				

					printf("Ivalue %lf mA\r\n", value);
					value /= 1000;			// mA to A
					
					
					regdata.RegFloatVal = value;
					DataStruct.len = sizeof(regdata.reg);
					DataStruct.data = (uint8_t *)regdata.reg;
				}
				else
				{
					DataStruct.len = 2;
					DataStruct.data = (uint8_t *)"ER";
				}
				Datasend(&DataStruct);
				break;
			case 0x90:
			case 0x91:
				//AD5422init();
				regdata.reg[0] = CmdRecvBuffer[i][2];
				regdata.reg[1] = CmdRecvBuffer[i][3];
				regdata.reg[2] = CmdRecvBuffer[i][4];
				regdata.reg[3] = CmdRecvBuffer[i][5];
				printf("set V =%f\r\n", regdata.RegFloatVal);
#if 1
				if(regdata.RegFloatVal <= 0)
				{
					printf("set 0.......\n");
					iFlashHead = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 1);
					if(iFlashHead != -1)
					{
						calDacIvalue = regdata.RegFloatVal - iFlashHead;
						regdata.RegFloatVal = calDacIvalue;
					}
				}
				else if(regdata.RegFloatVal <= 0.1)
				{
					printf("set 0~0.1.......\n");	
					iFlashHead = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 2);
					iFlashTail = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 3);
					printf("iFlashHead = %lf\n", iFlashHead);
					printf("iFlashTail = %lf\n", iFlashTail);
					if((iFlashHead != -1) && (iFlashTail != -1))
					{
						//calDacIvalue = (regdata.RegFloatVal + iFlashHead)/(1 + iFlashTail - iFlashHead);
						calDacIvalue = iFlashHead + ((iFlashTail - iFlashHead)*regdata.RegFloatVal);
						calDacIvalue = calDacIvalue + regdata.RegFloatVal;
						regdata.RegFloatVal = calDacIvalue;	
					}
				}
				else if(regdata.RegFloatVal <= 0.5)
				{
					printf("set 0.1~0.5.......\n");
					iFlashHead = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 3);
					iFlashTail = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 4);
					printf("iFlashHead = %lf\n", iFlashHead);
					printf("iFlashTail = %lf\n", iFlashTail);
					if((iFlashHead != -1) && (iFlashTail != -1))
					{
						calDacIvalue = (iFlashHead + iFlashTail)/2;
						calDacIvalue = calDacIvalue + regdata.RegFloatVal;
						regdata.RegFloatVal = calDacIvalue;	
					}	
				}
				else if(regdata.RegFloatVal <= 0.9)
				{
					printf("set 0.5~0.9.......\n");	
					iFlashHead = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 4);
					iFlashTail = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 5);
					printf("iFlashHead = %lf\n", iFlashHead);
					printf("iFlashTail = %lf\n", iFlashTail);
					if((iFlashHead != -1) && (iFlashTail != -1))
					{
						//calDacIvalue = (regdata.RegFloatVal + iFlashHead - (iFlashHead - iFlashTail)*0.8)/(1 - ((iFlashHead - iFlashTail)/0.4));
						calDacIvalue = iFlashHead - (((iFlashHead - iFlashTail)/0.4)*(regdata.RegFloatVal - 0.5));
						calDacIvalue = calDacIvalue + regdata.RegFloatVal;
						regdata.RegFloatVal = calDacIvalue;	
					}
				}
				else
				{
					printf("set 0.9~10.0.......\n");
					iFlashHead = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 5);
					iFlashTail = GetFlashPage_ChannelVal(DAC_V_FLASH_PAGE, channel, 6, 6);
					printf("iFlashHead = %lf\n", iFlashHead);
					printf("iFlashTail = %lf\n", iFlashTail);
					if((iFlashHead != -1) && (iFlashTail != -1))
					{
						//calDacIvalue = (regdata.RegFloatVal - iFlashHead + (((iFlashTail - iFlashHead)/9.1)*0.9))/(1 - ((iFlashTail - iFlashHead)/9.1));
						calDacIvalue = iFlashHead + (((iFlashTail - iFlashHead)/9.1)*(regdata.RegFloatVal - 0.9));
						calDacIvalue = calDacIvalue + regdata.RegFloatVal;
						regdata.RegFloatVal = calDacIvalue;	
					}
				}
				printf("set V =%f\r\n", regdata.RegFloatVal);
				if(regdata.RegFloatVal < 0)		//无法处理小于0的值
					regdata.RegFloatVal = 0;
#endif
				DataStruct.len = 2;
				DataStruct.data = (uint8_t *)(AD5422SetVal(regdata.RegFloatVal, (DataStruct.cmd & 0xf)) == 0 ? "OK" : "ER");
				//AD5422SetCur(10.0,(DataStruct.cmd&0xf));
				//AD5422SetVal(0,(DataStruct.cmd&0xf));
				Datasend(&DataStruct);
				break;
			case 0xa0:
			case 0xa1:
				//AD5422init();
				regdata.reg[0] = CmdRecvBuffer[i][2];
				regdata.reg[1] = CmdRecvBuffer[i][3];
				regdata.reg[2] = CmdRecvBuffer[i][4];
				regdata.reg[3] = CmdRecvBuffer[i][5];
				printf("set I =%f\r\n", regdata.RegFloatVal);

				if(regdata.RegFloatVal <= 0.01)
				{
					iFlashHead = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 1);
					iFlashTail = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 2);
					//0.01 = 0.01 - 0
					range = 0.01;
				}
				else if(regdata.RegFloatVal <= 0.1)
				{
					iFlashHead = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 2);
					iFlashTail = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 3);
					// 0.09 = 0.1 - 0.01
					range = 0.09;
				}
				else if(regdata.RegFloatVal <= 1)
				{
					iFlashHead = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 3);
					iFlashTail = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 4);
					// 0.9 = 1.0 - 0.1
					range = 0.9;
				}
				else if(regdata.RegFloatVal <= 10)
				{
					iFlashHead = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 4);
					iFlashTail = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 5);
					// 9 = 10 - 1
					range = 9;
				}
				else
				{
					iFlashHead = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 5);
					iFlashTail = GetFlashPage_ChannelVal(DAC_I_FLASH_PAGE, channel, 6, 6);
					//10 = 20 - 10
					range = 10;
				}
				
				//DAC电流校准，调整输入的电流值，使输出的电流值达到要求	

				printf("iFlashHead = %lf\n", iFlashHead);
				printf("iFlashTail = %lf\n", iFlashTail);
				if((iFlashHead != -1) && (iFlashTail != -1))
				{
					calDacIvalue = ((regdata.RegFloatVal - iFlashHead)*range)/(iFlashTail - iFlashHead + range);
					regdata.RegFloatVal = calDacIvalue;
					printf("set I =%f\r\n", regdata.RegFloatVal);
				}
				
				if(regdata.RegFloatVal < 0)		//无法处理小于0的值
					regdata.RegFloatVal = 0;

				DataStruct.len = 2;
				DataStruct.data = (uint8_t *)(AD5422SetCur(regdata.RegFloatVal, (DataStruct.cmd & 0xf)) == 0 ? "OK" : "ER");
				Datasend(&DataStruct);
				break;
			case 0xc1: //自动校准
				if (CmdRecvBuffer[i][2] == 0x33)//校准ADC电压
				{
					//printf("recvcmd =%x\r\n", CmdRecvBuffer[i][2]);
					channel = CmdRecvBuffer[i][3] & 0xf;
					
					ADC_V_Calculate(channel);

					DataStruct.len = 2;
					DataStruct.data = (uint8_t *)"OK";
				}
				else if (CmdRecvBuffer[i][1] == 0x55)//校准ADC电流
				{
					printf("recvcmd =%x\r\n", CmdRecvBuffer[i][2]);
					channel = CmdRecvBuffer[i][2] & 0xf;
					printf("channel = %d\n", channel);
					if (CmdRecvBuffer[i][3] == 0xFF)
						ADC_I_Calculate(channel);
					else
						ADC_I_REF_Calculate(channel);
					DataStruct.len = 2;
					DataStruct.data = (uint8_t *)"OK";
				}
				else if (CmdRecvBuffer[i][2] == 0x99)//校准DAC电压
				{
					printf("recvcmd =%x\r\n", CmdRecvBuffer[i][3]);

					num			   = CmdRecvBuffer[i][4];

					regdata.reg[0] = CmdRecvBuffer[i][5];
					regdata.reg[1] = CmdRecvBuffer[i][6];
					regdata.reg[2] = CmdRecvBuffer[i][7];
					regdata.reg[3] = CmdRecvBuffer[i][8];
					printf("DAC_V_Calculate V =%f\r\n", regdata.RegFloatVal);

					channel = CmdRecvBuffer[i][3] & 0xf;
					printf("channel = %d\n", channel);

					DAC_V_Calculate(channel, num, regdata.RegFloatVal);
					//DAC_FLash_Value_Clear(0, channel);		//清除标志
					
					DataStruct.len = 2;
					DataStruct.data = (uint8_t *)"OK";
				}
				else if (CmdRecvBuffer[i][2] == 0xaa)//校准DAC电流
				{
					printf("recvcmd =%x\r\n", CmdRecvBuffer[i][3]);

					num			   = CmdRecvBuffer[i][4];

					regdata.reg[0] = CmdRecvBuffer[i][5];
					regdata.reg[1] = CmdRecvBuffer[i][6];
					regdata.reg[2] = CmdRecvBuffer[i][7];
					regdata.reg[3] = CmdRecvBuffer[i][8];
					printf("DAC_I_Calculate I =%f\r\n", regdata.RegFloatVal);

					channel = CmdRecvBuffer[i][3] & 0xf;
					printf("channel = %d\n", channel);

					DAC_I_Calculate(channel, num, regdata.RegFloatVal);
					//DAC_FLash_Value_Clear(1, channel);			//清除标志
					
					DataStruct.len = 2;
					DataStruct.data = (uint8_t *)"OK";
				}
				else
				{
					DataStruct.len = 2;
					DataStruct.data = (uint8_t *)"ER";
				}
				Datasend(&DataStruct);
				break;
			default:
				break;
		}
	}
}
void app_start(void)
{
	//uint16_t flash_flag;
	//RegValDef tempdata;
	//uint16_t rbuffer[CAL_NUM * 2];

	printf("app starting...\r\n");

	//flag_modbus_cmd = 0;
	//rs485_receive_dma(rx_buffer, 12);

	//while (1);
	initRingbuffer();

	AD4112_init(&adc_4112_dev1, 1);
	AD4112_init(&adc_4112_dev2, 2);
	I_open = 0;
	V_open = 0;
	AD5422init(V_MODE, DAC1_PORT); //
	AD5422init(V_MODE, DAC2_PORT); //
	/*tbuffer[0]=0x5c5c;
	for(i=0;i<CAL_NUM;i++)
	{
	  tempdata.RegFloatVal=1.999999;
		if(i%2)tempdata.RegFloatVal=-1.222;
		memcpy(&tbuffer[2*i],tempdata.reg,4);
	}*/
	//FLASH_Write(ADDR_FLASH_PAGE_X(0)+2,tbuffer, ARRAY_SIZE(tbuffer));
	//flash_flag=0x5c5c;
	//FLASH_Write(ADDR_FLASH_PAGE_X(0),&flash_flag, 1);

/* 	flash_flag = 0;
	FLASH_Read(ADDR_FLASH_PAGE_X(0), &flash_flag, 1);
	if (flash_flag == 0x5c5c)
	{
		//FLASH_Read(ADDR_FLASH_PAGE_X(0)+2, rbuffer, ARRAY_SIZE(rbuffer));
		 for(i=0;i<CAL_NUM;i++)
		 { 
			 memcpy(tempdata.reg,&rbuffer[2*i],4);
			 calset[i].RegFloatVal=tempdata.RegFloatVal;
			 printf("%f \r\n",tempdata.RegFloatVal);
		 }//
		FLASH_Read(ADDR_FLASH_PAGE_X(0) + 2, rbuffer, 2);

		memcpy(tempdata.reg, &rbuffer[0], 4);

		VOLTAGE_REF = tempdata.RegFloatVal;
		printf("VOLTAGE_REF=%f \r\n", VOLTAGE_REF);
	}
	else
	{
		//for(i=0;i<CAL_NUM;i++)
		   calset[i].RegFloatVal=0.0;//
		VOLTAGE_REF = 2.500;
	} */
	printf("init ok \r\n");
}

/*
static void ad4112_0x10_func(uint8_t c_mask)
{
	uint8_t i, channel_num = 0, channel_single = 0, CHMAP_key, FILTCON_key, is_open = 0;
	uint32_t value;
	
	value = AD4112_get_param(AD717X_ADCMODE_REG);
	value &= ~(0x01 << 13);
	AD4112_set_param(AD717X_ADCMODE_REG, value);
	// printf("a 0x%02x:0x%04x \r\n", AD717X_ADCMODE_REG, AD4112_get_param(AD717X_ADCMODE_REG));
	
	for (i = 0;i < 8;i++)
	{
		CHMAP_key 	= AD717X_CHMAP0_REG + i;
		FILTCON_key = AD717X_FILTCON0_REG + i;
		
		// 開啓指定通道
		is_open = ((c_mask >> i) & 0x01);
		value = AD4112_get_param(CHMAP_key);
		value &= ~(0x01 << 15); //清除該位
		value |= (is_open?AD717X_CHMAP_REG_CH_EN:AD717X_CHMAP_REG_CH_DIS);
		AD4112_set_param(CHMAP_key, value);
		
		// 31.25k/sps
		value = AD4112_get_param(FILTCON_key);
		value &= ~(0x1F << 0);
		value |= AD717X_FILT_CONF_REG_ODR(0);
		AD4112_set_param(FILTCON_key, value);
		
		if (is_open) channel_num++;
		if (channel_num == 1 && is_open) channel_single = i;
		printf("b 0x%02x:0x%04x 0x%02x:0x%04x \r\n", CHMAP_key, AD4112_get_param(CHMAP_key), FILTCON_key, AD4112_get_param(FILTCON_key));
	}
	
	if (channel_num == 1)
	{
		value = AD4112_get_param(AD717X_ADCMODE_REG);
		value &= ~(0x01 << 13);
		value |= AD717X_ADCMODE_SING_CYC;
		AD4112_set_param(AD717X_ADCMODE_REG, value);
		
		// 10.417k/sps
		value = AD4112_get_param(AD717X_FILTCON0_REG + channel_single);
		value &= ~(0x1F << 0);
		value |= AD717X_FILT_CONF_REG_ODR(7);
		AD4112_set_param(AD717X_FILTCON0_REG + channel_single, value);
		// printf("c 0x%02x:0x%04x 0x%02x:0x%04x \r\n", AD717X_ADCMODE_REG, AD4112_get_param(AD717X_ADCMODE_REG), AD717X_FILTCON0_REG + channel_single, AD4112_get_param(AD717X_FILTCON0_REG + channel_single));
	}
}
*/
