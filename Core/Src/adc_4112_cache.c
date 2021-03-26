/*
 * adc_4112.c
 *
 *  Created on: July 7, 2020
 *      Author: aguang
 */
#include <stdio.h> 

#include "modbus.h"
#include "adc_4112.h"
#include "adc_4112_cache.h"


ad4112_datas chanel;
volatile uint8_t is_can_read;

void ADC_4112_data_start()
{
	is_can_read = 1;
	int32_t i;
	i = 1000000;
	while(i > 0) i--;
}

void ADC_4112_data_stop()
{
	is_can_read = 0;
	int32_t i, j;
	for (i = 0;i < 8;i++)
	{
		for (j = 0;j < 8;j++)
			chanel.datas[i].data[j] = 0x00;
	}
	i = 1000000;
	while(i > 0) i--;
}

static uint8_t get_channel_index(uint8_t channel_mask)
{
	uint8_t i;
	for (i = 0;i<8;i++)
	{
		if ((0x01 << i) & channel_mask) return i;
	}
	return 0;
}

void ADC_4112_data_out(uint8_t channel_mask, uint8_t channel_num)
{
//	int32_t i;
//	double am;
//	for (i = 0;i<8;i++)
//	{
//		am = AD4112_Code_To_Am(i, chanel.datas[i].data[0]);
//		printf("0x%04x %7.2lf ", chanel.datas[i].data[0], am);
//	}
//	printf("\r\n");

	uint8_t i, j, index;
	uint16_t data_len = 0;
	uint8_t data[256];
	
	if (channel_num == 0) return;
	
		// 八通道快速反應
	if (channel_num == 8)
	{
		for (j = 0; j < 8;j++)
		{
			data[2*j] 	= (uint8_t)(chanel.datas[j].data[0] >> 8);
			data[2*j+1] = (uint8_t) chanel.datas[j].data[0];
		}
		data_len = 16;
		goto modbus_s;
	}
	
	for (i = 0;i < channel_num;i++)
	{
		j = get_channel_index(channel_mask);
		data[2*i] 	= (uint8_t)(chanel.datas[j].data[0] >> 8);
		data[2*i+1] = (uint8_t) chanel.datas[j].data[0];
		data_len += 2; 
		channel_mask &= ~(0x01 << j);
	}
	
modbus_s:	
	modbus_response(MODBUS_FUNC_0x03, data, data_len);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t temp,c;
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	
	//if (is_can_read) AD4112_ReadData(&temp);
	
	c = temp & 0x0F;
	if (c <= 7)
	{
		chanel.datas[c].data[0] = (temp >> 8) & 0xFFFF;
		// printf("0x%x \r\n", chanel.datas[c].data[0]);
	}
	
//	uint8_t temps[4];
//	HAL_SPI_Receive_DMA(&hspi1, temps, 4);
//	c = temps[3] & 0x0F;
//	if (c <= 7)
//	{
//		chanel.datas[c].data[0] = (temps[1] << 8) | temps[2];
//	}
	
//	printf("0x%x 0x%x 0x%x\r\n", temps[0], temps[1], temps[2]);
	
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
