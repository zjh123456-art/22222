/*
 * app.c
 *
 *  Created on: July 6, 2020
 *      Author: aguang
 */ 
#include <stdio.h> 

#include "uart.h"  
#include "modbus.h"
#include "adc_4112.h"
#include "adc_4112_cache.h"
#include "clock.h"
#include "app.h"
 
volatile uint8_t flag_modbus_cmd;				//volatile 禁止編譯器優化省略變量變化
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t gobal_channel_num;
uint8_t gobal_channel_mask;

uint8_t rx_char;
uint8_t modbus_cmd[MODBUS_CMD_LEN];
 
void app_start(void)
{
	printf("app starting...\r\n");
	
	flag_modbus_cmd = 0;
	rs485_receive_dma(rx_buffer, 12);
	
	while (1);
}

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

static void ad4112_0x11_func(uint8_t filter)
{
}

static void ad4112_0x12_func(uint8_t on_off)
{
	uint32_t ret;
	ad717x_dev *adc_4112_dev1;
	if (on_off == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		ADC_4112_data_stop();
		him_stop_ti(2);
		AD4112_remove(adc_4112_dev1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		ret = AD4112_init(&adc_4112_dev1,1);
		him_start_ti(2);
		ADC_4112_data_start();
	} 
	else if (on_off == 0) 
	{
		him_stop_ti(2);
		AD4112_remove(adc_4112_dev1);
	}
}

static uint8_t get_channel_num(uint8_t channel_mask)
{
	uint8_t c_num = 0, i;
	for (i = 0;i < 8;i++)	c_num += ((0x01 << i) & channel_mask)?1:0;
	return c_num;
}

/*************************************
 *          AD4112 modbus寄存器説明
 *          0x00           //寄存器0x00,通道0
 *          0x01           //寄存器0x01,通道1
 *          0x02           //寄存器0x02,通道2
 *          0x03           //寄存器0x03,通道3
 *          0x04           //寄存器0x04,通道4
 *          0x05           //寄存器0x05,通道5
 *          0x06           //寄存器0x06,通道6
 *          0x07           //寄存器0x07,通道7
 *          0x08           //寄存器0x08,通道0~7
 *          0x10           //寄存器0x10,控制打開的通道數 	eg: B_0000_1111 打開0~3通道.
 *          0x11           //寄存器0x11,選擇過濾方式
 *          0x12           //寄存器0x12,0|1:關閉|啓動AD4112
 * 
 *************************************/
void ad4112_parse(uint16_t addr, uint16_t value)
{
	// printf("0x%02x:0x%02x \r\n", addr, value);
	if (addr > 0x12) return;
	switch(addr)
	{ 
		case 0x10:
			gobal_channel_mask = value;
			gobal_channel_num = get_channel_num(gobal_channel_mask);
			ad4112_0x10_func(value);
			break;
		case 0x11:
			ad4112_0x11_func(value);
			break;
		case 0x12:
			ad4112_0x12_func(value);
			break;
	}
	
}

void app_work(void)
{
	uint8_t i, k;
	uint8_t modbus_len;
	uint8_t modbus_func_id;
	uint8_t data[128] = {0};
	uint16_t addr, addr_num, addr_value;
	
	modbus_len = modbus_parse(rx_buffer);
	if (modbus_len == 0) goto out;

	for (i = 0;i<modbus_len;i++) 
		data[i] = rx_buffer[i];
	
	modbus_func_id = data[1];
	if (modbus_func_id == 0x10)
	{
		addr 		 = data[2] << 8 | data[3];
		addr_num = data[4] << 8 | data[5];
		for (i = 0;i < addr_num;i++)
		{
			k = 2*i + 6;
			addr_value = data[k] << 8 | data[k+1];
			ad4112_parse(addr + i, addr_value);
		}
	} 
	else if (modbus_func_id == 0x03)
	{
	}
	flag_modbus_cmd = 0;
	
//	uint8_t j;
//	for (j = 0;j < modbus_len;j++)
//		printf("0x%02x ", data[j]);
//	printf("\r\n");
	
	return;
out:
	flag_modbus_cmd = 0;
	return;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	uint8_t data[BUFFER_SIZE];
	if (flag_modbus_cmd == 1) return;
	
	flag_modbus_cmd = 1;
	
//	uint8_t j;
//	for (j = 0;j < 16;j++)
//		printf("0x%02x ", rx_buffer[j]);
//	printf("z %d\r\n", flag_modbus_cmd);
	
	app_work();
	
//	him_stop_ti(4);
//	him_set_counter(4, 0);
//	if (1 == flag_modbus_cmd) return;
//	
//	rx_buffer[rx_cnt] = rx_char;
//	rx_cnt++;
//	
//	him_start_ti(4);
//	rs485_receive_ch_IT(&rx_char);
}

void him2_callback(void)
{
	ADC_4112_data_out(gobal_channel_mask, gobal_channel_num);
}

void him4_callback(void)
{
//	flag_modbus_cmd = 1;
//	him_stop_ti(4);
}
