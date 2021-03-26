/*
 * 	modbus.c
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */
 
#include <stdio.h> 

#include "uart.h"
#include "modbus.h"

void modbus_epoll(void)
{	
	uint32_t i, temp;
	HAL_StatusTypeDef status;
	
	while(1);
//	rs485_receive_dma(&rx_char, 1);
	
//	HAL_SPI_Receive_DMA(&hspi1, rx_modbus_cmd, 2);
//	while(1);

//	while(1){
//		modbus_response(8);
//	}

//	HAL_UART_Receive_DMA(&huart2, &rx_char, 1);
//	while(1);

//	HAL_UART_Receive_DMA(&huart2, rx_modbus_cmd, MODBUS_CMD_LEN);
//	while(1);

//	rs485_receive_modbus_IT((char *)rx_modbus_cmd);
//	HAL_UART_Receive_IT(&huart2, &rx_char, 1);
//	while(1);

//	while (1)
//	{
//		if (flag_modbus == 1) break;
//		status = rs485_receive_modbus((char *)rx_modbus_cmd);
//		if (status == HAL_OK) 
//		{
//			flag_modbus = 1;
//			for (i = 0;i < MODBUS_CMD_LEN;i++)
//			{
//				rx_buffer[rx_cnt] = rx_modbus_cmd[i];
//				printf("0x%02x ", rx_modbus_cmd[i]);
//				rx_cnt++;
//			}
//			printf("\r\n");
//			rx_cnt = 0;
//			
//			modbus_work();
//			flag_modbus = 0;
//		}
//	}
}
 
uint8_t modbus_0x03_func(uint8_t* rx_buffer)
{
	
	uint8_t modbus_len = 8;
	if (crc_16(rx_buffer, modbus_len) == 0) return modbus_len;
	return 0;
}

uint8_t modbus_0x10_func(uint8_t* rx_buffer)
{
	uint8_t modbus_len;
	uint16_t addr, addr_num;
	addr 		 = rx_buffer[2] << 8 | rx_buffer[3];
	addr_num = rx_buffer[4] << 8 | rx_buffer[5];
	modbus_len = 2*addr_num + 8;
	
	if (crc_16(rx_buffer, modbus_len) == 0) return modbus_len;
	return 0;
}

void modbus_response(uint8_t modbus_func_id, uint8_t* data, uint16_t data_len)
{
	uint16_t modbus_len, i, crc;
	uint8_t modbus_frame[BUFFER_SIZE]; 
	if (!(modbus_func_id == MODBUS_FUNC_0x03 || modbus_func_id == MODBUS_FUNC_0x10)) return;
	
	modbus_frame[0] = MODBUS_SLAVE_ID;
	modbus_frame[1] = modbus_func_id;
	modbus_frame[2] = data_len;
	
	for (i = 0;i< data_len;i++)
	{
		modbus_len = 3 + i; 
		modbus_frame[modbus_len] = data[i];
	}
	
	crc = crc_16(modbus_frame, modbus_len + 1);
	modbus_frame[modbus_len + 1] = (uint8_t) crc;
	modbus_frame[modbus_len + 2] = (uint8_t)(crc >> 8);
	modbus_len += 3;
	
//	for (i = 0;i<modbus_len;i++)
//	{
//		printf("0x%02x ", modbus_frame[i]);
//	}
//	printf("\r\n");
	
	rs485_transmit_dma(modbus_frame, modbus_len);
}

void modbus_error(uint8_t modbus_func_id, modbus_error_code error)
{
	uint8_t modbus_frame[6]; 
	uint16_t crc;
	modbus_frame[0] = MODBUS_SLAVE_ID;
	modbus_frame[1] = modbus_func_id;
	modbus_frame[2] = 1;
	modbus_frame[3] = error;
	crc = crc_16(modbus_frame, 4);
	modbus_frame[4] = (uint8_t) crc;
	modbus_frame[5] = (uint8_t)(crc >> 8);
	
	rs485_transmit_dma(modbus_frame, 6);
	return;
}


uint16_t crc_16(uint8_t *vptr, uint8_t len)
{
	uint16_t TCPCRC = 0xFFFF;
	uint16_t POLYNOMIAL = 0xA001;
	uint8_t i, j;

	for (i = 0; i < len; i++)
	{
		TCPCRC ^= vptr[i] ;
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

uint8_t modbus_parse(uint8_t* rx_buffer)
{
	uint8_t i, modbus_len;
	
	if (rx_buffer[0] != MODBUS_SLAVE_ID) return 0;
	
	switch (rx_buffer[1])
	{
		case 0x03:
			modbus_len = modbus_0x03_func(rx_buffer);
			break;
		case 0x10:
			modbus_len = modbus_0x10_func(rx_buffer);
			break;
	}
	
	return modbus_len;
}





