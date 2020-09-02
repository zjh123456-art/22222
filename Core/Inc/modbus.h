/*
 * 	modbus.h
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */
 
#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f1xx_hal.h"

#define MODBUS_CMD_LEN			8
#define BUFFER_SIZE					64
#define MODBUS_SLAVE_ID			0x01
#define MODBUS_FUNC_0x03		0x03
#define MODBUS_FUNC_0x10		0x10

typedef enum{
	CRC_ERROR = 0x50,
	FUN_ERROR,
	ADDR_ERROR,
	CHAN_ERROR
} modbus_error_code;

void modbus_epoll(void);

uint8_t modbus_parse(uint8_t* rx_buffer);
uint8_t modbus_0x03_func(uint8_t* rx_buffer);
uint8_t modbus_0x10_func(uint8_t* rx_buffer);
void modbus_error(uint8_t modbus_func_id, modbus_error_code error);
void modbus_response(uint8_t modbus_func_id, uint8_t* data, uint16_t data_len);

uint16_t crc_16(uint8_t *vptr, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_MODBUS_H_ */
