/*
 *  uart.h
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

void dbg_scanf(char *ch);

HAL_StatusTypeDef rs485_receive_ch(char* ch);
HAL_StatusTypeDef rs485_receive_IT(uint8_t* ch, uint8_t len);
HAL_StatusTypeDef rs485_receive_dma(uint8_t* ch, uint8_t len);

HAL_StatusTypeDef rs485_transmit_ch(char ch);
HAL_StatusTypeDef rs485_transmit_dma(uint8_t *str, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_UART_H_ */
