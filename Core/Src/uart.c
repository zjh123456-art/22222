/*
 * uart.c
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */
#include <stdio.h>
#include "uart.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* With GCC, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(FILE *f)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	if (status != HAL_OK) return -1;
	
	return ch;
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    // Error Handler
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void dbg_scanf(char *ch)
{	
	HAL_StatusTypeDef status = HAL_BUSY;
	
	while (1)
	{
		status = HAL_UART_Receive(&huart1,(uint8_t *)ch, 1, 0xFFFF);
		if (status == HAL_OK) break;
	}
	
	return;
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    // Error Handler
  }
  /* USER CODE BEGIN USART2_Init 2 */
	
  /* USER CODE END USART2_Init 2 */
}

HAL_StatusTypeDef rs485_receive_ch(char* ch)
{
	return HAL_UART_Receive(&huart2,(uint8_t *)ch, 1, 0xFFFF);
}

HAL_StatusTypeDef rs485_receive_IT(uint8_t* ch, uint8_t len)
{
	return HAL_UART_Receive_IT(&huart2, ch, len);
}

HAL_StatusTypeDef rs485_receive_dma(uint8_t* ch, uint8_t len)
{
	return HAL_UART_Receive_DMA(&huart2, ch, len);
}

HAL_StatusTypeDef rs485_transmit_ch(char ch)
{
	return HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
}

HAL_StatusTypeDef rs485_transmit_dma(uint8_t *str, uint32_t len)
{
	// return HAL_UART_Transmit(&huart2, str, len, 1);
	return HAL_UART_Transmit_DMA(&huart2, str, len);
}
