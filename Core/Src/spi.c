/*
 * spi.c
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */
#include <stdio.h>
#include "spi.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
//  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    // Error Handler;
//  }
	hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    // Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void)
{

	hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    // Error_Handler();
  }

}
/**
 * @brief Initialize the SPI communication peripheral.
 * @param desc - The SPI descriptor.
 * @param init_param - The structure that contains the SPI parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_init(struct spi_desc **desc, struct spi_init_param *param) {
	if (desc) {
		// Unused variable - fix compiler warning
		*desc=(spi_desc *)malloc(sizeof(spi_desc));
	}

	return 0;
}

/**
 * @brief Free the resources allocated by spi_init().
 * @param desc - The SPI descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_remove(struct spi_desc *desc) {
	if (desc) {
		// Unused variable - fix compiler warning
		free(desc);
	}

	return 0;
}

/**
 * @brief Write data to SPI.
 * @param desc - The SPI descriptor.
 * @param data - The buffer with the transmitted/received data.
 * @param bytes_number - Number of bytes to write/read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_write(struct spi_desc *desc, uint8_t *data, uint8_t bytes_number) {
	if (data == NULL || desc==NULL) return -1;
//	printf(">>>spi_write>>>desc->id=%d\r\n",desc->id);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(desc->id==1?&hspi1:&hspi2, data, bytes_number, 0xFFFF);
	if (status != HAL_OK) return -1;
	
	return 0;
}

/**
 * @brief read data from SPI.
 * @param desc - The SPI descriptor.
 * @param data - The buffer with the transmitted/received data.
 * @param bytes_number - Number of bytes to write/read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_read(struct spi_desc *desc, uint8_t *data, uint8_t bytes_number) {
	if (data == NULL && desc==NULL) return -1;
//	printf(">>>spi_read>>>desc->id=%d bytes_number=%d\r\n",desc->id,bytes_number);
	HAL_StatusTypeDef status = HAL_SPI_Receive(desc->id==1?&hspi1:&hspi2, data, bytes_number, 0xFFFF);
	if (status != HAL_OK) return -1;
	
	return 0;
}

/**
 * @brief Write and read data to/from SPI.
 * @param desc - The SPI descriptor.
 * @param pTxData - The buffer with the transmitted data.
 * @param pRxData - The buffer with the received data.
 * @param Size - Number of bytes to write/read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_write_and_read(struct spi_desc *desc, uint8_t *pTxData,uint8_t *pRxData, uint16_t Size) {
	HAL_StatusTypeDef spi_status;
//printf(">>>>>>desc->id=%d\r\n",desc->id);
	if ( desc==NULL) return -1;
	if (Size) {
		spi_status = HAL_SPI_TransmitReceive(desc->id==1?&hspi1:&hspi2, pTxData, pRxData, Size, 0xFFFF);
		
		if (spi_status != HAL_OK)
			return -1;
	}

	return 0;
}


