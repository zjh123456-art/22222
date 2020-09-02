/*
 * spi.h
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_SPI_H_
#define INC_SPI_H_
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define	SPI_CPHA			0x01
#define	SPI_CPOL			0x02

typedef enum spi_type {
	GENERIC_SPI
} spi_type;

typedef enum spi_mode {
	SPI_MODE_0 = (0 | 0),
	SPI_MODE_1 = (0 | SPI_CPHA),
	SPI_MODE_2 = (SPI_CPOL | 0),
	SPI_MODE_3 = (SPI_CPOL | SPI_CPHA)
} spi_mode;

typedef struct spi_init_param {
	uint32_t		id;
	uint32_t		max_speed_hz;
	uint8_t			chip_select;
	enum spi_type	type;
	enum spi_mode	mode;
} spi_init_param;

typedef struct spi_desc {
	uint32_t		id;
	uint32_t		max_speed_hz;
	uint8_t			chip_select;
	enum spi_mode	mode;
} spi_desc;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
/* Initialize the SPI communication peripheral. */
int32_t spi_init(struct spi_desc **desc, struct spi_init_param *param);

/* Free the resources allocated by spi_init() */
int32_t spi_remove(struct spi_desc *desc);

/* Write data to SPI. */
int32_t spi_write(struct spi_desc *desc, uint8_t *data, uint8_t bytes_number);

/* read data from SPI. */
int32_t spi_read(struct spi_desc *desc, uint8_t *data, uint8_t bytes_number);

/* Write and read data to/from SPI. */
int32_t spi_write_and_read(struct spi_desc *desc, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* INC_SPI_H_ */
