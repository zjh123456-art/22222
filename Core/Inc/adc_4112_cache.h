/*
 *  app.h
 *
 *  Created on: July 7, 2020
 *      Author: aguang
 */

#ifndef INC_ADC_4112_CACHE_H_
#define INC_ADC_4112_CACHE_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef struct {
	uint16_t data[8];
	uint16_t index;
} ad4112_channel_data;

typedef struct {
	ad4112_channel_data datas[8];
} ad4112_datas;

void ADC_4112_data_stop(void);
void ADC_4112_data_start(void);

void ADC_4112_data_out(uint8_t channel_mask, uint8_t channel_num);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADC_4112_CACHE_H_ */
