/*
 * clock.h
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

void SystemClock_Config(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

void him_start_ti(uint8_t him_index);
void him_stop_ti(uint8_t him_index);
void him_set_counter(uint8_t him_index, uint32_t cnt);

void him2_callback(void);
void him4_callback(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* INC_CLOCK_H_ */
