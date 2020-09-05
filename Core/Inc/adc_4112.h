/*
 * adc_4112.h
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */

#ifndef INC_ADC_4112_H_
#define INC_ADC_4112_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "AD717X.h"

#define VOLTAGE_REF						2.5				//˜ËœÊ…¢¿¼ëŠ‰º
#define RESISTANCE_REF				50
#define DIFF_PRESSURE					0.1		

#define AD4112_BI_UNIPOLAR		1

#define CHANEL_MIN						0
#define CHANEL_MAX						7

#define NULL_POINTER					(void *)0
#define INVALID_VAL 					-1 /* Invalid argument */
#define COMM_ERR    					-2 /* Communication error on receive */
#define TIMEOUT     					-3 /* A timeout has occured */


int32_t AD4112_init(ad717x_dev **adc_4112_dev,int32_t ID);
//ad717x_dev* AD4112_get_device(void);

uint32_t AD4112_get_param(ad717x_dev *dev,uint8_t key);
void AD4112_set_param(ad717x_dev *dev,uint8_t key, uint32_t value);
int32_t AD4112_ReadData(ad717x_dev *adc_4112_dev,uint32_t* pData);
int32_t AD4112_ReadData_ByChannel(ad717x_dev *dev, uint32_t* pData, uint8_t ch);
double AD4112_Code_To_Am(ad717x_dev *adc_4112_dev,uint32_t chanel, uint32_t code);
double AD4112_Code_To_Ampere(ad717x_dev *device, uint32_t chanel, uint32_t code);
double AD4112_Code_To_Voltage(ad717x_dev *device, uint32_t chanel, uint32_t code);

static double AD4112_Code_To_value(ad717x_dev *device, uint32_t chanel, uint32_t code);

int32_t AD4112_remove(ad717x_dev *adc_4112_dev);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADC_4112_H_ */
