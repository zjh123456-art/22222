/*
 * adc_4112.c
 *
 *  Created on: Apr 13, 2020
 *      Author: aguang
 */
 
#include <stdio.h>
#include <math.h>
#include "adc_4112.h"
#include "adc_4112_regs.h"

//ad717x_dev *adc_4112_dev;
 
uint32_t AD4112_get_param(ad717x_dev *dev,uint8_t key)
{
	uint8_t i;
	uint8_t num_regs = sizeof(ad4112_regs) / sizeof(ad717x_st_reg);
	
	for (i = 0;i< num_regs;i++)
	{
		if (dev->regs[i].addr == key) 
		{
			return dev->regs[i].value;
		}
	}
	return NULL;
}
 
void AD4112_set_param(ad717x_dev *dev,uint8_t key, uint32_t value)
{
	uint8_t i;
	uint8_t num_regs = sizeof(ad4112_regs) / sizeof(ad717x_st_reg);
	
	for (i = 0;i< num_regs;i++)
	{
		if (dev->regs[i].addr == key) 
		{
			dev->regs[i].value = dev->regs[i].size == 2?(0xFFFF & value):(0xFFFFFF & value);
			break;
		}
	}
}
 
int32_t AD4112_init(ad717x_dev **adc_4112_dev,int32_t ID)
{
	ad717x_init_param param;
  if(ID==2){
	  param.regs = ad4112_regs2;
	  param.num_regs = sizeof(ad4112_regs2) / sizeof(ad717x_st_reg);
	}else{
		  param.regs = ad4112_regs;
	    param.num_regs = sizeof(ad4112_regs) / sizeof(ad717x_st_reg);
	}

	return AD717X_Init(adc_4112_dev, param,ID);
}

//ad717x_dev* AD4112_get_device(void)
//{
//	return adc_4112_dev;
//}

int32_t AD4112_remove(ad717x_dev *adc_4112_dev)
{
	return AD717X_remove(adc_4112_dev);
}

int32_t AD4112_ReadData(ad717x_dev *adc_4112_dev,uint32_t* pData)
{
	int32_t ret;
	
	if (!adc_4112_dev) return INVALID_VAL;
	
	ret = AD717X_ReadData(adc_4112_dev, pData);
	if (ret < 0) return 1;
	
	return 0;
}
 
 /***************************************************************************//**
 * @brief Reads the conversion result from the device by chanel.
 *
 * @param device - The handler of the instance of the driver.
 * @param pData  - Pointer to store the read data.
 * @param ch  	 - chanel num 0~7.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t AD4112_ReadData_ByChannel(ad717x_dev *device, uint32_t* pData, uint8_t ch)
{
	int32_t ret;
	uint8_t chanel;
	uint8_t isWL16;
	int32_t isDataSata = -1;
	ad717x_st_reg *preg;
	int32_t timeout=0;
	if (!device) return INVALID_VAL;

	preg = AD717X_GetReg(device, AD717X_IFMODE_REG);
	if (!preg) return INVALID_VAL;
	
	ret = AD717X_ReadRegister(device, AD717X_IFMODE_REG);
	if (ret < 0) return INVALID_VAL;
	
	isDataSata = preg->value & AD717X_IFMODE_REG_DATA_STAT;	
	isWL16 = preg->value & AD717X_IFMODE_REG_DATA_WL16;
	
	while (1)
	{
		ret = AD717X_ReadData(device, pData);
		timeout++;
		if (timeout>100) return -1;
		if (ret < 0) continue;
		
		if (!isDataSata) break;

		//printf("adc value:0x%08x\r\n", *pData);
		chanel = *pData & 0x0F;
		if (chanel == ch) 
		{
			*pData >>= AD4112_BI_UNIPOLAR?8:4;
			*pData &= isWL16?0x0000FFFF:0x00FFFFFF;
			break;
		}
	}
	
	return 0;
}

 /***************************************************************************//**
 * @brief 讀取值轉換成電流單位（mA）
 *
 * @param device - The handler of the instance of the driver.
 * @param chanel - 通道號0~7.
 * @param code   - 讀取值
 *
 * @return Returns 電流值
 *******************************************************************************/
double AD4112_Code_To_Am(ad717x_dev *adc_4112_dev,uint32_t chanel, uint32_t code)
{
	double y = AD4112_Code_To_Ampere(adc_4112_dev, chanel, code);
	if (chanel < 4)
	{
		y = y / DIFF_PRESSURE;
	}
	return y * 1000;
}
	
 /***************************************************************************//**
 * @brief 讀取值轉換成電流單位（A）
 *
 * @param device - The handler of the instance of the driver.
 * @param chanel - 通道號0~7.
 * @param code   - 讀取值
 *
 * @return Returns 電流值
 *******************************************************************************/
double AD4112_Code_To_Ampere(ad717x_dev *device, uint32_t chanel, uint32_t code)
{
	double y = AD4112_Code_To_value(device, chanel, code);
	y = y / RESISTANCE_REF;
	return y;
}


 /***************************************************************************//**
 * @brief 讀取值轉換成電壓單位（V）
 *
 * @param device - The handler of the instance of the driver.
 * @param chanel - 通道號0~7.
 * @param code   - 讀取值
 *
 * @return Returns 電壓值
 *******************************************************************************/
double AD4112_Code_To_Voltage_diff(ad717x_dev *device, uint32_t chanel, uint32_t code)
{
	double y = AD4112_Code_To_value(device, chanel, code);
	y = y / DIFF_PRESSURE;
	return y;
}

double AD4112_Code_To_Voltage(ad717x_dev *device, uint32_t chanel, uint32_t code)
{

	int32_t ret;
	uint8_t isWL16;
	ad717x_st_reg *preg;
	double y;
	double exponent = 24;
	
	if (!device) return INVALID_VAL;

	preg = AD717X_GetReg(device, AD717X_IFMODE_REG);
	if (!preg) return INVALID_VAL;
	
	ret = AD717X_ReadRegister(device, AD717X_IFMODE_REG);
	if (ret < 0) return INVALID_VAL;
	
	isWL16 = preg->value & AD717X_IFMODE_REG_DATA_WL16;
	
	exponent = isWL16?16:24;
	
	if (chanel > CHANEL_MAX) return -1;
		
	y = (code /pow(2, exponent ))  * VOLTAGE_REF;
	y = y / DIFF_PRESSURE;
	return y;
}
static double AD4112_Code_To_value(ad717x_dev *device, uint32_t chanel, uint32_t code)
{
	int32_t ret;
	uint8_t isWL16;
	ad717x_st_reg *preg;
	double y;
	double exponent = 24;
	
	if (!device) return INVALID_VAL;

	preg = AD717X_GetReg(device, AD717X_IFMODE_REG);
	if (!preg) return INVALID_VAL;
	
	ret = AD717X_ReadRegister(device, AD717X_IFMODE_REG);
	if (ret < 0) return INVALID_VAL;
	
	isWL16 = preg->value & AD717X_IFMODE_REG_DATA_WL16;
	
	exponent = isWL16?16:24;
	
	if (chanel > CHANEL_MAX) return -1;
		
	y = ((code / pow(2, exponent - 1)) - 1) * VOLTAGE_REF;

	return y;
}
