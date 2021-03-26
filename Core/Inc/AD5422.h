/********************************************************************************
 Author : CAC (CustomerApplications Center, Asia) 

 Date : 2012-07-28

 File name : AD5422.h

 Description : AD5422 write and read

 Hardware plateform : ADuC7026_DEMO_V1.2 + EVAL-AD5422EBZ   	
********************************************************************************/

#ifndef AD5422_H
#define AD5422_H

typedef enum 
{
	DAC1_PORT=0,
	DAC2_PORT
}DAC_PORT;

typedef enum 
{
	V_MODE=0,
	I_MODE
}MODE;
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

//AD5422 control

void WriteToAD5422(unsigned char count,unsigned char *buf,unsigned char port);

void ReadFromAD5422(unsigned char count,unsigned char *buf,unsigned char port);
int AD5422init(unsigned	char IV_flag,unsigned	char port);
int AD5422SetVal(float val,unsigned	char port);
int AD5422SetCur(float Iout,unsigned	char port);
#ifdef __cplusplus
}
#endif


#endif
