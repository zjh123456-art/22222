/********************************************************************************
 Author : CAC (CustomerApplications Center, Asia) 

 Date : 2012-07-28

 File name : AD5422.c

 Description : AD5422 write and read

 Hardware plateform : ADuC7026_DEMO_V1.2 + EVAL-AD5422EBZ   	
********************************************************************************/

//#include "ADuC7026.h"
//#include "ADuC7026Driver.h"
#include <stdio.h>
#include <math.h>
#include "AD5422.h"

//#define DEBUG
#ifndef DEBUG
#define printf(format, ...)
#endif
#define REFIN (5.0)
//#define ADuC7026OutputBit() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
typedef struct
{
	GPIO_TypeDef *GPIOx;
	uint16_t pin;
} ioport;
typedef struct
{
	ioport SDIN;
	ioport SCLK;
	ioport LATCH;
	ioport SDO;
} AD5422port;
/*
AD5422port port0={
	{GPIOA,GPIO_PIN_1},//PA1 sdin
	{GPIOA,GPIO_PIN_0},//PA0 sclk
	{GPIOC,GPIO_PIN_3},//PC3 latch
	{GPIOA,GPIO_PIN_2},//PA2 sdo
};
AD5422port port1={
	{GPIOA,GPIO_PIN_11},//PA11
	{GPIOA,GPIO_PIN_8},//PA8
	{GPIOC,GPIO_PIN_9},//PC9
	{GPIOA,GPIO_PIN_12},//PA12
};
*/
AD5422port dacport[] = {
	{
		{GPIOA, GPIO_PIN_1}, //PA1 sdin
		{GPIOA, GPIO_PIN_0}, //PA0 sclk
		{GPIOC, GPIO_PIN_3}, //PC3 latch
		{GPIOA, GPIO_PIN_2}, //PA2 sdo
	},
	{
		{GPIOA, GPIO_PIN_11}, //PA11
		{GPIOA, GPIO_PIN_8},  //PA8
		{GPIOC, GPIO_PIN_9},  //PC9
		{GPIOA, GPIO_PIN_12}, //PA12
	}};

void Delay(unsigned int ms)
{
	unsigned int i, j;
	for (i = 0; i < ms; i++)
		for (j = 0; j < 500; j++)
			;
}
//the function write data to the AD5422's register
void WriteToAD5422(unsigned char count, unsigned char *buf, unsigned char port)
{
	unsigned char ValueToWrite = 0;
	unsigned char i = 0;
	unsigned char j = 0;

	HAL_GPIO_WritePin(dacport[port].LATCH.GPIOx, dacport[port].LATCH.pin, 0);
	for (i = count; i > 0; i--)
	{
		ValueToWrite = *(buf + i - 1);
		//printf("%02x\n",ValueToWrite);
		for (j = 0; j < 8; j++)
		{
			HAL_GPIO_WritePin(dacport[port].SCLK.GPIOx, dacport[port].SCLK.pin, 0); //sclk
			if (0x80 == (ValueToWrite & 0x80))
			{
				HAL_GPIO_WritePin(dacport[port].SDIN.GPIOx, dacport[port].SDIN.pin, 1); //SDIN	    //Send one to SDIN pin of AD5422
			}
			else
			{
				HAL_GPIO_WritePin(dacport[port].SDIN.GPIOx, dacport[port].SDIN.pin, 0); //Send zero to SDIN pin of AD5422
			}

			Delay(1);
			HAL_GPIO_WritePin(dacport[port].SCLK.GPIOx, dacport[port].SCLK.pin, 1); //sclk
			Delay(1);
			ValueToWrite <<= 1; //Rotate data
		}
	}

	HAL_GPIO_WritePin(dacport[port].SCLK.GPIOx, dacport[port].SCLK.pin, 0);
	Delay(1);
	HAL_GPIO_WritePin(dacport[port].LATCH.GPIOx, dacport[port].LATCH.pin, 1);
	Delay(20);
}

//the function read the data register from AD5422
void ReadFromAD5422(unsigned char count, unsigned char *buf, unsigned char port)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char iTemp = 0;
	unsigned char RotateData = 0;
	HAL_GPIO_WritePin(dacport[port].LATCH.GPIOx, dacport[port].LATCH.pin, 0);
	for (j = count; j > 0; j--)
	{
		for (i = 0; i < 8; i++)
		{
			HAL_GPIO_WritePin(dacport[port].SCLK.GPIOx, dacport[port].SCLK.pin, 0); //sclk
			RotateData <<= 1;														//Rotate data
			Delay(1);
			HAL_GPIO_WritePin(dacport[port].SDIN.GPIOx, dacport[port].SDIN.pin, 0);   //Write a nop condition when read the data.
			iTemp = HAL_GPIO_ReadPin(dacport[port].SDO.GPIOx, dacport[port].SDO.pin); //Read SDO of AD5422
			HAL_GPIO_WritePin(dacport[port].SCLK.GPIOx, dacport[port].SCLK.pin, 1);   //sclk

			if (iTemp == 1)
			{
				RotateData |= 1;
			}
			Delay(1);
		}
		*(buf + j - 1) = RotateData;
	}

	HAL_GPIO_WritePin(dacport[port].SCLK.GPIOx, dacport[port].SCLK.pin, 0);
	Delay(1);
	HAL_GPIO_WritePin(dacport[port].LATCH.GPIOx, dacport[port].LATCH.pin, 1);
	Delay(20);
}

int AD5422SetVal(float val, unsigned char port)
{

	unsigned char buf[3] = {0, 0, 0};
	unsigned char buf_read[3] = {0x01, 0x00, 0x02};
	unsigned int data;
	if (port > 1 || val < 0)
		return -2;
	AD5422init(V_MODE, port);
	if (val == 0 || val >= 10)
	{
		data = val ? 0xffff : 0x0;
	}
	else
	{
		data = (unsigned short)((val * pow(2, 16)) / (2.0 * REFIN)); //增益2 参考电压5V
	}
	buf[2] = 0x01; //Data register
	buf[1] = (data >> 8) & 0xff;
	buf[0] = data & 0xff;
	WriteToAD5422(3, buf, port); //Write 0x010000 to SHIFT REGISTER  to write 0x018000 to DATA REGISTER

	WriteToAD5422(3, buf_read, port);
	ReadFromAD5422(3, buf_read, port); //Read data REGISTER
	printf("a=%02x\n", buf[2]);
	printf("b=%02x\n", buf[1]);
	printf("c=%02x\n", buf[0]);
	printf("a=%02x\n", buf_read[2]);
	printf("b=%02x\n", buf_read[1]);
	printf("c=%02x\n", buf_read[0]);
	if (buf_read[0] == buf[0] && buf_read[1] == buf[1])
		return 0;
	else
		return -1;
}
int AD5422SetCur(float Iout, unsigned char port)
{

	unsigned char buf[3] = {0, 0, 0};
	unsigned char buf_read[3] = {0x01, 0x00, 0x02};
	unsigned short data;
	if (port > 1 || Iout < 0)
		return -2;
	AD5422init(I_MODE, port);
	if (Iout == 0 || Iout >= 24)
	{
		data = Iout ? 0xffff : 0x0;
	}
	else
	{
		data = (unsigned short)((Iout * pow(2, 16)) / 24);
	}

	buf[2] = 0x01; //Data register
	buf[1] = (data >> 8) & 0xff;
	buf[0] = data & 0xff;
	WriteToAD5422(3, buf, port); //Write 0x010000 to SHIFT REGISTER  to write 0x018000 to DATA REGISTER

	WriteToAD5422(3, buf_read, port);
	ReadFromAD5422(3, buf_read, port); //Read data REGISTER
	printf("a=%02x\n", buf[2]);
	printf("b=%02x\n", buf[1]);
	printf("c=%02x\n", buf[0]);
	printf("a=%02x\n", buf_read[2]);
	printf("b=%02x\n", buf_read[1]);
	printf("c=%02x\n", buf_read[0]);
	if (buf_read[0] == buf[0] && buf_read[1] == buf[1])
		return 0;
	else
		return -1;
}
int AD5422init(unsigned char IV_flag, unsigned char port)
{

	unsigned char buf[3] = {0, 0, 0};
	unsigned char buf_read[3] = {0x02, 0, 0};
	static unsigned char flag[2] = {0xff, 0xff};
	if (flag[port] == IV_flag)
		return 0;
	else
	{
		flag[port] = IV_flag;
	}

	buf[2] = 0x56; //reset
	buf[1] = 0x00;
	buf[0] = 0x01;
	WriteToAD5422(3, buf, port); //Write 0x551000 to SHIFT REGISTER  to write 1005 to control register
	Delay(10000);
	buf[2] = 0x55; //Control Register
				   //	buf[1] = 0x1f;              //Enable Slew Rate and the Slew Rate Time is 20s while selecting the current mode
				   //	buf[0] = 0x15;
				   //	buf[1] = 0x1d;              //Enable Slew Rate and the Slew Rate Time is 12s while selecting the current mode
				   //	buf[0] = 0x15;
				   //	buf[1] = 0x1c;              //Enable Slew Rate and the Slew Rate Time is 4.8s while selecting the current mode
				   //	buf[0] = 0x35;
	buf[1] = 0x31; //Enable Slew Rate and the Slew Rate Time is 10ms while selecting the current mode
				   //	  buf[0] = 0x01;				//Selecting the Voltage Mode(0V - 10V)
				   //	  buf[0] = 0xB2;				//Selecting the Voltage Mode(-5V - 5V)

	//	buf[1] = 0x10;              //Disable Slew Rate
	//	buf[0] = 0x00;				//Selecting the Voltage Mode(0V - 5V)
	//	buf[0] = 0x01;				//Selecting the Voltage Mode(0V - 10V)
	//	buf[0] = 0x02;				//Selecting the Voltage Mode(-5V - 5V)
	//	buf[0] = 0x03;				//Selecting the Voltage Mode(-10V - 10V)
	//	buf[0] = 0x05;				//Selecting the Current Mode(4mA - 20mA)
	//	buf[0] = 0x06;				//Selecting the Current Mode(0mA - 20mA)
	//	buf[0] = 0x07;				//Selecting the Current Mode(0mA - 24mA)
	if (IV_flag)
	{
		buf[0] = 0x07; //Selecting the Current Mode(0mA - 24mA)
	}
	else
	{
		buf[0] = 0x01; //Selecting the Voltage Mode(0V - 10V)
	}
	WriteToAD5422(3, buf, port); //Write 0x551000 to SHIFT REGISTER  to write 1005 to control register
								 //WriteToAD5422(3,buf,1);		//Write 0x551000 to SHIFT REGISTER  to write 1005 to control register
}