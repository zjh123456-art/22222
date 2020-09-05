/*
 *  app.h
 *
 *  Created on: July 6, 2020
 *      Author: aguang
 */

#ifndef INC_APP_H_
#define INC_APP_H_
#define DEBUG
#ifndef DEBUG
  #define printf(format,...)
#endif

#define CMDLISTNUM (12)
#define BUFSIZE   (256)
extern unsigned int cmd_flag;
extern unsigned int DataParserFlag;
extern unsigned char CmdRecvBuffer[CMDLISTNUM][10];
extern unsigned char CmdNumCount;
typedef union{
	unsigned char reg[4];
	float RegFloatVal;
}RegValDef;

typedef struct {
unsigned char cmd;
float Regdata;
unsigned int crc;	
}SendDataStruct;

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

void app_start(void); 
void app_work(void);
void ad4112_parse(uint16_t addr, uint16_t value);
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void him2_callback(void);
void him4_callback(void);
unsigned char uart_data_process(void);
void cmd_run();

#ifdef __cplusplus
}
#endif

#endif /* INC_APP_H_ */
