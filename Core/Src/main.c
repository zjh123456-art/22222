/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "dma.h"
#include "spi.h"
#include "uart.h"
#include "gpio.h"
#include "clock.h"
#include "AD717X.h"
#include "adc_4112.h"

//#include "modbus.h"
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void QY_menu(void);
static void test_485(void);
static void test_adc(void);
static void QY_adc(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_DMA_Init();
  MX_SPI1_Init();
	MX_SPI2_Init();

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	printf(">>>>>>>>>>>>>>>>>");
	//MX_TIM2_Init();
	//MX_TIM4_Init();
	//test_adc();
  /* USER CODE BEGIN 2 */
	app_start();
	
//	printf("modbus testing...please Switch to port 485\r\n");
//	AD4112_init();
//	modbus_epoll();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
	{//printf("recvcmd>>>>>rec_flag=%d>>>>>>\r\n",rec_flag);
	 /****************************************************
     cmd{[1B]:           
     0x31 Vol1 ........    0x36 VOL6
     0x51 AM1  ......      0x56 AM6
     0xC0 set
     0x90 DA   
     data[4B]
    
    ****************************************************/
      uart_data_process();
		  cmd_run();

			 
	  }



}

/* USER CODE BEGIN 4 */
static void QY_menu(void)
{
	printf("\r\n");
	printf("*************** QY test menu ***************\r\n");
	printf("1.485 test\r\n");
	printf("2.ADC test\r\n");
	printf("********************************************\r\n");
	printf("input function number: ");
}

static void QY_adc(void)
{
	printf("\r\n");
	printf("-------------- QY ADC test -----------------\r\n");
	printf("1.read adc ID\r\n");
	printf("2.read adc chanel data\r\n");
	printf("--------------------------------------------\r\n");
	printf("type 'q' quit adc test\r\n\r\n");
	printf("input function number: ");
}

static void test_485(void)
{
	printf("modbus testing...please Switch to port 485\r\n");
	//AD4112_init();
	modbus_epoll();
}

static void test_adc(void)
{
	double am;
	int32_t i;
	int32_t ret;
	char adc_cmd = 0;
	uint32_t ret_data;
	ad717x_st_reg *reg_ret;
	ad717x_dev *adc_4112_dev1;
	ad717x_dev *adc_4112_dev2;
	AD4112_init(&adc_4112_dev2,2);
	AD4112_init(&adc_4112_dev1,1);

	
	//adc_4112_dev = AD4112_get_device();
	
	while (1)
	{
			QY_adc();
		
			dbg_scanf(&adc_cmd);
			printf("%c\r\n", adc_cmd);
		
			switch (adc_cmd)
			{
				case '1':
					adc_cmd = 0;
					ret = AD717X_ReadRegister(adc_4112_dev1, AD717X_ID_REG);
					if (ret < 0) continue;
					
					reg_ret = AD717X_GetReg(adc_4112_dev1, AD717X_ID_REG);
					if (reg_ret != NULL) printf("adc id 0x%x\r\n", reg_ret->value);
					break;
				
				case '2':
					adc_cmd = 0;
			
					for (i = 0; i < 8;i++)
					{
						ret = AD4112_ReadData_ByChannel(adc_4112_dev1, &ret_data, i);
						if (ret < 0) continue;
						
						ret = AD717X_ReadRegister(adc_4112_dev1, AD717X_STATUS_REG);
						if (ret < 0) continue;
						reg_ret = AD717X_GetReg(adc_4112_dev1, AD717X_STATUS_REG);
						
						am = AD4112_Code_To_Voltage(adc_4112_dev1,i, ret_data);
						printf("adc status 0x%02x chanel:%d ret value:0x%06x \t\t ampere:%lf\r\n", reg_ret->value, i, ret_data, am);
					}
				
					
						for (i = 0; i < 8;i++)
					{
						ret = AD4112_ReadData_ByChannel(adc_4112_dev2, &ret_data, i);
						if (ret < 0) continue;
						
						ret = AD717X_ReadRegister(adc_4112_dev2, AD717X_STATUS_REG);
						if (ret < 0) continue;
						reg_ret = AD717X_GetReg(adc_4112_dev2, AD717X_STATUS_REG);
						
						am = AD4112_Code_To_Voltage(adc_4112_dev2,i, ret_data);
						printf("adc2 status 0x%02x chanel:%d ret value:0x%06x \t\t ampere:%lf\r\n", reg_ret->value, i, ret_data, am);
					}
					break;
			}
			
			printf("\r\n");
			if (adc_cmd == 'q') 
			{
				AD4112_remove(adc_4112_dev1);
				break;
			}
	}
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
