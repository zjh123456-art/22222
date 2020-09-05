/*
 * app.c
 *
 *  Created on: July 6, 2020
 *      Author: aguang
 */ 
#include <stdio.h> 

#include "uart.h"  
//#include "modbus.h"
#include "adc_4112.h"
#include "adc_4112_cache.h"
#include "clock.h"
#define extern
#include "app.h"
#undef extern
 
//volatile uint8_t flag_modbus_cmd;				//volatile 禁止編譯器優化省略變量變化
//uint8_t rx_buffer[BUFFER_SIZE];
//uint8_t gobal_channel_num;
//uint8_t gobal_channel_mask;
//#define printf  printf(..)

#define BUFFER_SIZE  256   //缓冲区的长度,可以修改
#define TIMEOOT_COUNT 100
uint32_t validLen;//已使用的数据长度
uint8_t* pHead = NULL;//环形存储区的首地址
uint8_t* pTail = NULL;//环形存储区的结尾地址
uint8_t* pValid = NULL;//已使用的缓冲区的首地址
uint8_t* pValidTail = NULL;//已使用的缓冲区的尾地址

ad717x_dev *adc_4112_dev1;
ad717x_dev *adc_4112_dev2;
uint8_t I_open,V_open;
uint16_t SendCRC16(uint8_t *vptr, uint8_t len)
{
	uint16_t TCPCRC = 0xFFFF;
	uint16_t POLYNOMIAL = 0xA001;
	uint8_t i, j;

	for (i = 0; i < len; i++)
	{
		TCPCRC ^= vptr[i] ;
		for (j = 0; j < 8; j++)
		{
			if ((TCPCRC & 0x0001) != 0)
			{
				TCPCRC >>= 1;
				TCPCRC ^= POLYNOMIAL;
			}
			else
			{
				TCPCRC >>= 1;
			}
		}
	}
	return TCPCRC;
}
/*
 * 初始化环形缓冲区
 * 环形缓冲区这里可以是malloc申请的内存,也可以是Flash存储介质
 * */
void initRingbuffer(void)
{
    if(pHead == NULL)
    {
        pHead = (uint8_t*) malloc(BUFFER_SIZE);
    }
    pValid = pValidTail = pHead;
    pTail = pHead + BUFFER_SIZE;
    validLen = 0;
		printf("initRingbuffe\r\n");
}
 
/*
 * function:向缓冲区中写入数据
 * param:@buffer 写入的数据指针
 *       @addLen 写入的数据长度
 * return:-1:写入长度过大
 *        -2:缓冲区没有初始化
 * */
int wirteRingbuffer(uint8_t* buffer,uint32_t addLen)
{
	//printf("wirteRingbuffer\r\n");
    if(addLen > BUFFER_SIZE) return -2;
    if(pHead==NULL) return -1;
    //UNUSED(buffer);
 	//printf("DataLength=%d\r\n",addLen);
    //将要存入的数据copy到pValidTail处
    if(pValidTail + addLen > pTail)//需要分成两段copy
    {
        int len1 = pTail - pValidTail;
        int len2 = addLen - len1;
        memcpy( pValidTail, buffer, len1);
        memcpy( pHead, buffer + len1, len2);
        pValidTail = pHead + len2;//新的有效数据区结尾指针
    }else
    {
        memcpy( pValidTail, buffer, addLen);
        pValidTail += addLen;//新的有效数据区结尾指针
    }
 
    //需重新计算已使用区的起始位置
    if(validLen + addLen > BUFFER_SIZE)
    {
        int moveLen = validLen + addLen - BUFFER_SIZE;//有效指针将要移动的长度
        if(pValid + moveLen > pTail)//需要分成两段计算
        {
            int len1 = pTail - pValid;
            int len2 = moveLen - len1;
            pValid = pHead + len2;
        }else
        {
            pValid = pValid + moveLen;
        }
        validLen = BUFFER_SIZE;
    }else
    {
        validLen += addLen;
    }
 
    return 0;
}
 
/*
 * function:从缓冲区内取出数据
 * param   :@buffer:接受读取数据的buffer
 *          @len:将要读取的数据的长度
 * return  :-1:没有初始化
 *          >0:实际读取的长度
 * */
int readRingbuffer(uint8_t* buffer,uint32_t len)
{
    if(pHead==NULL) return -1;
 
  // UNUSED(buffer);
 
    if(validLen ==0) return 0;
 
    if( len > validLen) len = validLen;
 
    if(pValid + len > pTail)//需要分成两段copy
    {
        int len1 = pTail - pValid;
        int len2 = len - len1;
        memcpy( buffer, pValid, len1);//第一段
        memcpy( buffer+len1, pHead, len2);//第二段，绕到整个存储区的开头
        pValid = pHead + len2;//更新已使用缓冲区的起始
    }else
    {
        memcpy( buffer, pValid, len);
        pValid = pValid +len;//更新已使用缓冲区的起始
    }
    validLen -= len;//更新已使用缓冲区的长度
 
    return len;
}
 
/*
 * function:获取已使用缓冲区的长度
 * return  :已使用的buffer长度
 * */
uint32_t getRingbufferValidLen(void)
{
    return validLen;
}
 
/*
 * function:释放环形缓冲区
 * */
void releaseRingbuffer(void)
{
    if(pHead!=NULL) free(pHead);
    pHead = NULL;
}
 

int Datasend(SendDataStruct*  DataStruct) {
	unsigned char count = 0,loop;
	unsigned char UartSendBuffer[32];
	char asciidata[10];
		//	printf("\r\n**send in**********");
		sprintf(asciidata, "%02x", DataStruct->cmd);
		UartSendBuffer[count++] = asciidata[0];
		UartSendBuffer[count++] = asciidata[1];
		sprintf(asciidata, "%lf", DataStruct->Regdata);
	  for(loop=0;loop<strlen(asciidata);loop++)
	  {
			UartSendBuffer[count++]=asciidata[loop];
	  }
		DataStruct->crc=SendCRC16(UartSendBuffer,count);
		sprintf(asciidata, "%02x", (DataStruct->crc >> 8) & 0xff);
		UartSendBuffer[count++] = asciidata[0];
		UartSendBuffer[count++] = asciidata[1];
		sprintf(asciidata, "%02x", DataStruct->crc & 0xff);
		UartSendBuffer[count++] = asciidata[0];
		UartSendBuffer[count++] = asciidata[1];
		UartSendBuffer[count++] = '\r';
		UartSendBuffer[count++] = '\n';
		HAL_StatusTypeDef status = USART1_Data_Send(UartSendBuffer,count);
	  if (status != HAL_OK) return -1;
			//if(iStatus!=count)
	    //  close(SOCK_TCPS);

	return 0;
	}

//void lock_for_cmd()
//{
//}
unsigned char uart_data_process(void)
{
  int count = 0, Hex,ret;
	unsigned char *tmp, *endpoint, *startdatapoint,i;
	static unsigned char strdata[1024],data[32];
	unsigned int DataLength;
	CmdNumCount=0;
	DataLength=getRingbufferValidLen();
	if(DataLength<=0)
		     return 0;
	ret=readRingbuffer(data,31);
	printf("DataLength=%d\r\n",ret);
	data[ret]=0;
	strncat(strdata, data, ret);
	tmp = strdata;
	startdatapoint = strdata;
	endpoint = startdatapoint;
	//if(DataParserFlag==0)
	//	return CmdNumCount;
	
	while (*startdatapoint) {  //printf("startdatapoint=%x\n",*startdatapoint);
                    if ((*(startdatapoint) == '\r')&& (*(startdatapoint + 1) == '\n')) {
                            if ((CmdNumCount < CMDLISTNUM)) {
                                    for(i=0;i<startdatapoint-endpoint;i+=2)
			                              {  Hex = 0;
			                                 sscanf((char *) (endpoint+i), "%02x", &Hex);
                                       CmdRecvBuffer[CmdNumCount][count] = Hex;
			                                 count++;
                                     }
				                     }
             printf("CmdRecvBuffer[CmdNumCount]=%x\r\n",CmdRecvBuffer[CmdNumCount][0]);       
			       CmdNumCount++;
			       startdatapoint += 2;
			       endpoint = startdatapoint;
			       tmp = startdatapoint;
			       count = 0;
		         }
            else 
               startdatapoint ++;
	}
	if (endpoint != startdatapoint) {
		memcpy(strdata, endpoint, startdatapoint - endpoint);
	}
	memset(strdata + (startdatapoint - endpoint), 0,
			strlen(strdata) - (startdatapoint - endpoint));
	if (CmdNumCount == 0) {
		memset(CmdRecvBuffer, 0x00, sizeof(CmdRecvBuffer));
	}
	printf("CmdNumCount=%d\r\n",CmdNumCount);
	return CmdNumCount;
}
double read_Voltage_val(ad717x_dev *adc_4112_dev,uint8_t chanel)
{
		uint32_t ret_data,timeout=0;
		int32_t ret;
	  ad717x_st_reg *reg_ret;
	  if(!V_open)return -1;
		do{
				timeout++;
				ret = AD4112_ReadData_ByChannel(adc_4112_dev, &ret_data, chanel);
				if(timeout>=TIMEOOT_COUNT)break;
				if (ret < 0) continue;
						
				ret = AD717X_ReadRegister(adc_4112_dev, AD717X_STATUS_REG);
				if (ret < 0)continue;
				reg_ret = AD717X_GetReg(adc_4112_dev, AD717X_STATUS_REG);
				break;
				}while(1);
		if(timeout<TIMEOOT_COUNT)
		{
				return AD4112_Code_To_Voltage(adc_4112_dev,chanel, ret_data);

		 }else{
			return -1;
		}
}
double read_Ampere_val(ad717x_dev *adc_4112_dev,uint8_t chanel)
{
		uint32_t ret_data,timeout=0;
		int32_t ret;
	  ad717x_st_reg *reg_ret;
	  if(!I_open)return -1;
		do{
				timeout++;
				ret = AD4112_ReadData_ByChannel(adc_4112_dev, &ret_data, chanel);
				if(timeout>=TIMEOOT_COUNT)break;
				if (ret < 0) continue;
						
				ret = AD717X_ReadRegister(adc_4112_dev, AD717X_STATUS_REG);
				if (ret < 0)continue;
				reg_ret = AD717X_GetReg(adc_4112_dev, AD717X_STATUS_REG);
				break;
				}while(1);
		if(timeout<TIMEOOT_COUNT)
		{
				return AD4112_Code_To_Ampere(adc_4112_dev,chanel, ret_data);

		 }else{
			return -1;
		}
}

static void switchmode(uint8_t mode)
{ 
	uint32_t value;
	ad717x_st_reg *reg_ret;
	if(mode)//电流
	{printf("set I\r\n");
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xf) | AD717X_CHMAP_REG_AINNEG(0x8)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xe) | AD717X_CHMAP_REG_AINNEG(0x9)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP2_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xd) | AD717X_CHMAP_REG_AINNEG(0xa)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP3_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xc) | AD717X_CHMAP_REG_AINNEG(0xb)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP4_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP5_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP6_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP7_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xf) | AD717X_CHMAP_REG_AINNEG(0x8)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(1) | AD717X_CHMAP_REG_AINPOS(0xe) | AD717X_CHMAP_REG_AINNEG(0x9)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP2_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(2) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP3_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(3) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP4_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP5_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP6_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP7_REG, (0 | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		I_open=1;
		V_open=0;
	}else//电压
	{printf("set V\r\n");
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(0) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(1) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP2_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(2) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP3_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(3) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP4_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP5_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP6_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev1,AD717X_CHMAP7_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP0_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(0) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP1_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(1) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP2_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(2) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP3_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(3) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP4_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(4) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP5_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(5) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP6_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(6) | AD717X_CHMAP_REG_AINNEG(16)));
		AD4112_set_param(adc_4112_dev2,AD717X_CHMAP7_REG, (AD717X_CHMAP_REG_CH_EN | AD717X_CHMAP_REG_SETUP_SEL(0) | AD717X_CHMAP_REG_AINPOS(7) | AD717X_CHMAP_REG_AINNEG(16)));
		I_open=0;
		V_open=1;
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  AD4112_remove(adc_4112_dev1);
	AD4112_remove(adc_4112_dev2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	AD4112_init(&adc_4112_dev1,1);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	AD4112_init(&adc_4112_dev2,2);
	
		AD717X_ReadRegister(adc_4112_dev1, AD717X_ID_REG);
				
		reg_ret = AD717X_GetReg(adc_4112_dev1, AD717X_ID_REG);
	  printf("adc id 0x%x\r\n", reg_ret->value);
		AD717X_ReadRegister(adc_4112_dev2, AD717X_ID_REG);
				
		reg_ret = AD717X_GetReg(adc_4112_dev2, AD717X_ID_REG);
	  printf("adc id 0x%x\r\n", reg_ret->value);
}

void cmd_run()
{
	SendDataStruct  DataStruct;
  uint8_t i,j;
  double value;

	
	for(i=0;i<CmdNumCount;i++)
	{ 
		printf("recvcmd =%x\r\n",CmdRecvBuffer[i][0]);
		  /****************************************************
     cmd{[1B]:           
     0x31 Vol1 ........    0x36 VOL6
     0x51 AM1  ......      0x56 AM6
     0xC0 set
     0x91 DA1 ......  0x92 DA2 
     data[4B]
    
    ****************************************************/
		 DataStruct.cmd=CmdRecvBuffer[i][0];
			  switch(DataStruct.cmd)
			  {
				  case 0x30:
					case 0x31:
				  case 0x32:
				  case 0x33:
				  case 0x34:
				  case 0x35:
				  case 0x36:
					case 0x37:
            value=read_Voltage_val(adc_4112_dev1,(DataStruct.cmd&0xf));
						if(value!=-1)
						 {
							DataStruct.Regdata = value;
							Datasend(&DataStruct);
						 }else{
							DataStruct.Regdata = 255.0;
							Datasend(&DataStruct);
						 }
				    break;
					case 0x38:
				  case 0x39:
				  case 0x3a:
				  case 0x3b:
				  case 0x3c:
				  case 0x3d:
					case 0x3e:
					case 0x3f:
            value=read_Voltage_val(adc_4112_dev2,(DataStruct.cmd&0xf)-8);
						if(value!=-1)
						 {
							DataStruct.Regdata = value;
							Datasend(&DataStruct);
						 }else{
							DataStruct.Regdata = 255.0;
							Datasend(&DataStruct);
						 }
				    break;
				  case 0x50:
				  case 0x51:
				  case 0x52:
				  case 0x53:
				  case 0x54:
				  case 0x55:
            if((DataStruct.cmd&0xf)<4)				
              value=read_Ampere_val(adc_4112_dev1,(DataStruct.cmd&0xf));//0,1 2,3 四路电流
						else
							value=read_Ampere_val(adc_4112_dev2,(DataStruct.cmd&0xf)-4);//0,1 两路路电流
						if(value!=-1)
						 {
							DataStruct.Regdata = value;
							Datasend(&DataStruct);
						 }else{
							DataStruct.Regdata = 255.0;
							Datasend(&DataStruct);
						 }
				   break;
				  case 0x90:
				  case 0x91:
						break;
				  case 0xc0:
						switchmode(CmdRecvBuffer[i][1]);
						break;
				 default:
					break;
			    }
		   }
}
void app_start(void)
{
	printf("app starting...\r\n");
	
	//flag_modbus_cmd = 0;
	//rs485_receive_dma(rx_buffer, 12);
	
	//while (1);
	initRingbuffer();
	
	AD4112_init(&adc_4112_dev1,1);
	AD4112_init(&adc_4112_dev2,2);
	I_open=0;
	V_open=0;
}


/*
static void ad4112_0x10_func(uint8_t c_mask)
{
	uint8_t i, channel_num = 0, channel_single = 0, CHMAP_key, FILTCON_key, is_open = 0;
	uint32_t value;
	
	value = AD4112_get_param(AD717X_ADCMODE_REG);
	value &= ~(0x01 << 13);
	AD4112_set_param(AD717X_ADCMODE_REG, value);
	// printf("a 0x%02x:0x%04x \r\n", AD717X_ADCMODE_REG, AD4112_get_param(AD717X_ADCMODE_REG));
	
	for (i = 0;i < 8;i++)
	{
		CHMAP_key 	= AD717X_CHMAP0_REG + i;
		FILTCON_key = AD717X_FILTCON0_REG + i;
		
		// 開啓指定通道
		is_open = ((c_mask >> i) & 0x01);
		value = AD4112_get_param(CHMAP_key);
		value &= ~(0x01 << 15); //清除該位
		value |= (is_open?AD717X_CHMAP_REG_CH_EN:AD717X_CHMAP_REG_CH_DIS);
		AD4112_set_param(CHMAP_key, value);
		
		// 31.25k/sps
		value = AD4112_get_param(FILTCON_key);
		value &= ~(0x1F << 0);
		value |= AD717X_FILT_CONF_REG_ODR(0);
		AD4112_set_param(FILTCON_key, value);
		
		if (is_open) channel_num++;
		if (channel_num == 1 && is_open) channel_single = i;
		printf("b 0x%02x:0x%04x 0x%02x:0x%04x \r\n", CHMAP_key, AD4112_get_param(CHMAP_key), FILTCON_key, AD4112_get_param(FILTCON_key));
	}
	
	if (channel_num == 1)
	{
		value = AD4112_get_param(AD717X_ADCMODE_REG);
		value &= ~(0x01 << 13);
		value |= AD717X_ADCMODE_SING_CYC;
		AD4112_set_param(AD717X_ADCMODE_REG, value);
		
		// 10.417k/sps
		value = AD4112_get_param(AD717X_FILTCON0_REG + channel_single);
		value &= ~(0x1F << 0);
		value |= AD717X_FILT_CONF_REG_ODR(7);
		AD4112_set_param(AD717X_FILTCON0_REG + channel_single, value);
		// printf("c 0x%02x:0x%04x 0x%02x:0x%04x \r\n", AD717X_ADCMODE_REG, AD4112_get_param(AD717X_ADCMODE_REG), AD717X_FILTCON0_REG + channel_single, AD4112_get_param(AD717X_FILTCON0_REG + channel_single));
	}
}
*/



