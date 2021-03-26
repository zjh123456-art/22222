////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Create by AnKun on 2019/10/10
 */

#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"
//////////////////////////////////////////// ��ֲ�޸��� ///////////////////////////////////////////////

/* FLASH��С��256K */
#define STM32FLASH_SIZE  0x00040000UL

/* FLASH��ʼ��ַ */
#define STM32FLASH_BASE  FLASH_BASE  //0x08000000UL

/* FLASH������ַ */
#define STM32FLASH_END   (STM32FLASH_BASE | STM32FLASH_SIZE)

/* FLASHҳ��С��2K */
#define STM32FLASH_PAGE_SIZE FLASH_PAGE_SIZE //2K

/* FLASH��ҳ�� */
#define STM32FLASH_PAGE_NUM  (STM32FLASH_SIZE / STM32FLASH_PAGE_SIZE)

/* ��ȡҳ��ַ��X=0~STM32FLASH_PAGE_NUM ��0x08030000��ʼд 64K*/
#define ADDR_FLASH_PAGE_X(X)    (0x08030000 | (X * STM32FLASH_PAGE_SIZE))


/////////////////////////////////////////// ������������ ////////////////////////////////////////////
//void FLASH_Init(void);
uint32_t FLASH_Read(uint32_t Address, uint16_t *Buffer, uint32_t NumToRead);
uint32_t FLASH_Write(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite);

#endif // !__FLASH_H

//////////////////////////////////////////////////////// end of file ////////////////////////////////////////////////////////////////
