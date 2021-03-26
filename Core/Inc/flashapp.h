////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Create by AnKun on 2019/10/10
 */

#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"
//////////////////////////////////////////// 移植修改区 ///////////////////////////////////////////////

/* FLASH大小：256K */
#define STM32FLASH_SIZE  0x00040000UL

/* FLASH起始地址 */
#define STM32FLASH_BASE  FLASH_BASE  //0x08000000UL

/* FLASH结束地址 */
#define STM32FLASH_END   (STM32FLASH_BASE | STM32FLASH_SIZE)

/* FLASH页大小：2K */
#define STM32FLASH_PAGE_SIZE FLASH_PAGE_SIZE //2K

/* FLASH总页数 */
#define STM32FLASH_PAGE_NUM  (STM32FLASH_SIZE / STM32FLASH_PAGE_SIZE)

/* 获取页地址，X=0~STM32FLASH_PAGE_NUM 从0x08030000开始写 64K*/
#define ADDR_FLASH_PAGE_X(X)    (0x08030000 | (X * STM32FLASH_PAGE_SIZE))


/////////////////////////////////////////// 导出函数声明 ////////////////////////////////////////////
//void FLASH_Init(void);
uint32_t FLASH_Read(uint32_t Address, uint16_t *Buffer, uint32_t NumToRead);
uint32_t FLASH_Write(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite);

#endif // !__FLASH_H

//////////////////////////////////////////////////////// end of file ////////////////////////////////////////////////////////////////
