////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Create by AnKun on 2019/10/10
 */

#include "flashapp.h"
 
static uint16_t FlashBuffer[STM32FLASH_PAGE_SIZE >> 1];
static uint32_t FLASH_WriteNotCheck(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite);

/// ��ʼ��FLASH
void FLASH_Init(void)
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | \
	                       FLASH_FLAG_PGERR | \
	                       FLASH_FLAG_WRPERR);
	HAL_FLASH_Lock();
}

/**
 * ��FLASH
 * @param  Address   Ҫ������ʼ��ַ��������Ҫ��2�ֽڶ��룡����
 * @param  Buffer    ��Ŷ�ȡ�����ݣ�������Ҫ��2�ֽڶ��룡����
 * @param  NumToRead Ҫ��ȡ������������λ�����֣�������Ҫ��2�ֽڶ��룡����
 * @return           ʵ�ʶ�ȡ��������������λ���ֽ�
 */
uint32_t FLASH_Read(uint32_t Address, uint16_t *Buffer, uint32_t NumToRead)
{
	uint32_t nread = NumToRead;
	uint32_t AddrMax = STM32FLASH_END - 2;

	if (NumToRead == 0 || Buffer == NULL || Address > AddrMax)
		return 0;

	while (Address <= AddrMax && nread)
	{
		*Buffer++ = *(__IO uint16_t *)Address;
		Address += 2;
		nread--;
	}

	return ((NumToRead - nread) << 1);
}

/**
 * дFLASH
 * @param  Address    д����ʼ��ַ��������Ҫ��2�ֽڶ��룡����
 * @param  Buffer     ��д������ݣ�������Ҫ��2�ֽڶ��룡����
 * @param  NumToWrite Ҫд�������������λ�����֣�������Ҫ��2�ֽڶ��룡����
 * @return            ʵ��д�������������λ���ֽ�
 */
uint32_t FLASH_Write(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite)
{
	uint32_t i = 0;
	uint32_t pagepos = 0;         // ҳλ��
	uint32_t pageoff = 0;         // ҳ��ƫ�Ƶ�ַ
	uint32_t pagefre = 0;         // ҳ�ڿ���ռ�
	uint32_t offset = 0;          // Address��FLASH�е�ƫ��
	uint32_t nwrite = NumToWrite; // ��¼ʣ��Ҫд���������

	/* �Ƿ���ַ */
	if (Address < STM32FLASH_BASE || Address > (STM32FLASH_END - 2) || NumToWrite == 0 || Buffer == NULL)
		return 0;

	/* ����FLASH */
	HAL_FLASH_Unlock();

	/* ����ƫ�Ƶ�ַ */
	offset = Address - STM32FLASH_BASE;

	/* ���㵱ǰҳλ�� */
	pagepos = offset / STM32FLASH_PAGE_SIZE;

	/* ����Ҫд���ݵ���ʼ��ַ�ڵ�ǰҳ�ڵ�ƫ�Ƶ�ַ */
	pageoff = ((offset % STM32FLASH_PAGE_SIZE) >> 1);

	/* ���㵱ǰҳ�ڿ���ռ� */
	pagefre = ((STM32FLASH_PAGE_SIZE >> 1) - pageoff);

	/* Ҫд������������ڵ�ǰҳ������ */
	if (nwrite <= pagefre)
		pagefre = nwrite;

	while (nwrite != 0)
	{
		/* ����Ƿ�ҳ */
		if (pagepos >= STM32FLASH_PAGE_NUM)
			break;

		/* ��ȡһҳ */
		FLASH_Read(STM32FLASH_BASE + pagepos * STM32FLASH_PAGE_SIZE, FlashBuffer, STM32FLASH_PAGE_SIZE >> 1);

		/* ����Ƿ���Ҫ���� */
		for (i = 0; i < pagefre; i++)
		{
			if (*(FlashBuffer + pageoff + i) != 0xFFFF) /* FLASH������Ĭ������ȫΪ0xFF */
				break;
		}

		if (i < pagefre)
		{
			uint32_t count = 0;
			uint32_t index = 0;
			uint32_t PageError = 0;
			FLASH_EraseInitTypeDef pEraseInit;

			/* ����һҳ */
			pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
			pEraseInit.PageAddress = STM32FLASH_BASE + pagepos * STM32FLASH_PAGE_SIZE;
			pEraseInit.Banks = FLASH_BANK_1;
			pEraseInit.NbPages = 1;
			if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK)
				break;

			/* ���Ƶ����� */
			for (index = 0; index < pagefre; index++)
			{
				*(FlashBuffer + pageoff + index) = *(Buffer + index);
			}

			/* д��FLASH */
			count = FLASH_WriteNotCheck(STM32FLASH_BASE + pagepos * STM32FLASH_PAGE_SIZE, FlashBuffer, STM32FLASH_PAGE_SIZE >> 1);
			if (count != (STM32FLASH_PAGE_SIZE >> 1))
			{
				nwrite -= count;
				break;
			}
		}
		else
		{
			/* ���������ֱ��д */
			uint32_t count = FLASH_WriteNotCheck(Address, Buffer, pagefre);
			if (count != pagefre)
			{
				nwrite -= count;
				break;
			}
		}

		Buffer += pagefre;         /* ��ȡ��ַ����         */
		Address += (pagefre << 1); /* д���ַ����         */
		nwrite -= pagefre;         /* ����ʣ��δд�������� */

		pagepos++;   /* ��һҳ               */
		pageoff = 0; /* ҳ��ƫ�Ƶ�ַ����     */

		/* ����ʣ���������´�д�������� */
		pagefre = nwrite >= (STM32FLASH_PAGE_SIZE >> 1) ? (STM32FLASH_PAGE_SIZE >> 1) : nwrite;
	}

	/* ����FLASH */
	HAL_FLASH_Lock();

	return ((NumToWrite - nwrite) << 1);
}

static uint32_t FLASH_WriteNotCheck(uint32_t Address, const uint16_t *Buffer, uint32_t NumToWrite)
{
	uint32_t nwrite = NumToWrite;
	uint32_t addrmax = STM32FLASH_END - 2;

	while (nwrite)
	{
		if (Address > addrmax)
			break;

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, *Buffer);
		if ( (*(__IO uint16_t *)Address) != *Buffer )
			break;

		nwrite--;
		Buffer++;
		Address += 2;
	}
	return (NumToWrite - nwrite);
}


//////////////////////////////////////////////////////// end of file ////////////////////////////////////////////////////////////////
