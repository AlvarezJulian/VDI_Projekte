/*
 * flashMemory.c
 *
 *  Created on: Nov 12, 2024
 *      Author: Testrechner
 */

#include "flashMemory.h"

void writedata(uint32_t StartPageAddress, uint32_t Data) {
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;

	HAL_FLASH_Unlock();
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.Page = PAGE;
	EraseInitStruct.NbPages = 1;

	/* Erase the user Flash area*/
//	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);


	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	    {
	        HAL_FLASH_GetError();
	        HAL_FLASH_Lock();
	        return;
	    }
	/* Program the user Flash area word by word*/
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data);
	HAL_FLASH_Lock();

}

void readdata(uint32_t StartPageAddress, uint32_t *RxBuf) {

	*RxBuf = *(__IO uint32_t*) StartPageAddress;
}

