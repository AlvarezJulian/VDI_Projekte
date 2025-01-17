/*
 * flashMemory.h
 *
 *  Created on: Nov 12, 2024
 *      Author: Testrechner
 */

#ifndef INC_FLASHMEMORY_H_
#define INC_FLASHMEMORY_H_

#include "main.h"

//#define ADDRESS 0x0807F800 // Stm32F303RE , Page 255
#define ADDRESS 0x0801F800 // Stm32G0B1KBTX, Page 63 //Table 9. Flash memory organization for single-bank devices
#define PAGE 63 // 128 Kbyte devices


void writedata(uint32_t StartPageAddress, uint32_t Data);

void readdata(uint32_t StartPageAddress, uint32_t *RxBuf);

#endif /* INC_FLASHMEMORY_H_ */
