/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "can.h"
#include "flashMemory.h"
#include "ffz.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern bool timerCyc_100;
extern bool timerCyc_1000;
extern bool msg_701_HB;
extern FFZ_Objects FFZ;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EBZ_IN_Pin GPIO_PIN_8
#define EBZ_IN_GPIO_Port GPIOA
#define TEST_LED_Pin GPIO_PIN_10
#define TEST_LED_GPIO_Port GPIOA
#define CRASH_REL_Pin GPIO_PIN_5
#define CRASH_REL_GPIO_Port GPIOB
#define LOG_REL_Pin GPIO_PIN_6
#define LOG_REL_GPIO_Port GPIOB
#define DriverPresent_P_Pin GPIO_PIN_7
#define DriverPresent_P_GPIO_Port GPIOB
#define DriverPresent_M_Pin GPIO_PIN_8
#define DriverPresent_M_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void readParams();
void setParams();
uint32_t EBZ_Init();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
