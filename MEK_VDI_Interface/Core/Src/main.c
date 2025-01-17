/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "OD.h"
#include "CANopen.h"
//#include "CO_driver_target.h"
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
FDCAN_HandleTypeDef hfdcan2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
FFZ_Objects FFZ;
uint32_t Rx_OperationTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_FDCAN2_Init();
	MX_TIM14_Init();
	MX_USB_Device_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim7);
	readdata(ADDRESS, &Rx_OperationTime);
	FFZ_Init();
//	FFZ.OperationTime = EBZ_Init();

	CANopenNodeSTM32 canOpenNodeSTM32;
	canOpenNodeSTM32.CANHandle = &hfdcan2;
	canOpenNodeSTM32.HWInitFunction = MX_FDCAN2_Init;
	canOpenNodeSTM32.timerHandle = &htim14;
	canOpenNodeSTM32.desiredNodeID = 0x15;
	canOpenNodeSTM32.baudrate = 500;
	canopen_app_init(&canOpenNodeSTM32);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		readParams();
		setParams();

		/*
		 * Process CANopen Start:
		 * Nur wenn Master ein Heartbeat sendet kann das FFZ antworten,
		 */
		if (msg_701_HB == true) {
			canopen_app_process();
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

	/* USER CODE BEGIN FDCAN2_Init 0 */

	/* USER CODE END FDCAN2_Init 0 */

	/* USER CODE BEGIN FDCAN2_Init 1 */

	/* USER CODE END FDCAN2_Init 1 */
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission = ENABLE;
	hfdcan2.Init.TransmitPause = ENABLE;
	hfdcan2.Init.ProtocolException = DISABLE;
	hfdcan2.Init.NominalPrescaler = 8;
	hfdcan2.Init.NominalSyncJumpWidth = 1;
	hfdcan2.Init.NominalTimeSeg1 = 13;
	hfdcan2.Init.NominalTimeSeg2 = 2;
	hfdcan2.Init.DataPrescaler = 8;
	hfdcan2.Init.DataSyncJumpWidth = 1;
	hfdcan2.Init.DataTimeSeg1 = 13;
	hfdcan2.Init.DataTimeSeg2 = 2;
	hfdcan2.Init.StdFiltersNbr = 0;
	hfdcan2.Init.ExtFiltersNbr = 0;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN2_Init 2 */

	/* USER CODE END FDCAN2_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 64000 - 1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 1000 - 1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 64 - 1;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 1000 - 1;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CRASH_REL_Pin | LOG_REL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : EBZ_IN_Pin */
	GPIO_InitStruct.Pin = EBZ_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(EBZ_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TEST_LED_Pin */
	GPIO_InitStruct.Pin = TEST_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TEST_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CRASH_REL_Pin LOG_REL_Pin */
	GPIO_InitStruct.Pin = CRASH_REL_Pin | LOG_REL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : DriverPresent_P_Pin DriverPresent_M_Pin */
	GPIO_InitStruct.Pin = DriverPresent_P_Pin | DriverPresent_M_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/*Configure GPIO pin Output Level */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

//	/* USER CODE END Callback 0 */
//	if (htim->Instance == TIM14) {
//		HAL_IncTick();
//	}
	/* USER CODE BEGIN Callback 1 */
// Handle CANOpen app interrupts
	if (htim == canopenNodeSTM32->timerHandle) {
//			&& OD_PERSIST_COMM.x1017_producerHeartbeatTime != 0) {
		canopen_app_interrupt();
	}
	/* USER CODE END Callback 1 */

	if ((htim->Instance == TIM7)
			&& (HAL_GPIO_ReadPin(EBZ_IN_GPIO_Port, EBZ_IN_Pin) == GPIO_PIN_SET)
			&& (FFZ.ACRQ_AcceesReq == RELEASE)
			&& (FFZ.DriverPresent == ACTIVE )) {
		FFZ.OperationTime++;
		writedata(ADDRESS, FFZ.OperationTime);
	}
}

void readParams() {

	if (HAL_GPIO_ReadPin(DriverPresent_M_GPIO_Port, DriverPresent_M_Pin)
			|| HAL_GPIO_ReadPin(DriverPresent_P_GPIO_Port,
			DriverPresent_P_Pin)) {
		FFZ.DriverPresent = ACTIVE;
		HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_SET);

	} else {
		FFZ.DriverPresent = NOT_ACTIVE;
		HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
	}

	//txPDO 1 @ 100 ms
	OD_PERSIST_COMM.x6010_DP_LA_TA = (FFZ.DriverPresent << 4)
			| (FFZ.Lift_Hydraulic_Active << 2) | FFZ.Driving_Active;
	OD_PERSIST_COMM.x6012_truckSpeed = FFZ.TruckSpeed;

	//txPDO 2 @ 1000 ms
	OD_PERSIST_COMM.x6013_operationTime = FFZ.OperationTime;
	OD_PERSIST_COMM.x6014_feature = (FFZ.F3A_TruckSpeedSignal << 2)
			| (FFZ.F2A_LiftingLimitiation << 1) | FFZ.F1A_CreepSpeed;
	OD_PERSIST_COMM.x6015_mainVersion = FFZ.MainVersion;
	OD_PERSIST_COMM.x6016_subVersion = FFZ.Subversion;

	// end of ReadParams

}

void setParams() {

	//rxPDO 1 @ 100 ms
	FFZ.LRRQ_LiftReductionReq = (OD_PERSIST_COMM.x6020_LRRQ_SRRQ_ACRQ
			& 0b00010000) >> 4;

	FFZ.SRRQ_SpeedReductionReq = (OD_PERSIST_COMM.x6020_LRRQ_SRRQ_ACRQ
			& 0b00000100) >> 2;

	FFZ.ACRQ_AcceesReq = OD_PERSIST_COMM.x6020_LRRQ_SRRQ_ACRQ & 0b00000001;

	if (FFZ.ACRQ_AcceesReq == RELEASE) {
		HAL_GPIO_WritePin(LOG_REL_GPIO_Port, LOG_REL_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LOG_REL_GPIO_Port, LOG_REL_Pin, GPIO_PIN_RESET);
	}

	if (FFZ.SRRQ_SpeedReductionReq == RELEASE) {
		HAL_GPIO_WritePin(CRASH_REL_GPIO_Port, CRASH_REL_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(CRASH_REL_GPIO_Port, CRASH_REL_Pin, GPIO_PIN_RESET);
	}
}

uint32_t EBZ_Init() {
	uint32_t ebz = 0;

	/*
	 * Wenn ein Wert im Speicher vorhanden, -> Wert übernehmen
	 *
	 * Wenn Speicher leer, -> EBZ=0
	 *
	 */

	if (Rx_OperationTime != 0) {
		ebz = Rx_OperationTime;
	}
	return ebz;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
