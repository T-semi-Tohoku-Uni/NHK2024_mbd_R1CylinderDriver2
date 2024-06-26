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

//単体で動かすときには苗アー�?との衝突に十�?気を付けること
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "R1CANIDList.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CATCH 0
#define READY 1

#define UNEXPAND 1 // アーム格納
#define EXPAND 0 // アーム展開

#define DELAY 800

#define SHOOT 1
#define BACK 0

#define FALSE 0
#define TRUE 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
uint8_t isCatching = TRUE;

FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void Hand_Catch(uint8_t hand_state);
void Arm_Expander(uint8_t UE);
void Shoot(uint8_t isShoot);
void Hand_Ready(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Arm_Expander(uint8_t UE){
	if(UE == EXPAND){
		HAL_GPIO_WritePin(CYL_ARM_XP_GPIO_Port, CYL_ARM_XP_Pin, GPIO_PIN_SET);
	}
	else if(UE == UNEXPAND){
		HAL_GPIO_WritePin(CYL_ARM_XP_GPIO_Port, CYL_ARM_XP_Pin, GPIO_PIN_RESET);
	}
}

void Shoot(uint8_t isShoot){
	//ハンドが想定して�?な�?位置のまま�?填機構を稼働しな�?ためのコー�?
//	if(isCatching == FALSE){
////		Hand_Catch();
////		HAL_Delay(DELAY);
//	    printf("FALSE");
//	    return;
//	}

	if (isShoot == SHOOT) {
	    printf("SHOOT\r\n");
	    Arm_Expander(UNEXPAND);
	    Hand_Catch(CATCH);
	    HAL_GPIO_WritePin(CYL_SET_GPIO_Port, CYL_SET_Pin, GPIO_PIN_SET);
	} else if (isShoot == BACK) {
	    printf("BACK\r\n");
	    Arm_Expander(UNEXPAND);
	    Hand_Catch(CATCH);
	    HAL_GPIO_WritePin(CYL_SET_GPIO_Port, CYL_SET_Pin, GPIO_PIN_RESET);
	} else {
	    return;
	}
//	//�?填
//	HAL_GPIO_WritePin(CYL_SET_GPIO_Port, CYL_SET_Pin, GPIO_PIN_SET);
//	HAL_Delay(DELAY);
//	//�?填機構を戻�?
//	HAL_GPIO_WritePin(CYL_SET_GPIO_Port, CYL_SET_Pin, GPIO_PIN_RESET);
//	HAL_Delay(DELAY);
}

void Hand_Catch(uint8_t hand_state){
	//ハンドを閉じ�?
  if (hand_state == READY) {
      HAL_GPIO_WritePin(CYL_HND_GPIO_Port, CYL_HND_Pin, GPIO_PIN_SET);
      isCatching = FALSE;
  } else if (hand_state == CATCH) {
      HAL_GPIO_WritePin(CYL_HND_GPIO_Port, CYL_HND_Pin, GPIO_PIN_RESET);
      isCatching = TRUE;
  }
}

//ハンドをボールキャッチ待機状態に
void Hand_Ready(void){
	//ハンドを開く
	HAL_GPIO_WritePin(CYL_HND_GPIO_Port, CYL_HND_Pin, GPIO_PIN_SET);
	HAL_Delay(DELAY);
	//ハンドを下げ�?
	HAL_GPIO_WritePin(CYL_HND_GPIO_Port, CYL_HND_Pin, GPIO_PIN_SET);
	isCatching = FALSE;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		 if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
			Error_Handler();
		}

//		 __disable_irq();

		switch(RxHeader.Identifier){
		case CANID_BALL_HAND_UNEXPAND:
		  /*
		   * 0 => Expand
		   * 1 => UnExpand
		   */
			Arm_Expander(RxData[0]);
			if (RxData[0] == 0) {
			    printf("ARM UnExpand\r\n");
			} else {
			    printf("ARM Expand\r\n");
			}
			break;

		case CANID_BALL_EJECTOR_CONTROLE:
			Shoot(RxData[0]);
			printf("Shoot %d\r\n", RxData[0]);
			break;

		case CANID_BALL_HAND_OPEN:
		  if (RxData[0] == CATCH) {
		      printf("Catch\r\n");
		      Hand_Catch(CATCH);
		  } else if (RxData[0] == READY) {
		      printf("Ready\r\n");
		      Hand_Catch(READY);
		  } else {
		      printf("error\r\n");
		  }
			break;

		case CANID_CHECK_IS_ACTIVE:
      if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
      {
        Error_Handler();
      }
      break;

		default:
			break;
		}
//		 __enable_irq();
	}
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&hlpuart1,(uint8_t *)ptr,len,10);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);


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
  MX_FDCAN1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
//  FDCAN_FilterTypeDef sFilterConfig;
//   	sFilterConfig.IdType = FDCAN_STANDARD_ID;
//   	sFilterConfig.FilterIndex = 0;
//   	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
//   	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//   	sFilterConfig.FilterID1 = CANID_SHOOT;
//   	sFilterConfig.FilterID2 = 0b11111111100;
//
//   	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
//   			Error_Handler();
//   		}
//   		if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
//   			Error_Handler();
//   		}
//
//   		if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
//   			Error_Handler();
//   		}
//
//   		if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
//   			Error_Handler();
//   		}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_GPIO_WritePin(CYL_HND_GPIO_Port, CYL_HND_Pin, GPIO_PIN_SET);
//  HAL_Delay(DELAY);
//  HAL_GPIO_WritePin(CYL_SET_GPIO_Port, CYL_SET_Pin, GPIO_PIN_SET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  FDCAN1_TxHeader.Identifier = 0x000;
  FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;
  FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_1;
  FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  FDCAN1_TxHeader.FDFormat = FDCAN_FD_CAN;
  FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  FDCAN1_TxHeader.MessageMarker = 0;

  FDCAN_FilterTypeDef FDCAN1_sFilterConfig;
  FDCAN1_sFilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_sFilterConfig.FilterIndex = 0;
  FDCAN1_sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  FDCAN1_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_sFilterConfig.FilterID1 = 0x00;
  FDCAN1_sFilterConfig.FilterID2 = 0x7ff;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
      HAL_OK) {
      Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
      Error_Handler();
  }


  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 999;
  hiwdg.Init.Reload = 999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CYL_ARM_XP_Pin|CYL_HND_Pin|CYL_SET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BoardLED_GPIO_Port, BoardLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CYL_ARM_XP_Pin CYL_HND_Pin CYL_SET_Pin */
  GPIO_InitStruct.Pin = CYL_ARM_XP_Pin|CYL_HND_Pin|CYL_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BoardLED_Pin */
  GPIO_InitStruct.Pin = BoardLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BoardLED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_WritePin(BoardLED_GPIO_Port, BoardLED_Pin, GPIO_PIN_SET);
  __disable_irq();
  while (1)
  {
//      printf("Error Handler\r\n");
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
