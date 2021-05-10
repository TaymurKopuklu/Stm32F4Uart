/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define  UartHaberUzulugu 50
#define RxStart 0xAB
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


uint8_t UartHaberlesmeBaslat=0;


uint8_t DataTx[UartHaberUzulugu];

uint8_t DataRx[UartHaberUzulugu];


uint8_t DataRxDogrulama=0;

uint8_t UartParemetre=0;

uint8_t UartHaberlesmeAdrsi=0xAB;

uint32_t TimeCount=0;

uint8_t UartCount=0;

uint32_t RealTime=0;
uint8_t ChecksumKontrol=0;
uint8_t GelenChecksum=0;
uint8_t GelenDataDogru=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
		HAL_UART_Receive_IT(&huart1, DataRx, UartHaberUzulugu);
	
	__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim7);	//50Ms
	DataTx[0]=UartHaberlesmeAdrsi;
	DataTx[1]=0x01;
	DataTx[UartHaberUzulugu-3]=0x10;
	DataTx[UartHaberUzulugu-4]=0x11;
	DataTx[UartHaberUzulugu-4]=0xF1;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim7.Init.Prescaler = 14;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 59999;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Count_Pin|Tx_Pin|Rx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TxStart_Pin */
  GPIO_InitStruct.Pin = TxStart_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TxStart_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Count_Pin Tx_Pin Rx_Pin */
  GPIO_InitStruct.Pin = Count_Pin|Tx_Pin|Rx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)	//RPi
	{

	
		HAL_UART_Receive_IT(&huart1, DataRx, UartHaberUzulugu);
		HAL_GPIO_TogglePin(GPIOG,Rx_Pin);
		ChecksumKontrol=1;
		
//		if(DataRx[UartParemetre]==0xAB)UartParemetre++;
//		else
//		{
//		UartParemetre=0;
//		}
//		if(UartParemetre>=5)UartParemetre=0;
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	//HAL_UART_Stop(&huart1);		// ?
	//dummy=huart1.Instance->RDR;
	//HAL_UART_Receive_DMA(&huart1,rx_buff,25);

  __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
  __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

  /* Check if a receive process is ongoing or not */
  if(huart->gState == HAL_UART_STATE_BUSY_TX_RX)
  {
    huart->gState = HAL_UART_STATE_BUSY_RX;
  }
  else
  {
    huart->gState = HAL_UART_STATE_READY;
  }
	//  HAL_UART_RxCpltCallback(huart);
}

unsigned checksum(uint8_t *buffer, int len)
{  
	  buffer[len-1]=0;
		for(int i=0;i<len-1;i++)
	{ 
		buffer[len-1]=buffer[i]+buffer[len-1];
	}
      return buffer[UartHaberUzulugu-1];
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);

	
	if(ChecksumKontrol==1)
	{
		ChecksumKontrol=0;
		
		 if(checksum(DataRx,UartHaberUzulugu) == DataRx[UartHaberUzulugu-1] && RxStart == DataRx[0] )
		 { 
			 GelenDataDogru=1;
		 }
	}
	
	
	 
	if(UartHaberlesmeBaslat==0&&TimeCount==5)
	{ 


	  DataTx[UartHaberUzulugu-1]=checksum(DataTx,UartHaberUzulugu);
		DataTx[UartHaberUzulugu-1]=DataTx[UartHaberUzulugu-1] & 0xFF;
		TimeCount=0;
		UartHaberlesmeBaslat=1;
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)DataTx,UartHaberUzulugu);
		HAL_GPIO_TogglePin(GPIOG,Tx_Pin);
	}
	
	 if(HAL_GPIO_ReadPin(GPIOA,TxStart_Pin)==1)
 { DataTx[4]=0xF1;
	 HAL_UART_Transmit_IT(&huart1,(uint8_t *)DataTx,5);
	 UartHaberlesmeBaslat=0;
 }
	
	TimeCount++;
	if(TimeCount>10)
		{
		TimeCount=0;
 	  UartHaberlesmeBaslat=0;
	}

	HAL_GPIO_TogglePin(GPIOG,Count_Pin);
	RealTime=TIM7->CNT;

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
  __disable_irq();
  while (1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
