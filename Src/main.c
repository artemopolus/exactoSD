/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SDcardFun.h"
#include "ExSuppFun.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C_mode
#define I2C_mode_read
#define UART_mode

#define SD_mode
#define Led_mode

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DELAY_100HZ	10
#define DELAY_1HZ	1000
#define I2C_RECEIVE_CNT	(uint16_t)380
#define I2C_RECEIVE_TMT	10
#define I2C_TRANSMIT_TMT 10
#define I2C_TRANSMIT_CNT 4

#define BTN_SILENCE_TIME	100

#define LED_EXMODE_BLNK_TIME	5
#define LED_EXMODE_SLOW_TIME 5000
#define LED_EXMODE_MEDI_TIME 1000
#define LED_EXMODE_FAST_TIME 200


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


__IO uint32_t BTN_LastClckHappen;
__IO uint32_t LED_EXMODE_LstEVTm;



uint8_t ptI2Cbuffer2transmit[I2C_TRANSMIT_CNT] = {0};
uint8_t ptI2Cbuffer4receive[I2C_RECEIVE_CNT] = {0};

exactoSD_modes BaseMode = exactoSD_init;

__IO uint8_t flgBtnPress = 0;
__IO uint8_t TargetI2Cdevice = 0xff;

#ifdef SD_mode
SDcardFile_HandleTypeDef sdcfhtd;
				  //01234567890123456789
#endif
				  //0123456789012345678
TCHAR FileName[] = "/dtss/ss0000.txt";
TCHAR DirrName[] = "/dtss";
uint8_t BufferTMP[10];
__IO uint8_t I2Cflag = 0x00;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

#ifdef I2C_mode
static uint8_t GetExactoIMUaddress(void);
static uint8_t SetExactoIMUmode(uint8_t mode);
static uint8_t GetExactoIMUdata(void);
#endif

#ifdef Led_mode
static void LedSignalOn(void);
#endif

#ifdef SD_mode
static void InitNewSession(void);
static void CloseSession(void);
#endif

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

#ifdef I2C_mode
	ptI2Cbuffer2transmit[1] = 2;
#endif

//#ifdef SD_mode
//	UINT I2CgetDTcnt;
//#endif

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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  BaseMode = exactoSD_erro;
  BTN_LastClckHappen = HAL_GetTick();
  LED_EXMODE_LstEVTm = HAL_GetTick();

#ifdef I2C_mode
//  uint16_t possibleIndex;
//  for ( possibleIndex = 0x01; possibleIndex < 127; possibleIndex++)
//  {
//	  if(HAL_I2C_IsDeviceReady(&hi2c2,(possibleIndex<<1),1,100) == HAL_OK)
//	  {
//		  TargetI2Cdevice = possibleIndex;
//	  }
//	  else
//		  __NOP();
//  }
//  if(TargetI2Cdevice != 0xff)
//  {
//	  __NOP();
//  }
  TargetI2Cdevice = GetExactoIMUaddress();
#endif


#ifdef SD_mode
  if(SDcardSelfTest(&sdcfhtd) == SDcard_success)
  {


//	  SDcardOpenDir(&sdcfhtd, "data");
//	  SDcardOpenFile2write(&sdcfhtd, "data/newfile2.txt");

//	  UINT getmsglen;
//	  SDcardWrite2file(&sdcfhtd, (uint8_t*)"hello", (UINT)5, &getmsglen);
//	  SDcardCloseFile(&sdcfhtd);
#endif
#ifdef I2C_mode
	  if(TargetI2Cdevice != 0xff)
	  {
#endif
#ifdef SD_mode
		  BaseMode = exactoSD_wait;
		  //check file names
//		  uint16_t iterator = 1;
//		  SDcardOpenDir(&sdcfhtd, "dtss");
//
//		  while(SDcardTryOpen(&sdcfhtd, FileName) == SDcard_success)
//		  {
//			  uint8_t order = Dec_Convert(&BufferTMP[0],(int)iterator++);
//			  for(uint8_t i = 0; i < order; i++)	FileName[11 - order + i] = BufferTMP[10 - order + i];
//		  }
////		  TCHAR trgpath_tmp[] = "dtss/ss0000.txt";
//		  SDcardOpenFile2write(&sdcfhtd, FileName);
//		  SDcardWrite2file(&sdcfhtd, (uint8_t*)"new session\n", (UINT)sizeof("new session\n"), &getmsglen);
//
//		  uint8_t i = 0;
//		  while(i++ < 1)
//		  {
//			  ptI2Cbuffer2transmit[3] = 4;
//			  HAL_I2C_Master_Transmit(&hi2c2, (TargetI2Cdevice<<1), ptI2Cbuffer2transmit, 4, 10);
//			  HAL_Delay(100);
//			  uint8_t j = 0;
//			  while(j++ < 10)
//			  {
//					  HAL_I2C_Master_Receive(&hi2c2, (TargetI2Cdevice<<1), ptI2Cbuffer4receive, I2C_RECEIVE_CNT, I2C_RECEIVE_TMT);
//					HAL_Delay(10);
//			  }
//			  HAL_Delay(100);
//			  ptI2Cbuffer2transmit[3] = 0;
//			  HAL_I2C_Master_Transmit(&hi2c2, (TargetI2Cdevice<<1), ptI2Cbuffer2transmit, 4, 10);
//		  }
//
//		  SDcardCloseFile(&sdcfhtd);
		  InitNewSession();
#endif
#ifdef Led_mode
		 // LedSignalOn();
#endif
#ifdef I2C_mode
	  }
#endif
#ifdef SD_mode
	  __NOP();
  }
#endif
  __NOP();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (BaseMode)
		{
	  case exactoSD_init:
		  BlinkLed(50);
		  HAL_Delay(4950);
		  break;
	  case exactoSD_erro:
		  BlinkLed(50);
		  HAL_Delay(50);
		  BlinkLed(50);
		  HAL_Delay(4850);
		  break;
	  case exactoSD_wait:
		  if(flgBtnPress)
		  {
			  flgBtnPress = 0;
#ifdef SD_mode
			  CloseSession();
			  InitNewSession();
#endif
#ifdef I2C_mode_read
			  SetExactoIMUmode(0);
#endif
		  }
		  if((HAL_GetTick() - LED_EXMODE_LstEVTm) > LED_EXMODE_MEDI_TIME)
		  {
			  LED_EXMODE_LstEVTm = HAL_GetTick();
			  BlinkLed(LED_EXMODE_BLNK_TIME);
			  HAL_Delay(50);
			  BlinkLed(LED_EXMODE_BLNK_TIME);
			  HAL_Delay(LED_EXMODE_MEDI_TIME - 50 - 2*LED_EXMODE_BLNK_TIME);
		  }
		  else
		  {
			  HAL_Delay(LED_EXMODE_MEDI_TIME);
		  }
		  break;
	  case exactoSD_meas:
		  if(flgBtnPress)
		  {
			  flgBtnPress = 0;
#ifdef I2C_mode_read
			  SetExactoIMUmode(1);
#endif
		  }
		  else
		  {
#ifdef I2C_mode_read
			  GetExactoIMUdata();
#endif
		  }
		  if((HAL_GetTick() - LED_EXMODE_LstEVTm) > LED_EXMODE_FAST_TIME)
		  {
			  LED_EXMODE_LstEVTm = HAL_GetTick();
			  BlinkLed(LED_EXMODE_BLNK_TIME);
			  HAL_Delay(10 - LED_EXMODE_BLNK_TIME);
		  }
		  else
		  {
			  HAL_Delay(10);
		  }
		  break;
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */
  //BSP_SD_Init();
  //HAL_SD_MspInit(&hsd1);
  //BSP_SD_Init();
  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UsrBtn_Pin */
  GPIO_InitStruct.Pin = UsrBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UsrBtn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void BlinkLed(uint32_t delay)
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
	HAL_Delay(delay);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
}
void BtnPress_Callback(void)
{
	if((HAL_GetTick() - BTN_LastClckHappen) > BTN_SILENCE_TIME)
	{
	flgBtnPress = 1;
	BTN_LastClckHappen = HAL_GetTick();
	switch (BaseMode)
	{
		case exactoSD_wait:
			BaseMode = exactoSD_meas;
			break;
		case exactoSD_meas:
			BaseMode = exactoSD_wait;
			break;
		case exactoSD_erro:
		case exactoSD_init:
			break;
	}
	}
}
#ifdef Led_mode
static void LedSignalOn(void)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
}
#endif

#ifdef SD_mode
static uint8_t GetExactoIMUaddress(void)
{
	  uint16_t possibleIndex;
	  for ( possibleIndex = 0x01; possibleIndex < 127; possibleIndex++)
	  {
		  if(HAL_I2C_IsDeviceReady(&hi2c2,(possibleIndex<<1),1,100) == HAL_OK)
		  {
			  return possibleIndex;
		  }
		  else
			  __NOP();
	  }
	  return 0xff;
}
static void InitNewSession(void)
{
	  SDcardOpenDir(&sdcfhtd, DirrName);
	  uint16_t iterator = 1;
	  while(SDcardCountFiles(&sdcfhtd,DirrName,FileName) != 0)
	  {
		  uint8_t order = Dec_Convert(&BufferTMP[0],(int)iterator++);
		  for(uint8_t i = 0; i < order; i++)	FileName[12 - order + i] = BufferTMP[10 - order + i];
	  }
	  SDcardOpenFile2write(&sdcfhtd, FileName);
	  UINT getmsglen;
	  uint8_t initstr[] = "new session\n";
	  SDcardWrite2file(&sdcfhtd, initstr, (UINT)sizeof(initstr), &getmsglen);
}
static void CloseSession(void)
{
	SDcardCloseFile(&sdcfhtd);
}
#endif

#ifdef I2C_mode
static uint8_t SetExactoIMUmode(uint8_t mode)
{
	switch (mode)
	{
	case 0:
		ptI2Cbuffer2transmit[3] = 0;
		break;
	case 1:
		ptI2Cbuffer2transmit[3] = 4;
		break;
	}
	if(HAL_I2C_Master_Transmit(&hi2c2, (TargetI2Cdevice<<1), ptI2Cbuffer2transmit, I2C_TRANSMIT_CNT, I2C_TRANSMIT_TMT)== HAL_OK)
		__NOP();
	else
		__NOP();
	HAL_Delay(100);
	return 1;
}
static uint8_t GetExactoIMUdata(void)
{
	if(HAL_I2C_Master_Receive(&hi2c2, (TargetI2Cdevice<<1), ptI2Cbuffer4receive, I2C_RECEIVE_CNT, I2C_RECEIVE_TMT) == HAL_OK)
	{
		UINT getmsglen;
		SDcardWrite2fileln(&sdcfhtd, ptI2Cbuffer4receive, (UINT)I2C_RECEIVE_CNT, &getmsglen);
	}
	else
	{
		UINT getmsglen;
		uint8_t initstr[] = "no data\n";
		SDcardWrite2file(&sdcfhtd, initstr, (UINT)sizeof(initstr), &getmsglen);
	}
	return 1;
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
