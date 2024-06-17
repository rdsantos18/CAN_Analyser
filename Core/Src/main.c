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
#include "cmsis_os.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void CAN1_Config(void);
void CAN2_Config(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CAN_TxHeaderTypeDef   TxHeader1;
CAN_RxHeaderTypeDef   RxHeader1;
HAL_StatusTypeDef HAL_Retval1 = HAL_ERROR;

CAN_TxHeaderTypeDef   TxHeader2;
CAN_RxHeaderTypeDef   RxHeader2;
HAL_StatusTypeDef HAL_Retval2 = HAL_ERROR;

uint8_t	TxData1[32] = {0};
uint8_t RxData1[32] = {0};

uint8_t	TxData2[32] = {0};
uint8_t RxData2[32] = {0};

uint32_t TxMailbox1 = 0;
uint32_t TxMailbox2 = 0;

uint8_t Flag_CAN_1 = 0;
uint8_t Flag_CAN_2 = 0;

//static uint8_t buffer[_MAX_SS]; /* a work buffer for the f_mkfs() */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern volatile unsigned long ulHighFrequencyTimerTicks;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  logI("CAN ANALYSER STM32 FreeRTOS v1.0.0 17/06/2024\n\r");

  //HAL_GPIO_WritePin(LED_CAN1_TX_GPIO_Port, LED_CAN1_TX_Pin, GPIO_PIN_SET);	// PC0
  //HAL_GPIO_WritePin(LED_CAN1_RX_GPIO_Port, LED_CAN1_RX_Pin, GPIO_PIN_SET);	// PC1
  //HAL_GPIO_WritePin(LED_CAN2_TX_GPIO_Port, LED_CAN2_TX_Pin, GPIO_PIN_SET);	// PC2
  //HAL_GPIO_WritePin(LED_CAN2_RX_GPIO_Port, LED_CAN2_RX_Pin, GPIO_PIN_SET);	// PC3
  //HAL_GPIO_WritePin(LED_485_TX_GPIO_Port, LED_485_TX_Pin, GPIO_PIN_SET);      // PB0
  //HAL_GPIO_WritePin(LED_485_RX_GPIO_Port, LED_485_RX_Pin, GPIO_PIN_SET);		// PB1

  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  CAN1_Config();
  CAN2_Config();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CAN1_Config(void)
{
	CAN_FilterTypeDef CAN1_FilterType;

	CAN1_FilterType.FilterBank=0;
	CAN1_FilterType.FilterIdHigh=0x0000;
	CAN1_FilterType.FilterIdLow=0x0000;
	CAN1_FilterType.FilterMaskIdHigh=0x0000;
	CAN1_FilterType.FilterMaskIdLow=0x0000;
	CAN1_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;
	CAN1_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN1_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN1_FilterType.FilterActivation=ENABLE;
	CAN1_FilterType.SlaveStartFilterBank=14;

	if(HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterType)!=HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan1)!=HAL_OK)
	{
		Error_Handler();
	}
}

void CAN2_Config(void)
{
	CAN_FilterTypeDef CAN2_FilterType;

	CAN2_FilterType.FilterBank=14;
	CAN2_FilterType.FilterIdHigh=0x0000;
	CAN2_FilterType.FilterIdLow=0x0000;
	CAN2_FilterType.FilterMaskIdHigh=0x0000;
	CAN2_FilterType.FilterMaskIdLow=0x0000;
	CAN2_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;
	CAN2_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN2_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN2_FilterType.FilterActivation=ENABLE;
	CAN2_FilterType.SlaveStartFilterBank=14;

	if(HAL_CAN_ConfigFilter(&hcan2, &CAN2_FilterType)!=HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan2)!=HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1) {
		HAL_Retval1 = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, RxData1);
		if(HAL_Retval1 == HAL_OK) {
			// Tem Dados Validos
			Flag_CAN_1 = 1;
			HAL_GPIO_TogglePin(LED_CAN2_RX_GPIO_Port, LED_CAN2_RX_Pin);

			TxHeader2.ExtId = RxHeader1.ExtId;
			TxHeader2.RTR = CAN_RTR_DATA;
			TxHeader2.IDE = CAN_ID_EXT;
			TxHeader2.DLC = 8;
			TxHeader2.TransmitGlobalTime = DISABLE;

			TxData2[0] = RxData1[0];
			TxData2[1] = RxData1[1];
			TxData2[2] = RxData1[2];
			TxData2[3] = RxData1[3];
			TxData2[4] = RxData1[4];
			TxData2[5] = RxData1[5];
			TxData2[6] = RxData1[6];
		    TxData2[7] = RxData1[7];
		    // Start the Transmission process
		    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2) != HAL_OK)
		    {
				// Transmission request Error
		    }
		    else {
		    }
		}
		else {
			Flag_CAN_1 = 0;
			HAL_GPIO_WritePin(LED_CAN2_RX_GPIO_Port, LED_CAN2_RX_Pin, GPIO_PIN_RESET);
		}
	}
	else if(hcan->Instance == CAN2) {
		HAL_Retval2 = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData2);
		if(HAL_Retval2 == HAL_OK) {
			// Tem Dados Validos
			Flag_CAN_2 = 1;
			HAL_GPIO_TogglePin(LED_CAN2_TX_GPIO_Port, LED_CAN2_TX_Pin);

			TxHeader1.ExtId = RxHeader2.ExtId;
			TxHeader1.RTR = CAN_RTR_DATA;
			TxHeader1.IDE = CAN_ID_EXT;
			TxHeader1.DLC = 8;
			TxHeader1.TransmitGlobalTime = DISABLE;

			TxData1[0] = RxData2[0];
			TxData1[1] = RxData2[1];
			TxData1[2] = RxData2[2];
			TxData1[3] = RxData2[3];
			TxData1[4] = RxData2[4];
			TxData1[5] = RxData2[5];
			TxData1[6] = RxData2[6];
		    TxData1[7] = RxData2[7];
		    // Start the Transmission process
		    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1) != HAL_OK)
		    {
				// Transmission request Error
		    }
		    else {
		    }
		}
		else {
			Flag_CAN_2 = 0;
			HAL_GPIO_WritePin(LED_CAN2_TX_GPIO_Port, LED_CAN2_TX_Pin, GPIO_PIN_RESET);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) {
	  ulHighFrequencyTimerTicks++;
  }
  /* USER CODE END Callback 1 */
}

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
