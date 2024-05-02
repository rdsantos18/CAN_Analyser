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
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wizchip_conf.h"
#include "socket.h"
#include "httpServer.h"
#include "log.h"
#include "fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void cs_sel(void);
void cs_desel(void);
uint8_t spi_rb(void);
void spi_wb(uint8_t b);
void spi_rb_burst(uint8_t *buf, uint16_t len);
void spi_wb_burst(uint8_t *buf, uint16_t len);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rcvBuf[200] = {0}, sendBuf[200] = {0};
uint8_t	bufSize[] = {2, 2, 2, 2};
uint8_t WEBRX_BUF[2048];
uint8_t WEBTX_BUF[2048];
uint8_t socknumlist[] = {2, 3};
uint32_t timer_led = 0;
uint32_t timer_teste = 0;

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

RTC_TimeTypeDef gTime = {0};
RTC_DateTypeDef gDate = {0};

char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FATFS *pfs;
FRESULT fr;     /* FatFs return code */
DWORD fre_clust;
uint32_t totalSpace, freeSpace, SpaceUsed;
static uint8_t buffer[_MAX_SS]; /* a work buffer for the f_mkfs() */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_HTTPSOCK	2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  //MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
  reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);
  //reg_wizchip_cris_cbfunc(vPortEnterCritical, vPortExitCritical);

  wizchip_init(bufSize, bufSize);

  wizchip_init(bufSize, bufSize);
  wiz_NetInfo netInfo = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
		                    .ip 	= {192, 168, 10, 11},					// IP address
		                    .sn 	= {255, 255, 255, 0},					// Subnet mask
		                    .gw 	= {192, 168, 10, 250}};					// Gateway address

  wizchip_setnetinfo(&netInfo);
  wizchip_getnetinfo(&netInfo);

  httpServer_init(WEBTX_BUF, WEBRX_BUF, MAX_HTTPSOCK, socknumlist);		// Tx/Rx buffers (1kB) / The number of W5500 chip H/W sockets in use
  reg_httpServer_cbfunc(NVIC_SystemReset, NULL);

  logI("Init CAN ANALYSER STM32 v1.0.0\n\r");
  if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
	  // Mount SD CARD
	  fr = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
	  if(fr != FR_OK){
		  logI("STM32 FatFs - GetMount ERROR...\n\r");
	  }
	  else {
		  logI("STM32 FatFs - GetMount OK...\n\r");
	  }

//	  fr = f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, buffer, sizeof(buffer));
//	  if(fr != FR_OK){
//		  logI("STM32 FatFs - Mkfs ERROR...\n\r");
//	  }
//	  else {
//		  logI("STM32 FatFs - Mkfs OK...\n\r");
//	  }

	  // Check freeSpace space
	  fr = f_getfree("", &fre_clust, &pfs);
	  if(fr != FR_OK){
		  logI("STM32 FatFs - GetFree ERROR...\n\r");
	  }
	  else {
		  logI("STM32 FatFs - GetFree OK...\n\r");
		  totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		  freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
		  SpaceUsed = totalSpace - freeSpace;
		  logI("Total Space: %ld , SpeceUsed: %ld , FreeSpace: %ld \n\r", totalSpace, SpaceUsed, freeSpace);
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	  if(HAL_GetTick() - timer_led > 100) {
		  timer_led = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED_INT_GPIO_Port, LED_INT_Pin);
	  }
	  //
	  if(HAL_GetTick() - timer_teste > 1000) {
		  timer_teste = HAL_GetTick();
		  logI("CAN1 0x%08lx DATA: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  %02dh:%02dm:%02ds:%02ldmm\n\r", RxHeader1.ExtId,
			    RxData1[0], RxData1[1], RxData1[2], RxData1[3], RxData1[4], RxData1[5], RxData1[6], RxData1[7],
				gTime.Hours, gTime.Minutes, gTime.Seconds, gTime.SubSeconds);
	  }
	  // HTTP
	 // for(uint8_t i = 0; i < MAX_HTTPSOCK; i++)	{
	//	  httpServer_run(i); 	// HTTP Server handler
	//  }
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
void cs_sel(void)
{
	//HAL_GPIO_WritePin(CS_W5100_GPIO_Port, CS_W5100_Pin, GPIO_PIN_RESET); //CS LOW
}

void cs_desel(void)
{
	//HAL_GPIO_WritePin(CS_W5100_GPIO_Port, CS_W5100_Pin, GPIO_PIN_SET); //CS HIGH
}

uint8_t spi_rb(void)
{
	uint8_t rbuf;
	HAL_SPI_Receive(&hspi1, &rbuf, 1, 0xFFFFFFFF);
	return rbuf;
}

void spi_wb(uint8_t b)
{
	HAL_SPI_Transmit(&hspi1, &b, 1, 0xFFFFFFFF);
}

void spi_rb_burst(uint8_t *buf, uint16_t len)
{
	HAL_SPI_Receive_DMA(&hspi1, buf, len);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

void spi_wb_burst(uint8_t *buf, uint16_t len)
{
	HAL_SPI_Transmit_DMA(&hspi1, buf, len);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1) {
		HAL_Retval1 = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, RxData1);

		if(HAL_Retval1 == HAL_OK) {
			// Tem Dados Validos
			Flag_CAN_1 = 1;
			//
			logI("CAN1 0x%08lx MSG: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  %ld\n\r", RxHeader1.ExtId,
			      RxData1[0], RxData1[1], RxData1[2], RxData1[3], RxData1[4], RxData1[5], RxData1[6], RxData1[7], HAL_GetTick());
			//
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
		}
	}
	else if(hcan->Instance == CAN2) {
		HAL_Retval2 = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData2);

		if(HAL_Retval2 == HAL_OK) {
			// Tem Dados Validos
			Flag_CAN_2 = 1;
			//
			logI("CAN2 0x%08lx MSG: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  %ld\n\r", RxHeader2.ExtId,
			      RxData2[0], RxData2[1], RxData2[2], RxData2[3], RxData2[4], RxData2[5], RxData2[6], RxData2[7], HAL_GetTick());
			//
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
		}
	}
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
