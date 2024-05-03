/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "fatfs.h"
#include "rtc.h"
#include "log.h"
#include "usb_device.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "httpServer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern CAN_RxHeaderTypeDef   RxHeader1;
extern uint8_t RxData1[];

RTC_TimeTypeDef gTime = {0};
RTC_DateTypeDef gDate = {0};

uint32_t timer_led = 0;
uint32_t timer_teste = 0;

char line[100] = {0}; /* Line buffer */
extern char SDPath[4];   /* SD logical drive path */
FIL USERFile;       /* File object for USER */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
FATFS *pfs;
FRESULT fr;     /* FatFs return code */
DWORD fre_clust;
uint32_t totalSpace, freeSpace, SpaceUsed;
uint32_t duracao = 0;
uint32_t size = 0;
unsigned int ByteRead;
volatile unsigned long ulHighFrequencyTimerTicks = 0;

uint8_t rcvBuf[200] = {0}, sendBuf[200] = {0};
uint8_t	bufSize[] = {2, 2, 2, 2};
uint8_t WEBRX_BUF[2048];
uint8_t WEBTX_BUF[2048];
uint8_t socknumlist[] = {2, 3};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void cs_sel(void);
void cs_desel(void);
uint8_t spi_rb(void);
void spi_wb(uint8_t b);
void spi_rb_burst(uint8_t *buf, uint16_t len);
void spi_wb_burst(uint8_t *buf, uint16_t len);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_HTTPSOCK	2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskETH */
osThreadId_t TaskETHHandle;
const osThreadAttr_t TaskETH_attributes = {
  .name = "TaskETH",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskETH(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks = 0;
}

__weak unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TaskETH */
  TaskETHHandle = osThreadNew(StartTaskETH, NULL, &TaskETH_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
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
//	  // Test Open File + 8 Caracteres Tela_Principal.bin
//	  duracao = HAL_GetTick();
//	  fr = f_open(&USERFile, "Tela_Principal.bin", FA_READ);
//	  if(fr != FR_OK) {
//	 	 logI("FatFs - Tela_Principal.bin Error...Result: %d\n\r", fr);
//	  }
//	  else {
//		 duracao = HAL_GetTick() - duracao;
//	 	 size = f_size(&USERFile);
//	 	 logI("FatFs - Open File Tela_Principal.bin... Result: %d  Size:%d Duracao: %d\n\r", fr, size, duracao);
	      //fr = f_read(&USERFile, line , 100, &ByteRead);
	      //if(fr == FR_OK) {
	     //	 logI("FatFs ReadFile...line: %s\n\r", line);
	      //}
	      //else {
	     //	 logI("FatFs ReadFile...Error:  Result: %d \n\r", fr);
	      //}
//
  }
  /* Infinite loop */
  for(;;)
  {
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

	  if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
		  //logI("Cable USB Connected\n\r");
	  }
	  else {
		  //logI("Cable USB Not Connected\n\r");
	  }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskETH */
/**
* @brief Function implementing the TaskETH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskETH */
void StartTaskETH(void *argument)
{
  /* USER CODE BEGIN StartTaskETH */
	reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
	//reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);
	reg_wizchip_cris_cbfunc(vPortEnterCritical, vPortExitCritical);

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
  /* Infinite loop */
  for(;;)
  {
	  // HTTP
	  for(uint8_t i = 0; i < MAX_HTTPSOCK; i++)	{
		  httpServer_run(i); 	// HTTP Server handler
	  }
    osDelay(1);
  }
  /* USER CODE END StartTaskETH */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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
/* USER CODE END Application */

