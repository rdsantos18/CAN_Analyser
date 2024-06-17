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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define KEY_USER_Pin GPIO_PIN_13
#define KEY_USER_GPIO_Port GPIOC
#define LED_CAN1_TX_Pin GPIO_PIN_0
#define LED_CAN1_TX_GPIO_Port GPIOC
#define LED_CAN1_RX_Pin GPIO_PIN_1
#define LED_CAN1_RX_GPIO_Port GPIOC
#define LED_CAN2_TX_Pin GPIO_PIN_2
#define LED_CAN2_TX_GPIO_Port GPIOC
#define LED_CAN2_RX_Pin GPIO_PIN_3
#define LED_CAN2_RX_GPIO_Port GPIOC
#define SPI_NSS_Pin GPIO_PIN_0
#define SPI_NSS_GPIO_Port GPIOA
#define TX_485_EN_Pin GPIO_PIN_1
#define TX_485_EN_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define INT_ETH_Pin GPIO_PIN_4
#define INT_ETH_GPIO_Port GPIOC
#define ETH_RST_Pin GPIO_PIN_5
#define ETH_RST_GPIO_Port GPIOC
#define LED_485_TX_Pin GPIO_PIN_0
#define LED_485_TX_GPIO_Port GPIOB
#define LED_485_RX_Pin GPIO_PIN_1
#define LED_485_RX_GPIO_Port GPIOB
#define LED_INT_Pin GPIO_PIN_2
#define LED_INT_GPIO_Port GPIOB
#define LED_232_TX_Pin GPIO_PIN_14
#define LED_232_TX_GPIO_Port GPIOB
#define LED_232_RX_Pin GPIO_PIN_15
#define LED_232_RX_GPIO_Port GPIOB
#define SD_DETECT_Pin GPIO_PIN_8
#define SD_DETECT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
