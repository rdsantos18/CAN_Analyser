/*
 * RS232.c
 *
 *  Created on: Jun 17, 2024
 *      Author: rinaldo.santos
 */

#include "usart.h"
#include "RS232.h"

void send_uart(uint8_t *data, uint16_t size)
{
	HAL_GPIO_TogglePin(LED_232_TX_GPIO_Port, LED_232_TX_Pin);
	HAL_UART_Transmit(&huart6, data, size, 1000);
	HAL_GPIO_WritePin(LED_232_TX_GPIO_Port, LED_232_TX_Pin, GPIO_PIN_RESET);
}
