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
	HAL_UART_Transmit(&huart6, data, size, 1000);
}
