/*
 * RS485.c
 *
 *  Created on: May 3, 2024
 *      Author: rinaldo.santos
 */

#include "main.h"
#include "usart.h"
#include "RS485.h"
#include "log.h"

uint8_t Tx485[128] = {0};
uint8_t Rx485[128] = {0};
uint16_t crc_16;
uint8_t crc_8;

void sendRS485 (uint8_t *data, uint16_t size)
{
	// Pull DE high to enable TX operation
	HAL_GPIO_WritePin(TX_485_EN_GPIO_Port, TX_485_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_485_TX_GPIO_Port, LED_485_TX_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, data, size , 1000);
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(LED_485_TX_GPIO_Port, LED_485_TX_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_485_EN_GPIO_Port, TX_485_EN_Pin, GPIO_PIN_RESET);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	Tx485[0] = 0xA5;
	Tx485[1] = 0x40;
	Tx485[2] = 0x90;
	Tx485[3] = 0x08;
	Tx485[4] = 0x00;
	Tx485[5] = 0x00;
	Tx485[6] = 0x00;
	Tx485[7] = 0x00;
	Tx485[8] = 0x00;
	Tx485[9] = 0x00;
	Tx485[10] = 0x00;
	Tx485[11] = 0x00;
	Tx485[12] = 0x7D;

	sendRS485(Tx485, 13);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, Rx485, 128);
	logI("BMS Len: %d  %02X 02X 02X 02X 02X 02X 02X 02X \n\r", Rx485[0],
			Rx485[1], Rx485[2], Rx485[3], Rx485[4], Rx485[5], Rx485[6], Rx485[7], Rx485[8] );
	HAL_GPIO_TogglePin(LED_CAN1_RX_GPIO_Port, LED_CAN1_RX_Pin);
}

void test_485(void)
{
	Tx485[0] = 0xA5;
	Tx485[1] = 0x40;
	Tx485[2] = 0x90;
	Tx485[3] = 0x08;
	Tx485[4] = 0x00;
	Tx485[5] = 0x00;
	Tx485[6] = 0x00;
	Tx485[7] = 0x00;
	Tx485[8] = 0x00;
	Tx485[9] = 0x00;
	Tx485[10] = 0x00;
	Tx485[11] = 0x00;
	Tx485[12] = 0x7D;

	sendRS485(Tx485, 13);
}

uint8_t calc_crc_8(uint8_t *data)
{
	crc_16 = 0;

	for(uint8_t x=0; x < 12; x++) {
		crc_16 += data[x];
	}
	crc_8 = (uint8_t)crc_16 & 0xFF;

	return crc_8;
}
