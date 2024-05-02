/*
 * log.c
 *
 *  Created on: May 2, 2024
 *      Author: rinaldo.santos
 */

#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "usart.h"

char string_usb[2048] = {0};

void HAL_printf_valist(const char *fmt, va_list argp)
{
	if (vsprintf(string_usb, fmt, argp) > 0) {
		//CDC_Transmit_FS((uint8_t*)string_usb, strlen(string_usb));				// send message via USB CDC
		HAL_UART_Transmit(&huart1, (uint8_t *)string_usb, strlen(string_usb), 1000);
	}
}


void logUSB(const char *fmt, va_list argp)
{
	HAL_printf_valist(fmt, argp);
}

void logI(const char* fmt, ...)
{
	va_list argp;

	va_start(argp, fmt);
	logUSB(fmt, argp);
	va_end(argp);
}
