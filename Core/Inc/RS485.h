/*
 * RS485.h
 *
 *  Created on: May 3, 2024
 *      Author: rinaldo.santos
 */

#ifndef INC_RS485_H_
#define INC_RS485_H_

void sendRS485 (uint8_t *data, uint16_t size);
uint8_t calc_crc_8(uint8_t *data);

#endif /* INC_RS485_H_ */
