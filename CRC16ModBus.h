/*
 * CRC16ModBus.h
 *
 *  Created on: 11 окт. 2024 г.
 *      Author: user
 */

#ifndef CRC16MODBUS_H_
#define CRC16MODBUS_H_
#include <stdint.h>

#define CRC16SWAP 1

uint16_t CRC16(uint16_t *buf, int len);
uint16_t CRC16s(const uint16_t *nData, uint16_t wLength);


#endif /* CRC16MODBUS_H_ */
