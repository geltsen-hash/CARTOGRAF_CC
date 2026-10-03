/*
 * uart.h
 *
 *  Created on: 16 мая 2022 г.
 *      Author: 1
 *
 *      для DriverLib
 */

#ifndef UART_H_
#define UART_H_

#include "driverlib.h"

void SendString(char *msg);
void SendChar(int c);
void SendDataExt(uint16_t *pData, uint16_t len);
void SendData16Ext(uint16_t *pData, uint16_t n);
//Для приема и обработки команд  по UART
struct TUartCMD
{
    char buf[2000]; //принятые байты
    unsigned int len; //длина
    bool ready; // =1 -> команда принята и ждет обработки, по окончанию обработки уст = 0
};

extern uint64_t cmdTime;
extern  struct TUartCMD UartCmd;

#endif /* UART_H_ */
