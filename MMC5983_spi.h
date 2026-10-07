/*
 * MMC5983_spi.h
 *
 *  Created on: 1 ���. 2024 �.
 *      Author: user
 *
 *      ������ ��� DriverLib
 */

#ifndef MMC5983_SPI_H_
#define MMC5983_SPI_H_

#include "driverlib.h"

#define SPIBASE SPIA_BASE //� ������ SPI ���������

#define MMC5983_X_OUT_0_REG      0x0
#define MMC5983_X_OUT_1_REG      0X01
#define MMC5983_Y_OUT_0_REG      0x02
#define MMC5983_Y_OUT_1_REG      0x03
#define MMC5983_Z_OUT_0_REG      0x04
#define MMC5983_Z_OUT_1_REG      0x05
#define MMC5983_XYZ_OUT_2_REG    0x06
#define MMC5983_T_OUT_REG        0x07
#define MMC5983_STATUS_REG       0x08
#define MMC5983_INT_CTRL_0_REG   0x09
#define MMC5983_INT_CTRL_1_REG   0x0a
#define MMC5983_INT_CTRL_2_REG   0x0b
#define MMC5983_INT_CTRL_3_REG   0x0c
#define MMC5983_PROD_ID_REG      0x2f



void MMC5983_Init();
void MMC5983_writeReg(uint16_t addr, uint16_t value);
uint16_t MMC5983_readReg(uint16_t regaddr);
void MMC5983_ReadXYZ(long *Mptr);
void MMC5983_StartAutoSR(void);
uint8_t MMC5983_ReadTemperature(void);
//void MMC5983_Reset();
//void MMC5983_Set();


#endif /* MMC5983_SPI_H_ */
