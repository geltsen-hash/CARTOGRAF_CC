/*
 * spil3gd20.h
 *
 *  Created on: 19 мая 2022 г.
 *      Author: 1
 */

#ifndef SPIL3GD20_H_
#define SPIL3GD20_H_

#define WHO_AM_I    0x0F// Register addresses from sensor datasheet.
#define CTRL1       0x20
#define CTRL2       0x21
#define CTRL3       0x22
#define CTRL4       0x23
#define CTRL5       0x24
#define REFERENCE   0x27
#define OUT_TEMP    0x26
#define STATUS      0x27

#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

//Full scale selection reg CTRL4
#define  DPS_245    0x00 //8.75 mdps/digit
#define  DPS_500    0x20 //17.50 mdps/digit
#define  DPS_2000   0x30 //70 mdps/digit

void L3GD20_Init(void);
void L3GD20_WReg(unsigned int reg, unsigned int val);
unsigned int L3GD20_RReg(unsigned int reg);
void L3GD20_ReadXYZ(int *W);

#endif /* SPIL3GD20_H_ */
