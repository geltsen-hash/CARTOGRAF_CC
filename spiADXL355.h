/*
 * spiADXL355.h
 *
 *  Created on: 28 февр. 2024 г.
 *      Author: user
 */

#ifndef SPIADXL355_H_
#define SPIADXL355_H_

#define ADXL355_DEVID_AD 0
#define ADXL355_TEMP2 0x06
#define ADXL355_TEMP1 0x07

#define ADXL355_FILTER 0x28
#define ADXL355_SELF_TEST 0x2E
#define ADXL355_RESET 0x2F

#define ADXL355_XDATA3 0x08
#define ADXL355_XDATA2 0x09
#define ADXL355_XDATA1 0x0A
#define ADXL355_YDATA3 0x0B
#define ADXL355_YDATA2 0x0C
#define ADXL355_YDATA1 0x0D
#define ADXL355_ZDATA3 0x0E
#define ADXL355_ZDATA2 0x0F
#define ADXL355_ZDATA1 0x10

#define ADXL355_RANGE 0x2C
#define ADXL355_POWER_CTL 0x2D

// Device values
#define ADXL355_RANGE_2G 0x01
#define ADXL355_RANGE_4G 0x02
#define ADXL355_RANGE_8G 0x03
#define ADXL355_MEASURE_MODE 0x06 // Only accelerometer

// Operations
//const int READ_BYTE = 0x01;
//const int WRITE_BYTE = 0x00;

uint16_t ADXL355_ReadReg(uint16_t addr);
void ADXL355_WriteReg(uint16_t addr, uint16_t data);
void ADXL355_Init(void);
void ADXL355_ReadXYZ(int32_t* Gdata);
void ADXL355_Reset();


#endif /* SPIADXL355_H_ */
