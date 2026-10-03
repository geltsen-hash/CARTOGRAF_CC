/*
 * spiADXL355.c
 *
 *  Created on: 28 февр. 2024 г.
 *      Author: user
 */

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "spiADXL355.h"
#include <stdio.h>
#include <string.h>
//-----------------------------------------------------------------------------
void ADXL355_Init(void)
{
    if(ADXL355_ReadReg(ADXL355_DEVID_AD) != 0xAD)
        return; //отказ

    ADXL355_WriteReg(ADXL355_RANGE, ADXL355_RANGE_2G);
    ADXL355_WriteReg(ADXL355_POWER_CTL, ADXL355_MEASURE_MODE);
}
//-----------------------------------------------------------------------------
void ADXL355_ReadXYZ(int32_t* Gdata)
{
int32_t buffer[9] = {0,};
uint16_t cmd = ((ADXL355_XDATA3<<1)|0x01)<<8; //начало = xdata3
volatile int32_t x=0, y=0, z=0;
volatile uint16_t dummy;
  x = 0;
  y = 0;
  z = 0;

    GPIO_writePin(Axel_CS, 0);

    cmd = ((ADXL355_XDATA3<<1)|0x01)<<8;

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, cmd);
    dummy = SPI_readDataBlockingNonFIFO(SPIA_BASE);

        for(uint16_t i=0;i<9;i++)
        {
            SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0);
            buffer[i] = SPI_readDataBlockingNonFIFO(SPIA_BASE);
        }

        x = ((buffer[0]<<24)|(buffer[1]<<16)|(buffer[2]<<8))>>12;
        y = ((buffer[3]<<24)|(buffer[4]<<16)|(buffer[5]<<8))>>12;
        z = ((buffer[6]<<24)|(buffer[7]<<16)|(buffer[8]<<8))>>12;

        Gdata[0] = x;
        Gdata[1] = y;
        Gdata[2] = z;

    GPIO_writePin(Axel_CS, 1);

}
//-----------------------------------------------------------------------------
uint16_t ADXL355_ReadReg(uint16_t addr)
{
uint16_t cmd = ((addr<<1)|0x01)<<8; //0x01=read 0x00=write
uint16_t data = 0;
   GPIO_writePin(Axel_CS, 0);
       SPI_writeDataBlockingNonFIFO(SPIA_BASE, cmd);
       data = SPI_readDataBlockingNonFIFO(SPIA_BASE);
       SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x00);
       data = SPI_readDataBlockingNonFIFO(SPIA_BASE);
   GPIO_writePin(Axel_CS, 1);
return data;
}
//-----------------------------------------------------------------------------
void ADXL355_WriteReg(uint16_t addr, uint16_t data)
{
uint16_t cmd = ((addr<<1)&0xFFFE)<<8;
volatile uint16_t dummy = 0;
   GPIO_writePin(Axel_CS, 0);
       SPI_writeDataBlockingNonFIFO(SPIA_BASE, cmd);
       dummy = SPI_readDataBlockingNonFIFO(SPIA_BASE);
       SPI_writeDataBlockingNonFIFO(SPIA_BASE, data<<8);
       dummy = SPI_readDataBlockingNonFIFO(SPIA_BASE);
   GPIO_writePin(Axel_CS, 1);
}
//-----------------------------------------------------------------------------
void ADXL355_Reset()
{
    ADXL355_WriteReg(ADXL355_RESET, 0x52);
}
