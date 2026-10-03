/*
 * spil3gd20.c
 * Версия для driverlib
 *  Created on: 19 мая 2022 г.
 *      Author: 1
 */

#include "driverlib.h"
#include "_globals.h"
#include "device.h"
#include "board.h"
#include "spil3gd20.h"
//--------------------------------------------------------------------------------------------
void L3GD20_Init(void)
{
    L3GD20_WReg(CTRL1, 0xFF); //normal, xyz=enable, ODR = 760Hz, cutoff = 100Hz
    L3GD20_WReg(CTRL4, DPS_2000);
}
//--------------------------------------------------------------------------------------------
unsigned int L3GD20_RReg(unsigned int reg)
{
 uint16_t cmd = 0;
 uint16_t data = 0;
 cmd = ((reg | 0x80) & 0x00FF) << 8; //RW=1; MS=0
 GPIO_writePin(Gyro_CS, 0);
     SPI_writeDataBlockingNonFIFO(SPIA_BASE, cmd);
     data = SPI_readDataBlockingNonFIFO(SPIA_BASE);
     SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x00);
     data = SPI_readDataBlockingNonFIFO(SPIA_BASE);
 GPIO_writePin(Gyro_CS, 1);
 return data;
}
//--------------------------------------------------------------------------------------------
void L3GD20_WReg(unsigned int reg, unsigned int val)
{
  uint16_t cmd = 0;
  volatile uint16_t data = 0; //dummy data
  cmd = ((reg & 0x3F) & 0x00FF) << 8; //RW=0; MS=0
    GPIO_writePin(Gyro_CS, 0);
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, cmd);
        data = SPI_readDataBlockingNonFIFO(SPIA_BASE);

        cmd = (val & 0x00FF) << 8;
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, cmd);
        data = SPI_readDataBlockingNonFIFO(SPIA_BASE);
    GPIO_writePin(Gyro_CS, 1);
}
//--------------------------------------------------------------------------------------------
void L3GD20_ReadXYZ(int *W)
{
    uint16_t cmd = 0;
    volatile uint16_t data = 0;
    int w_val;
    cmd = (uint16_t)(0x00E8 << 8); //RW=1; MS=1 адрес начала чтения 0x28
    GPIO_writePin(Gyro_CS, 0);
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, cmd);
        data = SPI_readDataBlockingNonFIFO(SPIA_BASE);
        for(int i=0; i<3; i++)
        {
            w_val = 0;
            SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0);
            w_val = SPI_readDataBlockingNonFIFO(SPIA_BASE);

            SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0);
            data = SPI_readDataBlockingNonFIFO(SPIA_BASE);
            w_val = w_val | (data << 8);

            W[i] = w_val;
        }

    GPIO_writePin(Gyro_CS, 1);

}
