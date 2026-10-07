/*
 * MMC5983_spi.c
 *
 *  Created on: 1 ���. 2024 �.
 *      Author: user
 *
 *      ������ ��� DriverLib
 *      ����������� ������ continuous mode ����� SPI
 */

#include "MMC5983_spi.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"

//-----------------------------------------------------------------------------
uint16_t MMC5983_readReg(uint16_t regaddr)
{
uint16_t cmd = ((regaddr)|0x80)<<8; //0x01=read 0x00=write
uint16_t data = 0;
   GPIO_writePin(MMC_CS, 0);
   DEVICE_DELAY_US(1);
       SPI_writeDataBlockingNonFIFO(SPIBASE, cmd);
       data = SPI_readDataBlockingNonFIFO(SPIBASE);
       SPI_writeDataBlockingNonFIFO(SPIBASE, 0x00);
       data = SPI_readDataBlockingNonFIFO(SPIBASE);
   DEVICE_DELAY_US(1);
   GPIO_writePin(MMC_CS, 1);
return data;
}
//-----------------------------------------------------------------------------
void MMC5983_writeReg(uint16_t addr, uint16_t value)
{
uint16_t cmd = ((addr)&0x7FFF)<<8;
volatile uint16_t dummy = 0;
   GPIO_writePin(MMC_CS, 0);
       SPI_writeDataBlockingNonFIFO(SPIBASE, cmd);
       dummy = SPI_readDataBlockingNonFIFO(SPIBASE);
       SPI_writeDataBlockingNonFIFO(SPIBASE, value<<8);
       dummy = SPI_readDataBlockingNonFIFO(SPIBASE);
   GPIO_writePin(MMC_CS, 1);
}
//-----------------------------------------------------------------------------
uint8_t MMC5983_ReadTemperature(void)
{
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, 0x02); // TM_T
    DEVICE_DELAY_US(2000);
    return (uint8_t)MMC5983_readReg(MMC5983_T_OUT_REG);
}

void MMC5983_StartAutoSR(void)
{
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, 0x21); // Auto_SR_en (0x20) | TM_M (0x01)
}

void MMC5983_Init()
{
    MMC5983_writeReg(MMC5983_INT_CTRL_1_REG, 0x80); // SW Reset
    DEVICE_DELAY_US(15000);

    // Degaussing cycle with full 20ms capacitor charge:
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, (1<<4)); // coil RESET
    DEVICE_DELAY_US(20000); // 20ms full recharge of CAP (10uF)

    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, (1<<3)); // coil SET (final forward state)
    DEVICE_DELAY_US(20000); // 20ms full recharge of CAP (10uF)

    MMC5983_writeReg(MMC5983_INT_CTRL_1_REG, 0x03); // BW=800Hz (0.5ms A/D conversion time)
    MMC5983_writeReg(MMC5983_INT_CTRL_2_REG, 0x00); // Continuous Mode disabled (One-Shot mode)

    // Prime the very first Auto_SR measurement so registers have valid data on first timer tick:
    MMC5983_StartAutoSR();
    DEVICE_DELAY_US(2000); // 2ms initial completion wait
}

void MMC5983_ReadXYZ(long *Mptr)
{
    volatile uint16_t buffer[7] = {0,};
    volatile long x=0, y=0, z=0;
    volatile uint16_t data = 0;

    GPIO_writePin(MMC_CS, 0);
        SPI_writeDataBlockingNonFIFO(SPIBASE, 0x8000); // read from address 0
        data = SPI_readDataBlockingNonFIFO(SPIBASE);
        for(int i = 0; i < 7; i++)
        {
            SPI_writeDataBlockingNonFIFO(SPIBASE, 0x00);
            buffer[i] = SPI_readDataBlockingNonFIFO(SPIBASE);
        }
    GPIO_writePin(MMC_CS, 1);

    x = buffer[0]; // Xout[17:10]
    x = (x << 8) | buffer[1]; // Xout[9:2]
    x = (x << 2) | (buffer[6] >> 6); // Xout[1:0]
    y = buffer[2]; // Yout[17:10]
    y = (y << 8) | buffer[3]; // Yout[9:2]
    y = (y << 2) | ((buffer[6] >> 4) & 0x03); // Yout[1:0]
    z = buffer[4]; // Zout[17:10]
    z = (z << 8) | buffer[5]; // Zout[9:2]
    z = (z << 2) | ((buffer[6] >> 2) & 0x03); // Zout[1:0]

    x -= (uint32_t)1 << 17;
    y -= (uint32_t)1 << 17;
    z -= (uint32_t)1 << 17;

    Mptr[0] = -x;
    Mptr[1] = -y;
    Mptr[2] = -z;
}
