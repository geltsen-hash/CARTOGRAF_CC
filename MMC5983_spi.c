/*
 * MMC5983_spi.c
 *
 *  Created on: 1 апр. 2024 г.
 *      Author: user
 *
 *      Версия для DriverLib
 *      Реализовано только continuous mode через SPI
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
long gMMC5983_BridgeOffset[3] = {0, 0, 0};

static void MMC5983_ReadRawXYZ_Internal(long *raw)
{
    volatile uint16_t buffer[7] = {0,};
    volatile long x=0, y=0, z=0;
    volatile uint16_t data = 0;

    GPIO_writePin(MMC_CS, 0);
        SPI_writeDataBlockingNonFIFO(SPIBASE, 0x8000);
        data = SPI_readDataBlockingNonFIFO(SPIBASE);
        for(int i = 0; i < 7; i++)
        {
            SPI_writeDataBlockingNonFIFO(SPIBASE, 0x00);
            buffer[i] = SPI_readDataBlockingNonFIFO(SPIBASE);
        }
    GPIO_writePin(MMC_CS, 1);

    x = buffer[0];
    x = (x << 8) | buffer[1];
    x = (x << 2) | (buffer[6] >> 6);
    y = buffer[2];
    y = (y << 8) | buffer[3];
    y = (y << 2) | ((buffer[6] >> 4) & 0x03);
    z = buffer[4];
    z = (z << 8) | buffer[5];
    z = (z << 2) | ((buffer[6] >> 2) & 0x03);

    x -= (uint32_t)1 << 17;
    y -= (uint32_t)1 << 17;
    z -= (uint32_t)1 << 17;

    raw[0] = x;
    raw[1] = y;
    raw[2] = z;
}

uint8_t MMC5983_ReadTemperature(void)
{
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, 0x02); // TM_T
    DEVICE_DELAY_US(2000);
    return (uint8_t)MMC5983_readReg(MMC5983_T_OUT_REG);
}

void MMC5983_CalibrateBridge(void)
{
    long set_val[3] = {0, 0, 0};
    long reset_val[3] = {0, 0, 0};

    // 1. ременно отключаем непрерывный режим
    MMC5983_writeReg(MMC5983_INT_CTRL_2_REG, 0x00);
    DEVICE_DELAY_US(1000);

    // 2. мпульс SET и измерение при прямой намагниченности
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, (1<<3)); // SET coil
    DEVICE_DELAY_US(1000);
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, 0x01); // TM_M
    DEVICE_DELAY_US(1200);
    MMC5983_ReadRawXYZ_Internal(set_val);

    // 3. мпульс RESET и измерение при обратной намагниченности
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, (1<<4)); // RESET coil
    DEVICE_DELAY_US(1000);
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, 0x01); // TM_M
    DEVICE_DELAY_US(1200);
    MMC5983_ReadRawXYZ_Internal(reset_val);

    // 4. ычисление смещения нуля пермаллоевого моста (Null Field Offset)
    gMMC5983_BridgeOffset[0] = (set_val[0] + reset_val[0]) / 2;
    gMMC5983_BridgeOffset[1] = (set_val[1] + reset_val[1]) / 2;
    gMMC5983_BridgeOffset[2] = (set_val[2] + reset_val[2]) / 2;

    // 5. озврат датчика в рабочее состояние (импульс SET)
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, (1<<3)); // SET coil
    DEVICE_DELAY_US(1000);

    // 6. озврат в непрерывный режим 1000 ц с периодическим SET
    MMC5983_writeReg(MMC5983_INT_CTRL_2_REG, 0xDF);
    MMC5983_writeReg(MMC5983_INT_CTRL_0_REG, (1<<5)); // Auto_SR
}

void MMC5983_Init()
{
    MMC5983_writeReg(MMC5983_INT_CTRL_1_REG, 0x80); // SW Reset
    DEVICE_DELAY_US(15000);

    MMC5983_writeReg(MMC5983_INT_CTRL_1_REG, 0x03); // BW=800Hz
    DEVICE_DELAY_US(1000);

    // алибровка моста при включении (нахождение Null Field Offset)
    MMC5983_CalibrateBridge();
}

void MMC5983_ReadXYZ(long *Mptr)
{
    long raw[3] = {0, 0, 0};
    MMC5983_ReadRawXYZ_Internal(raw);

    // ычитаем аппаратный оффсет моста пермаллоя
    long x = raw[0] - gMMC5983_BridgeOffset[0];
    long y = raw[1] - gMMC5983_BridgeOffset[1];
    long z = raw[2] - gMMC5983_BridgeOffset[2];

    Mptr[0] = -x;
    Mptr[1] = -y;
    Mptr[2] = -z;
}
