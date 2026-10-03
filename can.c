/*
 * can.c
 *
 *  Created on: 22 февр. 2024 г.
 *      Author: user
 */


#include "driverlib.h"
#include "_globals.h"
#include "device.h"
#include "board.h"
#include "uart.h"
#include <stdio.h>
#include "string.h"

#define CAN_ID_BROADCAST 1
#define CAN_ID_GETRXDATA 2
#define CAN_ID_WRITEDATATODEVICE 31
#define CAN_TX_TIMEOUT 250 //us попытка передачи 2 байт
#define CAN_RX_TIMEOUT 60000 //us время на передачу 16 пакетов по 8 байт (5760)

uint16_t gTxDataCAN[8];
uint16_t gRxDataCAN[8];
volatile uint16_t gCANError = CAN_ERROR_NOERROR;
volatile uint16_t gCANRxPktCnt = 0;
uint16_t gCANBuffer[256]; //136

//-----------------------------------------------------------------------------
void CAN_Send(uint16_t *data)
{

}
//-----------------------------------------------------------------------------
/* src - исходные неупакованные данные
* dest - куда сохранять готовые упакованные данные
* n_words - размер dest в 16-битных словах
*/
void PackBuffer(uint16_t *src, uint16_t *dest, uint16_t n_words)
{
    for(int i=0;i<n_words;i++)
    {
       dest[i] = (src[i*2]&0x00FF)|(((src[i*2+1]&0x00FF)<<8)&0xFF00);
    }
}
//-----------------------------------------------------------------------------
void CAN_Send_Broadcast() //широковещательный пакет, формируется из структуры oneworkcycle
{
    uint16_t cmd = 0;
    uint16_t data[2];

    if(wc.currFreq>=1000)
    {
        cmd |= (1<<1);//freq
    }

    if(wc.isDirectCycle == true)
    {
        cmd |= (1<<2);//rxmode
    }

    if(wc.TxWorkTime[wc.currTx]==4)//время работы
    {
    }
    else if(wc.TxWorkTime[wc.currTx]==8)
    {
        cmd |= (1<<4);
    }
    else if(wc.TxWorkTime[wc.currTx]==12)
    {
        cmd |= (1<<5);
    }
    else if(wc.TxWorkTime[wc.currTx]==16)
    {
        cmd |= (1<<4);
        cmd |= (1<<5);
    }
    else
    {
        //недопустимое значение
    }

    cmd |= (wc.currTx & 0x0007)<<6; //номер передатчика
    cmd |= (gSectorIdx & 0x000F)<<9; //номер сектора

    if(wc.bFinishFlag)
        cmd |=(1<<13);//финиш

    data[0] = cmd&0x00FF;
    data[1] = (cmd>>8)&0x00FF;

    CPUTimer_setPeriod(TIMER_US_BASE, 0xFFFFFFFF);//1us
    CPUTimer_reloadTimerCounter(TIMER_US_BASE);
    CPUTimer_startTimer(TIMER_US_BASE);

    CAN_sendMessage(CANA_BASE, CAN_ID_BROADCAST, 2, data);
    while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)
    {
        if(CPUTimer_getTimerCount(TIMER_US_BASE) < 0xFFFFFFFF - CAN_TX_TIMEOUT)
        {
            gCANError = CAN_ERROR_TX_FAILED;
            GPIO_writePin(led4, 1);
            break;
        }
    }
}
//-----------------------------------------------------------------------------
uint16_t CAN_WriteDataToDevice(uint16_t targetdevice, uint32_t addr, uint32_t *value32)
{
    //byte 0: target
    //byte 1: address LSB
    //byte 2: -//-
    //byte 3: address MSB
    //byte 4: value LSB
    //byte 5: -//-
    //byte 6: -//-
    //byte 7: value MSB
    uint16_t data[8];
    uint32_t val = 0;
    memcpy(&val, value32, sizeof(uint32_t));

    data[0] = targetdevice & 0x00FF;

    data[1] = addr & 0x000000FF;
    data[2] = (addr >> 8) & 0x000000FF;
    data[3] = (addr >> 16) & 0x000000FF;

    data[4] = val & 0x000000FF;
    data[5] = (val >> 8) & 0x000000FF;
    data[6] = (val >> 16) & 0x000000FF;
    data[7] = (val >> 24) & 0x000000FF;

    CPUTimer_setPeriod(TIMER_US_BASE, 0xFFFFFFFF);//1us
    CPUTimer_reloadTimerCounter(TIMER_US_BASE);
    CPUTimer_startTimer(TIMER_US_BASE);

    CAN_sendMessage(CANA_BASE, CAN_ID_WRITEDATATODEVICE, 8, data); // предаем команду
        while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)// ждем завершения передачи команды
        {
            if(CPUTimer_getTimerCount(TIMER_US_BASE) < 0xFFFFFFFF - CAN_TX_TIMEOUT)
            {
                //CPUTimer_stopTimer(TIMER_US_BASE);
                gCANError = CAN_ERROR_TX_FAILED;
                GPIO_writePin(led4, 1);
                return false; //таймаут
            }
        }

    return true; //успешно
}
//-----------------------------------------------------------------------------
uint16_t CAN_GetRxdata(uint16_t rxNum) //получить данные с приемника по номеру в gCANBuffer
{
    uint16_t cmd = 1; //опрос
    uint16_t data[2];
    volatile uint16_t pktcnt = 0;//сколько пакетов ожидаем (сейчас 24+1 для ненапр. или 16+1 для напр.)
    memset(gCANBuffer, 0x00, sizeof(gCANBuffer));

    if(wc.currFreq>=1000)
    {
        cmd |= (1<<1);//freq
    }

    if(rxNum == 1 || rxNum == 2)
    {
        pktcnt = 12; //с ненаправленного
    }
    else
    {
        pktcnt = 17; //с направленного
        if(wc.isDirectCycle == true)
        {
           cmd |= (1<<2);//rxmode=direct
        }
    }

    cmd |= (rxNum & 0x0007)<<6; //номер приемника
    //cmd |= (gSectorIdx & 0x000F)<<9; //номер сектора

    data[0] = cmd&0x00FF;
    data[1] = (cmd>>8)&0x00FF;

    CPUTimer_setPeriod(TIMER_US_BASE, 0xFFFFFFFF);//1us
    CPUTimer_reloadTimerCounter(TIMER_US_BASE);
    CPUTimer_startTimer(TIMER_US_BASE);

    gCANRxPktCnt = 0;
    CAN_sendMessage(CANA_BASE, CAN_ID_GETRXDATA, 2, data); // предаем команду
    while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)// ждем завершения передачи команды
    {
        if(CPUTimer_getTimerCount(TIMER_US_BASE) < 0xFFFFFFFF - CAN_TX_TIMEOUT)
        {
            //CPUTimer_stopTimer(TIMER_US_BASE);
            gCANError = CAN_ERROR_TX_FAILED;
            GPIO_writePin(led4, 1);
            return false;
        }
    }

    while(gCANRxPktCnt < pktcnt) //ждем 12 или 17 пакетов от приемника
    {
       if(CPUTimer_getTimerCount(TIMER_US_BASE) < 0xFFFFFFFF - CAN_RX_TIMEOUT)
       {
           //CPUTimer_stopTimer(TIMER_US_BASE);
           gCANError = CAN_ERROR_RX_DATA_TIMEOUT;
           GPIO_writePin(led4, 1);
           return false;
       }
    }

    return true;
}
//-----------------------------------------------------------------------------
uint16_t CAN_GetRxdataRAW(uint16_t rxNum, uint16_t txNum) //получить RAW данные с приемника по номеру в gCANBuffer
{
    uint16_t cmd = 1;
    uint16_t data[2];
    memset(gCANBuffer, 0x00, sizeof(gCANBuffer));

    if(wc.currFreq>=1000)
    {
        cmd |= (1<<1);//freq
    }

    if(wc.isDirectCycle == true)
    {
        cmd |= (1<<3);//rxmode=raw
    }

    cmd |= (rxNum & 0x0007)<<6; //номер приемника
    cmd |= (txNum & 0x000F)<<9; //номер сектора

    data[0] = cmd&0x00FF;
    data[1] = (cmd>>8)&0x00FF;

    CPUTimer_setPeriod(TIMER_US_BASE, 0xFFFFFFFF);//1us
    CPUTimer_reloadTimerCounter(TIMER_US_BASE);
    CPUTimer_startTimer(TIMER_US_BASE);

    gCANRxPktCnt = 0;
    CAN_sendMessage(CANA_BASE, CAN_ID_GETRXDATA, 2, data); // предаем команду
    while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)// ждем завершения передачи команды
    {
        if(CPUTimer_getTimerCount(TIMER_US_BASE) < 0xFFFFFFFF - CAN_TX_TIMEOUT)
        {
            //CPUTimer_stopTimer(TIMER_US_BASE);
            gCANError = CAN_ERROR_TX_FAILED;
            GPIO_writePin(led4, 1);
            return false;
        }
    }

    while(gCANRxPktCnt < 17) //ждем 17 пакетов от приемника
    {
       if(CPUTimer_getTimerCount(TIMER_US_BASE) < 0xFFFFFFFF - CAN_RX_TIMEOUT)
       {
           //CPUTimer_stopTimer(TIMER_US_BASE);
           gCANError = CAN_ERROR_RX_DATA_TIMEOUT;
           GPIO_writePin(led4, 1);
           return false;
       }
    }

    return true;
}
//-----------------------------------------------------------------------------
__interrupt void INT_CAN_A_1_ISR(void)
{
    //не исп.
    CAN_clearGlobalInterruptStatus(CAN_A_BASE, CAN_GLOBAL_INT_CANINT1); //Clear the global interrupt flag for the CAN interrupt line
    Interrupt_clearACKGroup(INT_CAN_A_1_INTERRUPT_ACK_GROUP); //Acknowledge this interrupt located in group 9
}
//-----------------------------------------------------------------------------
__interrupt void INT_CAN_A_0_ISR(void)
{
    volatile uint32_t status = CAN_getInterruptCause(CANA_BASE);
    if(status == CAN_INT_INT0ID_STATUS)
    {
        status = CAN_getStatus(CANA_BASE);
        if(((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 7) &&
           ((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 0))
        {
            gCANError = 1;
        }
    }
    else if(status == 1)//TX - broadcast передано
    {
        CAN_clearInterruptStatus(CANA_BASE, 1);
        gCANError = 0;
    }
    else if(status == 2)//TX - get data передано
    {
        //gCANPktCount = 0; //начало приема 16 пакетов
        CAN_clearInterruptStatus(CANA_BASE, 2);//1
        gCANError = 0;
    }
    else if(status == 32) //принят последний пакет с информацией
    {
        gCANRxPktCnt = 0;
        for(int i=0;i<24;i++)//информация
        {
            status = CAN_readMessage(CANA_BASE, 3+i, gCANBuffer+i*8);
            if(status == true)
                gCANRxPktCnt++;

        }

        status = CAN_readMessage(CANA_BASE, 32, gCANBuffer+8*29); //служебный
        if(status == true)
            gCANRxPktCnt++;

        CAN_clearInterruptStatus(CANA_BASE, 32);
    }
    else
    {
        CAN_clearInterruptStatus(CANA_BASE, status); //
    }


    CAN_clearGlobalInterruptStatus(CAN_A_BASE, CAN_GLOBAL_INT_CANINT0); //Clear the global interrupt flag for the CAN interrupt line
    Interrupt_clearACKGroup(INT_CAN_A_0_INTERRUPT_ACK_GROUP); //Acknowledge this interrupt located in group 9
}
//-----------------------------------------------------------------------------

