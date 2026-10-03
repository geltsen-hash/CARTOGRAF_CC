/*
 * uart.c
 *
 *  Created on: 16 ��� 2022 �.
 *      Author: 1
 *
 *      ��� DriverLib
 */
#include "uart.h"
#include "string.h"
#include "device.h"
#include "board.h"
#include "_globals.h"
#include "CRC16ModBus.h"

#define UART_BUF_SIZE 2048

struct TUartCMD UartCmd; //BT
struct TUartCMD UartCmdExt; //EXT

uint64_t gPktTimeEXT = 0;
volatile uint16_t gExtPktsRq = 0;
volatile uint16_t gExtPktsAns = 0;
volatile uint16_t gErrorCnt = 0;
//volatile uint64_t gRxBytesExt = 0;
//volatile uint64_t gRxBytesBT = 0;
//------------------------------------------------------------------------------------
void SendChar(int c)
{
    SCI_writeCharBlockingNonFIFO(SCIA_BASE, c);
}
//------------------------------------------------------------------------------------
void SendString(char *msg)
{
    static int i;
    i = 0;
    while(msg[i] != '\0')
    {
        SCI_writeCharBlockingNonFIFO(SCIA_BASE, msg[i]);
        i++;
    }
}
//------------------------------------------------------------------------------------
void SendDataExt(uint16_t *pData, uint16_t len)
{
    //uint16_t sz = len/2;
    for(uint16_t i=0;i<len;i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, pData[i]);
        //SCI_writeCharBlockingNonFIFO(SCIC_BASE, pData[i]&0xFF);
        //SCI_writeCharBlockingNonFIFO(SCIC_BASE, (pData[i]>>8)&0xFF);
    }
}
//------------------------------------------------------------------------------------
void SendData16Ext(uint16_t *pData, uint16_t n) //�� ������
{
    uint16_t i = 0;
    while(n > i)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, pData[i]);
        i++;
    }
}
//------------------------------------------------------------------------------------
//--------------����� BLUETOOTH-------------------------------------------------------
uint16_t RxBufferBT[256] = {0,};
__interrupt void INT_UartBT_RX_ISR(void) //SCI A
{
   //gRxBytesBT++;
   static int idx = 0;
   volatile char dummy;

   static uint64_t pktTimeBT = 0;

   if((GetNow() - pktTimeBT) > 5)
   {
     idx = 0;
     memset(RxBufferBT, 0x00, sizeof(RxBufferBT));
   }
   pktTimeBT = GetNow();

   if(UartCmd.ready == false) //�������� ����� �������
   {
       while(SCI_getRxFIFOStatus(UartBT_BASE) != 0)
       {
           if(idx < sizeof(RxBufferBT))
           {
               RxBufferBT[idx] = SCI_readCharBlockingFIFO(UartBT_BASE);
               idx++;
           }
           else
           {
               dummy = SCI_readCharBlockingFIFO(UartBT_BASE);
               idx = 0;
               memset(RxBufferBT, 0x00, sizeof(RxBufferBT));
               break;
           }
       }

       if(idx > 0 && RxBufferBT[idx-1] == '\n')
       {
           uint16_t copy_len = (idx <= sizeof(UartCmd.buf)) ? idx : sizeof(UartCmd.buf);
           memcpy(UartCmd.buf, RxBufferBT, copy_len);
           UartCmd.len = copy_len;
           UartCmd.ready = true;
           idx = 0;
           pktTimeBT = 0;
           memset(RxBufferBT, 0x00, sizeof(RxBufferBT));
       }
   }
   else //���� ������� ��� �� ������������ - ������ �� ������
   {
        while(SCI_getRxFIFOStatus(UartBT_BASE) != 0)
        {
          dummy = SCI_readCharBlockingFIFO(UartBT_BASE);
        }
   }

    SCI_clearOverflowStatus(UartBT_BASE);
    SCI_clearInterruptStatus(UartBT_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
//-----------------------------------------------------------------------------
__interrupt void INT_UartBT_TX_ISR(void)
{

}
__interrupt void INT_UartExt_TX_ISR(void)
{

}
//------------------------------------------------------------------------------------
//--------------����� ������� �������-------------------------------------------------
uint16_t RxBufferExt[UART_BUF_SIZE] = {0,};
__interrupt void INT_UartExt_RX_ISR(void) //SCI C
{
//static uint16_t RxBufferExt[UART_BUF_SIZE] = {0,};
//gRxBytesExt++;
volatile static int16_t len=0, idx=0;
volatile uint16_t dummy=0;

    if(GetNow() > gPktTimeEXT + 5)
    {
        memset(RxBufferExt, 0x00, UART_BUF_SIZE);
        idx = 0;
        len = 0;
    }

    gPktTimeEXT = GetNow(); //��������� ����� ������� ���������� �����

    if(UartCmdExt.ready == false) //������� ��� �� ������� - ��������
    {
        while(SCI_getRxFIFOStatus(UartExt_BASE) != 0)
        {
           if(idx >= UART_BUF_SIZE)
           {
               dummy = SCI_readCharBlockingFIFO(UartExt_BASE);
               memset(RxBufferExt, 0x00, UART_BUF_SIZE);
               idx = 0;
               len = 0;
               continue;
           }

           RxBufferExt[idx] = SCI_readCharBlockingFIFO(UartExt_BASE);
           if(idx == 2)
           {
               len = RxBufferExt[1] + ((RxBufferExt[2]<<8)&0xFF00);
               if(len > UART_BUF_SIZE || len < 4)
               {
                   memset(RxBufferExt, 0x00, UART_BUF_SIZE);
                   idx = 0;
                   len = 0;
                   continue;
               }
           }
           else if(len > 0 && idx == len-1)
           {
               if(CRC16(RxBufferExt, len) == 0)
               {
                    uint16_t copy_len = (len <= sizeof(UartCmdExt.buf)) ? len : sizeof(UartCmdExt.buf);
                    memcpy(UartCmdExt.buf, RxBufferExt, copy_len);
                    UartCmdExt.len = copy_len;
                    UartCmdExt.ready = true;
                    gExtPktsRq++;
               }
               else
               {
                   gErrorCnt++;
               }
               memset(RxBufferExt, 0x00, UART_BUF_SIZE);
               idx = 0;
               len = 0;
               gPktTimeEXT = 0;
               break;
           }
           idx++;
        }
    }
    else //������� ��� ������� �� ��� �� ������������ - ������ FIFO �� ������ �� ������
    {
        while(SCI_getRxFIFOStatus(UartExt_BASE) != 0)
        {
          dummy = SCI_readCharBlockingFIFO(UartExt_BASE);
        }
    }

SCI_clearOverflowStatus(UartExt_BASE);
SCI_clearInterruptStatus(UartExt_BASE, SCI_INT_RXFF);
Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8); //9
}
//-----------------------------------------------------------------------------
