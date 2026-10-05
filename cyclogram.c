/*
 * cyclogram.c
 *
 *  Created on: 19 февр. 2025 г.
 *      Author: user
 */
#include "_globals.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include "CRC16ModBus.h"

#define TX_ADD_TIME 2
#define DELAY_2ND_CYCLE 50

//uint16_t bIsCartograph = false;

//const uint16_t sectorsCount = 16; // общее число секторов
//const uint16_t prepareDataTimeMcs = 128; // время ожидания вычиления данных на контроллерах приемников в мкс,
                                         // кратно 64мкс, но можно указывать любые значения
uint16_t bc_cmd = 0;
uint16_t condition = 0;
uint32_t frame = 1;

float gWlim = 0.0;
float fWgAtStart = 0.0;
uint16_t gStartSectorIdx;
struct TOneWorkCycle wc;
/*const uint16_t constTx_undirect[6] = {1,2,3,4,5,7};
const uint16_t constTxByTurn[5][6] = {{3,4,7,7,7,7},{1,2,5,7,7,7},{7,7,7,7,7,7},{7,7,7,7,7,7},{7,7,7,7,7,7}}; //макс. 5 оборотов 6 передатчиков
const uint16_t constTxWorkTime[7] = {4,4,4,8,8,4,0};*/

float GetWlim(void)
{
volatile uint32_t Time, TimeLim;
volatile uint16_t nTx, nTx_max; //сколько передатчиков
float wlim;

  TimeLim = 0;
  Time = 0;
  nTx_max = 0;

  for(int i=0; i<5; i++)//сколько нужно оборотов
   {
      Time = 0;
      nTx = 0;
       for(int j=0; j<6; j++)
       {
         if(wc.TxByTurn[SetOfPresets.TBTIndex][i][j]!=7)
         {
             Time += wc.TxWorkTime[j];
             nTx++;
         }
       }

       if(Time > TimeLim)
       {
           TimeLim = Time;
           nTx_max = nTx;
       }

   }

  TimeLim = TimeLim + TX_ADD_TIME + nTx_max;
  if(TimeLim>0)
      wlim = 125*PI/(TimeLim); //w = (22.5*1000*(pi/180))/T
  else
      wlim = 1;

  return wlim;
}

// Конструктор по умолчанию
void TOneWorkCycle_Create()
{
  memcpy(&wc.Tx_undirect, &(Metro.turn_presets.Tx_undirect[0]), sizeof(wc.Tx_undirect));
  memcpy(&wc.TxByTurn, &(Metro.turn_presets.Tx_by_turn[0]), sizeof(wc.TxByTurn));
  memcpy(&wc.TxWorkTime, &(Metro.turn_presets.tx_work_time_units[0]), sizeof(wc.TxWorkTime));

  GPIO_writePin(led1, 0);
  GPIO_writePin(led2, 0);
  GPIO_writePin(led3, 0);
  GPIO_writePin(led4, 0);

  if(CheckMetroData() == 0)
  {
      GPIO_writePin(led3, 1); //ошибка в файле метрологии
  }
  else
  {
      gWlim = GetWlim(); //Проверка предельной скорости
        if(gWlim > 0) //Индикатор скорости
        {
          if(fabs(Wg)>=gWlim)
          {
              GPIO_writePin(led3, 1);
              delay_ms(50);
          }
          else
          {
              GPIO_writePin(led3, 0);
          }
        }
  }

  fWgAtStart = (fabs(Wg)*30.0)/PI;

  if(gWorkType == TYPE_CARTOGRAPH) //======================================КАРТОГРАФ==================================
  {
    if(fabs(Wg)>=(PI*SetOfPresets.MinRotSpeed)/30.0) //2*pi
      wc.isDirectCycle = true;
    else
      wc.isDirectCycle = false;

    wc.freqMode = (FMode)((Metro.select_key >> 24) & 0x0003);

    if(wc.freqMode == FMODE_400_ONLY)//только 400
    {
        wc.currFreq = Metro.F[0];
    }
    else if(wc.freqMode == FMODE_2000_ONLY) //только 2000
    {
        wc.currFreq = Metro.F[1];
    }
    else if(wc.freqMode == FMODE_BOTH)
    {
        if(frame & 1) //по очереди: четный-нечетный фрейм
        {
            wc.currFreq = Metro.F[0];
        }
        else
        {
            wc.currFreq = Metro.F[1];
        }
    }
    else
    {
        GPIO_writePin(led3, 1); //ошибка в файле метрологии
    }

    for(int i=0; i<5; i++)//сколько нужно оборотов
    {
      for(int j=0; j<6; j++)
      {
        if(wc.TxByTurn[SetOfPresets.TBTIndex][i][j]!=7)
        {
          wc.turnsCount++;
          break;
        }
      }
    }
  }
  else if(gWorkType == TYPE_LWD_4TX) //======================================ИНДУКЦИОННИК==================================
  {
      //в этом режиме всегда ненапр. измерения 4 передатчика + молчащий по 4 мс
      wc.isDirectCycle = false;
      wc.turnsCount = 1;
      wc.currFreq = Metro.F[0];

      wc.Tx_undirect[0] = 1;
      wc.Tx_undirect[1] = 2;
      wc.Tx_undirect[2] = 3;
      wc.Tx_undirect[3] = 4;
      wc.Tx_undirect[4] = 7;
      wc.Tx_undirect[5] = 7;

      wc.TxWorkTime[0] = 4; //молчащий
      wc.TxWorkTime[1] = 4;
      wc.TxWorkTime[2] = 4;
      wc.TxWorkTime[3] = 4;
      wc.TxWorkTime[4] = 4;

      wc.delay2ndCycle = DELAY_2ND_CYCLE;
  }
  else
  {
      //неподдерживаемый тип
      GPIO_writePin(led3, 1);
      delay_ms(500);
  }

  wc.inclinoSector = 0;
  wc.sectorCounter = 0;
  wc.turnsCount = 0;
  wc.turnCounter = 0;
  wc.indexTxByTurn = 0; // для молчащих передатчиков это не важно
  wc.currTx = 0; // начинаем всегда с молчащих передатчиков
  wc.timeUnitCounter = 0;
  wc.allTimeUnits = wc.TxWorkTime[0]; // число тиков для молчащих передатчиков
  wc.prepareDataTimeMcsCounter = 0;
  wc.waitNextSector = false;
  wc.wasNewSectorInt = false;
  wc.wasADCInt = false;
  wc.bFinishFlag = false;
  wc.bTxActive = false;

  wc.currSubState = DMCSS_WaitForRequest;

  //AllData.frame = frame++;
}
//-------------------------------------------------------------------------------------------------------------
//volatile uint16_t crc_rx = 0;
volatile uint16_t bFreq = 0;
void WorkCyclogram()
{
  uint16_t crc = 0;
  volatile uint16_t crc_rx = 0;
  //volatile uint16_t bFreq = 0;
  if(wc.currFreq >= 1000)
    bFreq = 1;
  else
    bFreq = 0;

  if(wc.currSubState==DMCSS_WaitForRequest)
  {
    if(bExtCmdGetData)
    { // пришла команда от Олега на получение данных:
        bExtCmdGetData = false;
      //1. Отправляем данные Олегу - делаем это блокирующими посылками

      //2. Инициализируем счетчики и переменные и переходим в сбор данных для первого кванта:
        wc.turnCounter = 0;  // в цикле может быть несколько оборотов. Они все должны уложиться в 2 секунды
        wc.indexTxByTurn = 0; // начинаем с начального передатчика
        wc.sectorCounter = 0; // это просто счетчик секторов, который начинается с нуля а заканчивается 15;
                         // собственно привязка к конкретному сектору заключается в inclinoSector

        wc.currTx = 0; // молчащий передатчик
        wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
        wc.timeUnitCounter = 0;
        wc.bFinishFlag = false;
        wc.cycleStartTime_ms = GetNow(); // здесь начали считать время цикла;
        GPIO_writePin(led1, 0);
        GPIO_writePin(led2, 1); //begin --------------------------
        //GPIO_writePin(led3, 0);
        GPIO_writePin(led4, 0);
#ifndef EMUL
        CAN_Send_Broadcast(); //0-й молчащий передатчик
#endif
        wc.bTxActive = true;
        wc.currSubState = DMCSS_SilentTxState;
    }
  }
  else if(wc.currSubState==DMCSS_SilentTxState)
  {
    if(wc.wasADCInt)
    {  // было прерывание от АЦП
        wc.wasADCInt = false;
        wc.timeUnitCounter++;
      if(wc.timeUnitCounter>=wc.allTimeUnits)
      { // переходим к следующему передатчику:
        wc.timeUnitCounter = 0;
        if(wc.isDirectCycle) // направленный цикл
        {
            wc.currTx = wc.TxByTurn[SetOfPresets.TBTIndex][wc.turnCounter][wc.indexTxByTurn];  // текущий номер передатчика
            wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
            apsMMode = APS_READY; //готовим сетку 1 передатчик ждет момента захвата 1 точки след. сектора (1 сектор запас)
            wc.bTxActive = false; //поэтому здесь он не включается
            wc.waitNextSector = true;
            wc.wasNewSectorInt = false;//!
        }
        else //ненаправленный цикл
        {
            wc.currTx = wc.Tx_undirect[wc.indexTxByTurn];
            wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
#ifndef EMUL
            CAN_Send_Broadcast(); //1-й передатчик
#endif
            wc.bTxActive = true;
        }
        wc.currSubState = DMCSS_IntermedialState;
      }
    }
  }

  else if(wc.currSubState==DMCSS_IntermedialState)
  {
        if(wc.wasADCInt==true)
        {  // было прерывание от АЦП
            wc.wasADCInt = false;
           if(wc.bTxActive == true)
           {
             wc.timeUnitCounter++;
             if(wc.timeUnitCounter>=wc.allTimeUnits) //время работы текущего передатчика вышло
             { // переходим к следующему передатчику:
               wc.bTxActive = false;
               if(wc.indexTxByTurn<6)
               {
                  wc.indexTxByTurn++;
               }
               else
               {
                   //оборот завершен?
               }
               if(wc.isDirectCycle)//---------------НАПРАВЛЕННЫЙ ЦИКЛ------------------------
               {
                 if((wc.indexTxByTurn<6)&&(wc.TxByTurn[SetOfPresets.TBTIndex][wc.turnCounter][wc.indexTxByTurn]!=7))
                 { // если новый передатчик существует:
                     wc.currTx = wc.TxByTurn[SetOfPresets.TBTIndex][wc.turnCounter][wc.indexTxByTurn];
                     wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
                     wc.timeUnitCounter = 0;
                     //wc.waitNextSector = true;
                     // посылаем команду на новый передатчик
#ifndef EMUL
                     CAN_Send_Broadcast();
#endif
                     wc.bTxActive = true;
                 }
                 else
                 { // кончился набор передатчиков для данного сектора, готовим под новый сектор
                   // Проверяем не кончились ли у нас сектора и обороты:
                   wc.sectorCounter++;
                   if(wc.sectorCounter>=SECTORS_COUNT || apsMMode == APS_COMPLETE)//todo учесть пропуск сектора
                   { // начинаем новый оборот
                       wc.sectorCounter = 0;
                       wc.turnCounter++;
                       if(apsMMode != APS_COMPLETE)
                           GPIO_writePin(led4, 1);
                       apsMMode = APS_READY;
                     if(wc.turnCounter>=wc.turnsCount)
                     { // закончили все обороты цикла, можно выходить на обработку
                         wc.turnCounter = 0;
                         apsMMode = APS_IDLE;
                         wc.wasNewSectorInt = false;
                         // Здесь собственно вываливаемся на обработку:
                         // Посылаем команду на окончание цикла:
                         wc.bFinishFlag = true;
                         wc.bTxActive = false;
#ifndef EMUL
                     CAN_Send_Broadcast();

                   // И переходим в состояние ожидания подготовки данных:
                     wc.currSubState = DMCSS_PreparingData;
#else
                     wc.currSubState = DMCSS_GettingData;
#endif
                     }
                     else
                     {  // закончили не все обороты цикла, переходим к следующиму обороту данного цикла:
                       wc.indexTxByTurn = 0;
                       wc.currTx = wc.TxByTurn[SetOfPresets.TBTIndex][wc.turnCounter][wc.indexTxByTurn];
                       wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
                       wc.timeUnitCounter = 0;
                       wc.waitNextSector = true;
                       // Состояние конечного автомата не изменяем!
                     }
                   }
                   else
                   {
                       wc.indexTxByTurn = 0;
                       wc.currTx = wc.TxByTurn[SetOfPresets.TBTIndex][wc.turnCounter][wc.indexTxByTurn];
                       wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
                       wc.timeUnitCounter = 0;
                       wc.waitNextSector = true;
                       wc.bTxActive = false;
                   }
                 }
               }
               else //---------------НЕНАПРАВЛЕННЫЙ ЦИКЛ------------------------
               { // если цикл ненаправленный, то окончание передатчиков
                 if((wc.indexTxByTurn<6)&&(wc.Tx_undirect[wc.indexTxByTurn]!=7))
                 { // если новый передатчик существует:
                     wc.currTx = wc.Tx_undirect[wc.indexTxByTurn];
                     wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
                     wc.timeUnitCounter = 0;
                     // посылаем команду на новый передатчик
                     wc.bTxActive = true;
#ifndef EMUL
                     CAN_Send_Broadcast();
#endif
                 }
                 else
                 {
                     wc.turnCounter = 0;
                   // Здесь собственно вываливаемся на обработку:
                   // Посылаем команду на окончание цикла:
                     wc.bFinishFlag = true;
                     wc.bTxActive = false;
#ifndef EMUL
                     CAN_Send_Broadcast();

                   // И переходим в состояние ожидания подготовки данных:
                     wc.currSubState = DMCSS_PreparingData;
#else
                     wc.currSubState = DMCSS_GettingData;
#endif
                 }
               }//endif isDirectCycle
             }//endif wc.timeUnitCounter>=wc.allTimeUnits
           }//endif txactive

           // Если было прерывание от инклинометра по новому сектору:
            if(wc.wasNewSectorInt && wc.isDirectCycle)
            {
              wc.wasNewSectorInt = false;
              if(wc.waitNextSector)
              {
                  wc.waitNextSector = false;
                  //GPIO_writePin(led4, 1); /////////////////////////////////////////////////////
                // Здесь посылаем команду на начало работы
#ifndef EMUL
                  CAN_Send_Broadcast();
#endif
                  wc.bTxActive = true;
              }
            }
          }//endif wasadcint
    // Если вышло время максимальной длительности цикла, то вываливаемся:
    if((wc.cycleStartTime_ms + MAX_CYCLE_TIME_MS) <= GetNow())
    {
        wc.turnCounter = 0;
      // Здесь собственно вываливаемся на обработку:
      // Посылаем команду на окончание цикла:
        wc.bFinishFlag = true;
        wc.bTxActive = false;
#ifndef EMUL
        CAN_Send_Broadcast();
        // И переходим в состояние ожидания подготовки данных:
        wc.currSubState = DMCSS_PreparingData;
        CPUTimer_setPeriod(TIMER_US_BASE, 0xFFFFFFFF);//1us
        CPUTimer_reloadTimerCounter(TIMER_US_BASE);
        CPUTimer_startTimer(TIMER_US_BASE);
#else
        wc.currSubState = DMCSS_GettingData;
#endif

    }
  }
  else if(wc.currSubState==DMCSS_PreparingData)
  {
    // ждем по времени и переходим в состояние приема данных из приемников :
    if(CPUTimer_getTimerCount(TIMER_US_BASE) < 0xFFFFFFFF - RX_PREPARETIME_US)
    {
        wc.currSubState = DMCSS_GettingData;
        //CPUTimer_stopTimer(TIMER_US_BASE); //1.12.2025
    }
  }
  else if(wc.currSubState==DMCSS_GettingData)
  {
      memset(&AllData[bFreq], 0x00, sizeof(struct ALLDATA));
      memset(&AllRxDataRAW, 0x00, sizeof(AllRxDataRAW));
      memset(&GPData, 0x00, sizeof(GPData));

      if(gWorkType == TYPE_CARTOGRAPH)
          AllData[bFreq].frame = frame++; //для картографа инктементируем фрейм тут

      AllData[bFreq].start_sector = gStartSectorIdx;
#ifndef EMUL
    // Принимаем последовательно данные со всех приемников:
      volatile uint16_t success;
      success = CAN_GetRxdata(1); //с 1 приемника получаем данные всегда
      if(success)
      {
          crc = CRC16(gCANBuffer, sizeof(struct undirect_RX)*2); //подсчет CRC
          crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
          if(crc == crc_rx)
              PackBuffer(gCANBuffer, (uint16_t *)&(AllData[bFreq].R_zz), sizeof(struct undirect_RX)); // было 40
          else
              GPIO_writePin(led4, 1);
      }

      if(wc.isDirectCycle)//если направленный цикл - забираем данные с 3 и 4 приемника
      {
          success = CAN_GetRxdata(3);
          if(success)
          {
              crc = CRC16(gCANBuffer, sizeof(struct direct_RX)*2); //подсчет CRC
              crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
              if(crc == crc_rx)
                  PackBuffer(gCANBuffer, (uint16_t *)&(AllData[bFreq].R_zx[0]), sizeof(struct direct_RX));
              else
                  GPIO_writePin(led4, 1);
          }

          success = CAN_GetRxdata(4);
          if(success)
          {
              crc = CRC16(gCANBuffer, sizeof(struct direct_RX)*2); //подсчет CRC
              crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение
              if(crc == crc_rx)
                  PackBuffer(gCANBuffer, (uint16_t *)&(AllData[bFreq].R_zx[1]), sizeof(struct direct_RX));
              else
                  GPIO_writePin(led4, 1);
          }

          //10.04.2026 DataValid - для телесистемы
          if(AllData[bFreq].R_zx[0].condition != 0 && AllData[bFreq].R_zx[1].condition != 0)
              AllData[bFreq].DataValid = 1;
          else
              AllData[bFreq].DataValid = 0;


          if(SetOfPresets.DebugSession & 0x0001)//Получение RAW data со всех направленных приемников
          {
             //3 приемник
            success = CAN_GetRxdataRAW(3, 1); //RX3 TX1
            if(success)
            {
              crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
              crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
              if(crc == crc_rx)
                  PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_L[0]), 64);
              else
                  GPIO_writePin(led4, 1);

              //AllData[bFreq].TXSectorCondition[0] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }

            success = CAN_GetRxdataRAW(3, 2); //RX3 TX2
            if(success)
            {
                crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
                crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
                if(crc == crc_rx)
                    PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_L[1]), 64);
                else
                    GPIO_writePin(led4, 1);

                //AllData[bFreq].TXSectorCondition[1] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }

            success = CAN_GetRxdataRAW(3, 4); //RX3 TX4
            if(success)
            {
                crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
                crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
                if(crc == crc_rx)
                    PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_L[2]), 64);
                else
                    GPIO_writePin(led4, 1);

                //AllData[bFreq].TXSectorCondition[2] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }

            success = CAN_GetRxdataRAW(3, 5); //RX3 TX5
            if(success)
            {
                crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
                crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
                if(crc == crc_rx)
                    PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_L[3]), 64);
                else
                    GPIO_writePin(led4, 1);

                //AllData[bFreq].TXSectorCondition[3] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }

             //4 приемник
            success = CAN_GetRxdataRAW(4, 1); //RX4 TX1
            if(success)
            {
              crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
              crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
              if(crc == crc_rx)
                  PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_R[0]), 64);
              else
                  GPIO_writePin(led4, 1);

              //AllData[bFreq].TXSectorCondition[4] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }
            success = CAN_GetRxdataRAW(4, 2); //RX4 TX2
            if(success)
            {
                crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
                crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
                if(crc == crc_rx)
                    PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_R[1]), 64);
                else
                    GPIO_writePin(led4, 1);

                //AllData[bFreq].TXSectorCondition[5] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }
            success = CAN_GetRxdataRAW(4, 3); //RX4 TX3
            if(success)
            {
                crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
                crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
                if(crc == crc_rx)
                    PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_R[2]), 64);
                else
                    GPIO_writePin(led4, 1);

                //AllData[bFreq].TXSectorCondition[6] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }
            success = CAN_GetRxdataRAW(4, 5); //RX4 TX5
            if(success)
            {
                crc = CRC16(gCANBuffer, sizeof(struct _16_sector_data)*2); //подсчет CRC
                crc_rx = (gCANBuffer[233]&0x00FF)|((gCANBuffer[234]&0x00ff)<<8)&0xFF00; //принятое значение CRC
                if(crc == crc_rx)
                    PackBuffer(gCANBuffer, (uint16_t *)&(AllRxDataRAW.Rx_R[3]), 64);
                else
                    GPIO_writePin(led4, 1);

                //AllData[bFreq].TXSectorCondition[7] = (gCANBuffer[235] + gCANBuffer[236]*256 + gCANBuffer[237]*65536 + gCANBuffer[238]*16777216);
            }
          }
      }
#else
      //emul
              AllData[bFreq].R_zz.Rzz1[0] = 1.10 + I*1.10;
              AllData[bFreq].R_zz.Rzz1[1] = 1.11 + I*1.11;
              AllData[bFreq].R_zz.Rzz1[2] = 1.12 + I*1.12;
              AllData[bFreq].R_zz.Rzz1[3] = 1.13 + I*1.13;
              AllData[bFreq].R_zz.Rzz2[0] = 2.10 + I*2.10;
              AllData[bFreq].R_zz.Rzz2[1] = 2.11 + I*2.11;
              AllData[bFreq].R_zz.Rzz2[2] = 2.12 + I*2.12;
              AllData[bFreq].R_zz.Rzz2[3] = 2.13 + I*2.13;
              AllData[bFreq].R_zz.temperature = 36.6;

              AllData[bFreq].R_zx[0].Tx_0 = 1.23456;
              AllData[bFreq].R_zx[0].temperature = 36.6;
              AllData[bFreq].R_zx[0].amp_Vzx[0] = 0.1;
              AllData[bFreq].R_zx[0].amp_Vzx[1] = 0.2;
              AllData[bFreq].R_zx[0].amp_Vzx[2] = 0.3;
              AllData[bFreq].R_zx[0].amp_Vzx[3] = 0.4;
              AllData[bFreq].R_zx[0].amp_Vzz[0] = 1.1;
              AllData[bFreq].R_zx[0].amp_Vzz[1] = 1.2;
              AllData[bFreq].R_zx[0].amp_Vzz[2] = 1.3;
              AllData[bFreq].R_zx[0].amp_Vzz[3] = 1.4;
              AllData[bFreq].R_zx[0].border_angle[0] = 11.11;
              AllData[bFreq].R_zx[0].border_angle[1] = 22.11;
              AllData[bFreq].R_zx[0].border_angle[2] = 33.11;
              AllData[bFreq].R_zx[0].border_angle[3] = 44.11;
              AllData[bFreq].R_zx[0].ph_Vzx_Vzz[0] = 1.0;
              AllData[bFreq].R_zx[0].ph_Vzx_Vzz[1] = 2.0;
              AllData[bFreq].R_zx[0].ph_Vzx_Vzz[2] = 3.0;
              AllData[bFreq].R_zx[0].ph_Vzx_Vzz[3] = 4.0;
              AllData[bFreq].R_zx[0].dv[0] = 11.0;
              AllData[bFreq].R_zx[0].dv[1] = 12.0;
              AllData[bFreq].R_zx[0].dv[2] = 13.0;
              AllData[bFreq].R_zx[0].dv[3] = 14.0;
              AllData[bFreq].R_zx[0].Geo[0] = 11.11 + I*11.111;
              AllData[bFreq].R_zx[0].Geo[1] = 12.11 + I*21.111;
              AllData[bFreq].R_zx[0].Geo[2] = 13.11 + I*31.111;
              AllData[bFreq].R_zx[0].Geo[3] = 14.11 + I*41.111;
              AllData[bFreq].R_zx[0].condition = 1;

              AllData[bFreq].R_zx[1].Tx_0 = 1.23456;
              AllData[bFreq].R_zx[1].temperature = 36.6;
              AllData[bFreq].R_zx[1].amp_Vzx[0] = 0.1;
              AllData[bFreq].R_zx[1].amp_Vzx[1] = 0.2;
              AllData[bFreq].R_zx[1].amp_Vzx[2] = 0.3;
              AllData[bFreq].R_zx[1].amp_Vzx[3] = 0.4;
              AllData[bFreq].R_zx[1].amp_Vzz[0] = 1.1;
              AllData[bFreq].R_zx[1].amp_Vzz[1] = 1.2;
              AllData[bFreq].R_zx[1].amp_Vzz[2] = 1.3;
              AllData[bFreq].R_zx[1].amp_Vzz[3] = 1.4;
              AllData[bFreq].R_zx[1].border_angle[0] = 11.11;
              AllData[bFreq].R_zx[1].border_angle[1] = 22.11;
              AllData[bFreq].R_zx[1].border_angle[2] = 33.11;
              AllData[bFreq].R_zx[1].border_angle[3] = 44.11;
              AllData[bFreq].R_zx[1].ph_Vzx_Vzz[0] = 1.0;
              AllData[bFreq].R_zx[1].ph_Vzx_Vzz[1] = 2.0;
              AllData[bFreq].R_zx[1].ph_Vzx_Vzz[2] = 3.0;
              AllData[bFreq].R_zx[1].ph_Vzx_Vzz[3] = 4.0;
              AllData[bFreq].R_zx[1].dv[0] = 11.0;
              AllData[bFreq].R_zx[1].dv[1] = 12.0;
              AllData[bFreq].R_zx[1].dv[2] = 13.0;
              AllData[bFreq].R_zx[1].dv[3] = 14.0;
              AllData[bFreq].R_zx[1].Geo[0] = 11.21 + I*12.111;
              AllData[bFreq].R_zx[1].Geo[1] = 12.21 + I*22.111;
              AllData[bFreq].R_zx[1].Geo[2] = 13.21 + I*32.111;
              AllData[bFreq].R_zx[1].Geo[3] = 14.21 + I*42.111;
              AllData[bFreq].R_zx[1].condition = 1;

              AllData[bFreq].phase_smt[0] = 1.1;
              AllData[bFreq].phase_smt[1] = 2.1;
              AllData[bFreq].phase_smt[2] = 3.1;
              AllData[bFreq].phase_smt[3] = 4.1;

              AllData[bFreq].rho_smt[0] = 2.1;
              AllData[bFreq].rho_smt[1] = 2.2;
              AllData[bFreq].rho_smt[2] = 2.3;
              AllData[bFreq].rho_smt[3] = 2.4;
              AllData[bFreq].signature = Metro.signature;
              AllData[bFreq].INC.Aps = MTF_deg;
              AllData[bFreq].INC.Azm = angle_azm_deg;
              AllData[bFreq].INC.Zen = angle_zen_deg;
              AllData[bFreq].dds_freq = wc.currFreq;
              AllData[bFreq].Wg = -fWgAtStart;

              AllData[bFreq].ATT_dB_geo_signal_smt[0] = 10.0;
              AllData[bFreq].ATT_dB_geo_signal_smt[1] = 20.0;
              AllData[bFreq].ATT_dB_geo_signal_smt[2] = 30.0;
              AllData[bFreq].ATT_dB_geo_signal_smt[3] = 40.0;

              AllData[bFreq].PH_deg_geo_signal_smt[0] = 11.1;
              AllData[bFreq].PH_deg_geo_signal_smt[1] = 22.2;
              AllData[bFreq].PH_deg_geo_signal_smt[2] = 33.3;
              AllData[bFreq].PH_deg_geo_signal_smt[3] = 44.4;

            //raw
            for(uint16_t i=0;i<16;i++)
            {
                for(uint16_t j=0;j<4; j++)
                {
                    AllRxDataRAW.Rx_L[j].Im_sector[i] = i*0.1;
                    AllRxDataRAW.Rx_L[j].Re_sector[i] = i*0.2;
                    AllRxDataRAW.Rx_R[j].Im_sector[i] = i*0.3;
                    AllRxDataRAW.Rx_R[j].Re_sector[i] = i*0.4;
                }
            }

            //gp_data


#endif
      AllData[bFreq].signature = Metro.signature;
      AllData[bFreq].INC.Aps = MTF_deg;
      AllData[bFreq].INC.Azm = angle_azm_deg;
      AllData[bFreq].INC.Zen = angle_zen_deg;
      AllData[bFreq].dds_freq = wc.currFreq;
      AllData[bFreq].Wg = fWgAtStart; //-

      //condition = GetTXCondition(&AllData[bFreq], &Metro, bFreq);
      //AllData[bFreq].R_zz.condition = condition; //27.07.26
      //simmetry(&AllData[bFreq], &Metro, bFreq, condition, nTx);  //-?
      struct ID id = get_sonde_id(Metro.signature);
      condition = GetTXCondition(&AllData[bFreq], &Metro, bFreq);
      simmetry(&AllData[bFreq], &Metro, bFreq, condition, id.N_Tx);
      for(int Tx = 0; Tx < 4; Tx++)
      {
          RO_ARG(&Metro, &AllData[bFreq], Tx, bFreq);
          RO_ATT(&Metro, &AllData[bFreq], Tx, bFreq);
      }

      //calc_geo_smt(&AllData[bFreq]);
      calc_geo_signal_smt(&Metro, &AllData[bFreq], bFreq);
      //---------------------------------------------
      OutCompressedData(&AllData[0], &Metro);

      if(gWorkType == TYPE_CARTOGRAPH)
      {
           wc.currSubState = DMCSS_DONE;
           GPIO_writePin(led2, 0);
      }
      else if(gWorkType == TYPE_LWD_4TX)//в этом режиме 2 цикла на 2 частотах всегда
      {
          if(bFreq == 0) //будем делать еще 1 цикл на 2 частоте
              wc.currSubState = DMCSS_Perform2ndCycle;
          else //2 цикла завершено
          {
              transform_data(&AllData[0], &GPData);
              GPData.frame = frame;
              wc.currSubState = DMCSS_DONE;
              frame++;
              GPIO_writePin(led2, 0);
          }
      }
      else
      {
          // пока не поддерживаемый тип
      }
  }
  else if(wc.currSubState==DMCSS_Perform2ndCycle)
  {
      if(wc.wasADCInt==true)
      {
          wc.wasADCInt = false;
          //задержка между частотами при LWD 4 TX
          if(wc.delay2ndCycle > 0)
          {
              wc.delay2ndCycle--;
          }
          else
          {
              //в режиме простого индукционника начинаем еще 1 цикл измерений на 2 частоте
              wc.inclinoSector = 0;
              wc.sectorCounter = 0;
              wc.turnsCount = 0;
              wc.turnCounter = 0;
              wc.indexTxByTurn = 0; // для молчащих передатчиков это не важно
              wc.currTx = 0; // начинаем всегда с молчащих передатчиков
              wc.timeUnitCounter = 0;
              wc.prepareDataTimeMcsCounter = 0;
              wc.waitNextSector = false;
              wc.wasNewSectorInt = false;
              wc.wasADCInt = false;
              wc.bFinishFlag = false;
              wc.bTxActive = false;
              wc.currFreq = Metro.F[1];
              bFreq = 1;
              wc.allTimeUnits = wc.TxWorkTime[wc.currTx]+TX_ADD_TIME;
              wc.cycleStartTime_ms = GetNow(); // здесь начали считать время цикла;
#ifndef EMUL
              CAN_Send_Broadcast(); //0-й молчащий передатчик
#endif
              wc.bTxActive = true;
              wc.currSubState = DMCSS_SilentTxState;
          }
      }
  }
  else
  {
      // Ничего не делаем
  }
}





