#include "_globals.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "uart.h"
#include <stdio.h>
#include "CRC16ModBus.h"
#include <string.h>

uint16_t TxBufferExt[4096];
enum
{
    EXT_CMD_GETDATA = 0x11,
    EXT_CMD_SET_METRO = 0x12,
    EXT_CMD_GET_METRO = 0x13,
    EXT_CMD_SET_PRESETS = 0x14,
    EXT_CMD_GET_PRESETS = 0x15,
    EXT_CMD_GET_SIGNATURE = 0x16,
    EXT_CMD_SET_KEY = 0x17,
    EXT_CMD_ENTER_BOOTLOADER = 0x18,
    EXT_CMD_SET_WORKTYPE = 0x19,
    EXT_CMD_SET_ZERO = 0x1A
};
struct SERVICE { //16
   uint16_t CmdCode; // 0x1111
   uint16_t bPeriodicStart;
   uint16_t Speed; // об/мин
   uint16_t SkipSector; //побитно: LSB=сектор0 MSB=сектор15, 1=пропуск 0=нет пропуска
   //uint64_t dummy64; //не исп
   float MTF;
   float dummy;
};
struct SERVICE svc;
extern uint16_t bPeriodicStart;
extern struct Vec GBuff, MBuff, WBuff; //для печати
struct TGMW GMW;

//------------------------------------------------------------------------------------
//универсальная функция для передачи данных
//pData-указатель на данные; words-размер данных в словах; cmdcode-первый байт при передаче
void TransmitDataExt(uint16_t *pData, uint16_t words, uint16_t cmdcode) //посылает в UART (SCI C) данные и вставляет 5 байт: cmdcode; cmdsize(1:0); data(...); crc(0:1)
{
    memset(TxBufferExt, 0, sizeof(TxBufferExt));
    uint16_t cmdsize = 0;
    uint16_t idx = 3;
    uint16_t crc = 0;
    TxBufferExt[0] = cmdcode;
    for(uint16_t i=0; i<words; i++)
    {
        TxBufferExt[idx] = pData[i]&0x00FF;
        idx++;
        TxBufferExt[idx] = (pData[i]>>8)&0x00FF;
        idx++;
    }
    cmdsize = idx+2;
    TxBufferExt[1] = cmdsize & 0x00FF; ; //длина мл
    TxBufferExt[2] = (cmdsize >> 8) & 0x00FF ; //длина ст
    crc = CRC16(TxBufferExt, idx); //idx = количеству байт до CRC тут
    TxBufferExt[idx+1] = crc&0xFF;
    TxBufferExt[idx] = (crc>>8)&0xFF;

    for(int i=0;i<(idx+2);i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, TxBufferExt[i]);
    }
}
//------------------------------------------------------------------------------------
void TransmitDataIncluded(uint16_t *pData, uint16_t words, uint16_t inccmdcode) //тоже, через вложенную команду
{
    memset(TxBufferExt, 0, sizeof(TxBufferExt));
    uint16_t cmdsize = 0;
    uint16_t idx = 5;
    uint16_t crc = 0;
    TxBufferExt[0] = EXT_CMD_SET_PRESETS;
    for(uint16_t i=0; i<words; i++)
    {
        TxBufferExt[idx] = pData[i]&0x00FF;
        idx++;
        TxBufferExt[idx] = (pData[i]>>8)&0x00FF;
        idx++;
    }
    cmdsize = idx+2;
    TxBufferExt[1] = cmdsize & 0x00FF; ; //длина мл
    TxBufferExt[2] = (cmdsize >> 8) & 0x00FF ; //длина ст
    TxBufferExt[3] = inccmdcode & 0x00FF; ; //код вложенной ком мл
    TxBufferExt[4] = (inccmdcode >> 8) & 0x00FF ; //код вложенной ком ст
    crc = CRC16(TxBufferExt, idx); //idx = количеству байт до CRC тут
    TxBufferExt[idx+1] = crc&0xFF;
    TxBufferExt[idx] = (crc>>8)&0xFF;

    for(int i=0;i<(idx+2);i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, TxBufferExt[i]);
    }
}
//------------------------------------------------------------------------------------
void SendAllDataExt(struct ALLDATA *pAllData, struct raw_sector_data_all *pRawData)
{
    volatile int sz1, sz2;
    sz1 = sizeof(struct ALLDATA);
    //if(SetOfPresets.DebugSession)
    sz2 = sizeof(struct raw_sector_data_all);

    volatile uint16_t idx = 3;
    uint16_t len = 0; //общая длина пакета, включая CRC
    uint16_t crc = 0;
    uint16_t *ptr = (uint16_t *)pAllData;

    //dbg//////////////////////////////////////
    /*for(uint16_t i=0; i<sizeof(struct ALLDATA); i++)
    {
        ptr[i] = (2*i) & 0x00FF; //мл
        ptr[i] |= ((2*i + 1) & 0x00FF) << 8; //ст
    }

    ptr = (uint16_t *)pRawData;
    for(uint16_t i=0; i<sizeof(struct raw_sector_data_all); i++)
    {
        ptr[i] = (2*i) & 0x00FF; //мл
        ptr[i] |= ((2*i + 1) & 0x00FF) << 8; //ст
    }*/
    //////////////////////////////////////////
    ptr = (uint16_t *)pAllData;
    memset(TxBufferExt, 0, sizeof(TxBufferExt));
    TxBufferExt[0] = EXT_CMD_GETDATA; //0x11
    //TxBufferExt[1] =  ;
    //TxBufferExt[2] =  ;
    volatile uint16_t tmp = 0;
    for(uint16_t i=0; i<sz1; i++)
    {
        TxBufferExt[idx] = ptr[i]&0x00FF;
        idx++;
        TxBufferExt[idx] = (ptr[i]>>8)&0x00FF;
        idx++;
    }

    ptr = (uint16_t *)pRawData;
    if(SetOfPresets.DebugSession & 0x0001)
    {
       for(uint16_t i=0; i<sz2; i++)
        {
            TxBufferExt[idx] = ptr[i]&0x00FF;
            idx++;
            TxBufferExt[idx] = (ptr[i]>>8)&0x00FF;
            idx++;
        }
    }
    len = idx + 2;
    TxBufferExt[1] = len & 0x00FF; ; //длина мл
    TxBufferExt[2] = (len >> 8) & 0x00FF ; //длина ст
    crc = CRC16(TxBufferExt, idx); //idx = количеству байт до CRC тут
    TxBufferExt[idx+1] = crc&0xFF;
    TxBufferExt[idx] = (crc>>8)&0xFF;

    for(int i=0;i<(idx+2);i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, TxBufferExt[i]);
    }
}
//------------------------------------------------------------------------------------
void SendGPDataExt(struct GP_DATA *pGPData)
{
    volatile int sz, sz1, sz2;
    sz = sizeof(struct GP_DATA);
    sz1 = sizeof(struct ALLDATA);
    sz2 = sizeof(struct raw_sector_data_all);
    volatile uint16_t idx = 3;
    uint16_t len = 0; //общая длина пакета, включая CRC
    uint16_t crc = 0;
    uint16_t *ptr = (uint16_t *)pGPData;

    memset(TxBufferExt, 0, sizeof(TxBufferExt));
    TxBufferExt[0] = EXT_CMD_GETDATA; //0x11
    //в начало буфера пишем GP_DATA
    for(uint16_t i=0; i<sz; i++)
    {
        TxBufferExt[idx] = ptr[i]&0x00FF;
        idx++;
        TxBufferExt[idx] = (ptr[i]>>8)&0x00FF;
        idx++;
    }

    /*sz = sz2 + sz1 - sizeof(struct GP_DATA); //добиваем нулями (RAW+ALL)-GP
    for(uint16_t i=0; i<sz; i++)
    {
        TxBufferExt[idx] = 0;
        idx++;
        TxBufferExt[idx] = 0;
        idx++;
    }*/

    len = idx + 2;
    TxBufferExt[1] = len & 0x00FF; ; //длина мл
    TxBufferExt[2] = (len >> 8) & 0x00FF ; //длина ст
    crc = CRC16(TxBufferExt, idx); //idx = количеству байт до CRC тут
    TxBufferExt[idx+1] = crc&0xFF;
    TxBufferExt[idx] = (crc>>8)&0xFF;

    for(int i=0;i<(idx+2);i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, TxBufferExt[i]);
    }
}
//------------------------------------------------------------------------------------
void SendMetroExt(struct METROLOGY_CARTOGRAPH *pM)
{
    volatile int sz; //words
    volatile uint16_t idx = 3;
    uint16_t len = 0; //для записи в поле "длина" в байтах. Длина = количество ВСЕХ байт пакета
    uint16_t crc = 0;
    uint16_t *ptr = (uint16_t *)pM;
    memset(TxBufferExt, 0, sizeof(TxBufferExt));
    if(gWorkType == TYPE_CARTOGRAPH) //картограф
    {
        sz = sizeof(struct METROLOGY_CARTOGRAPH);
        len = sz*2 + 5; //размер структуры в байтах + cmdcode + len lsb + len msb + crc msb + crc lsb
        TxBufferExt[0] = EXT_CMD_GET_METRO; //0x13
        TxBufferExt[1] = len & 0x00FF; //lsb
        TxBufferExt[2] = (len >> 8) & 0x00FF; //msb
        for(uint16_t i=0; i<sz; i++)
        {
            TxBufferExt[idx] = ptr[i]&0x00FF;
            idx++;
            TxBufferExt[idx] = (ptr[i]>>8)&0x00FF;
            idx++;
        }
        crc = CRC16(TxBufferExt, idx); //idx = количеству байт до CRC тут
        TxBufferExt[idx+1] = crc&0xFF;
        TxBufferExt[idx] = (crc>>8)&0xFF;
    }
    else if(gWorkType == TYPE_LWD_4TX) //индукционник
    {
        sz = sizeof(struct METROLOGY_GP);
        len = sz*2 + 5;
        TxBufferExt[0] = EXT_CMD_GET_METRO; //0x13
        TxBufferExt[1] = len & 0x00FF; //lsb
        TxBufferExt[2] = (len >> 8) & 0x00FF; //msb
        for(uint16_t i=0; i<sz; i++)
        {
            TxBufferExt[idx] = ptr[i]&0x00FF;
            idx++;
            TxBufferExt[idx] = (ptr[i]>>8)&0x00FF;
            idx++;
        }
        /*sz = 119 - sizeof(struct METROLOGY_GP);
        for(uint16_t i=0; i<sz; i++)
        {
            TxBufferExt[idx] = 0x00;
            idx++;
            TxBufferExt[idx] = 0x00;
            idx++;
        }*/
        crc = CRC16(TxBufferExt, idx); //idx = количеству байт до CRC тут
        TxBufferExt[idx+1] = crc & 0x00FF;
        TxBufferExt[idx] = (crc>>8) & 0x00FF;
    }
    else
    {
        return; //неизвестный тип
    }

    for(int i=0;i<(idx+2);i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, TxBufferExt[i]);
    }
}
//------------------------------------------------------------------------------------
void SendSetOfPresetsExt(struct SET_OF_PRESETS *pS)
{
    volatile int sz = sizeof(struct SET_OF_PRESETS); //8
    volatile uint16_t idx = 3;
    uint16_t len = 0;
    uint16_t crc = 0;
    uint16_t *ptr = (uint16_t *)pS;
    memset(TxBufferExt, 0, sizeof(TxBufferExt));
    len = sz*2 + 5;
    TxBufferExt[0] = EXT_CMD_GET_PRESETS;
    TxBufferExt[1] = len & 0x00FF; //lsb
    TxBufferExt[2] = (len >> 8) & 0x00FF; //msb

    for(uint16_t i=0; i<sz; i++)
    {
        TxBufferExt[idx] = ptr[i]&0x00FF;
        idx++;
        TxBufferExt[idx] = (ptr[i]>>8)&0x00FF;
        idx++;
    }
    crc = CRC16(TxBufferExt, 19);
    TxBufferExt[idx+1] = crc&0xFF;
    TxBufferExt[idx] = (crc>>8)&0xFF;

    for(int i=0;i<len;i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, TxBufferExt[i]);
    }
}
//------------------------------------------------------------------------------------
void SendSignatureExt(uint32_t signature)
{
    uint16_t crc = 0;
    memset(TxBufferExt, 0, sizeof(TxBufferExt));
    //cmd code
    TxBufferExt[0] = EXT_CMD_GET_SIGNATURE;
    //len
    TxBufferExt[1] = 9;//len lsb
    TxBufferExt[2] = 0;//len msb
    //сигнатура 4 байта старшим вперед
    TxBufferExt[3] = Metro.signature & 0x000000FF; //lsb
    TxBufferExt[4] = (Metro.signature >> 8) & 0x000000FF;
    TxBufferExt[5] = (Metro.signature >> 16) & 0x000000FF;
    TxBufferExt[6] = (Metro.signature >> 24) & 0x000000FF; //msb
    //crc
    crc = CRC16(TxBufferExt, 7);
    TxBufferExt[8] = crc&0xFF;
    TxBufferExt[7] = (crc>>8)&0xFF;

    for(int i=0;i<9;i++)
    {
        SCI_writeCharBlockingNonFIFO(SCIC_BASE, TxBufferExt[i]);
    }
}
//------------------------------------------------------------------------------------
void ParseExtCmd()
{
    uint16_t ticket[64];
    uint16_t crc = 0;
    volatile uint16_t sz = 0;
    //bool air_write_flg = true; //
    uint16_t bFreq;
    float KWCorr = 1.0;

    if(wc.currFreq >= 1000)
        bFreq = 1;
    else
        bFreq = 0;

    if(UartCmdExt.ready == true)
    {
        GPIO_writePin(led1, 1);
        UartCmdExt.ready = false;
        if(wc.currSubState != DMCSS_DONE) //!!!!!!!!!!!!!!
        {
            GPIO_writePin(led3, 1); //ERROR Wlim, ERROR Metro
            delay_ms(10); //dbg
            GPIO_writePin(led1, 0);
            return;
        }

        if(UartCmdExt.buf[0] == EXT_CMD_GETDATA)
        {
            if(gWorkType == TYPE_CARTOGRAPH)
                SendAllDataExt(&AllData[bFreq], &AllRxDataRAW);
            else if(gWorkType == TYPE_LWD_4TX)
                SendGPDataExt(&GPData);
            else //Error
            {
                gErrorCnt++;
                GPIO_writePin(led3, 1);
                delay_ms(200);
                GPIO_writePin(led1, 0);
                return;
            }
            gExtPktsAns++;
            TOneWorkCycle_Create();
            bExtCmdGetData = true;
            GPIO_writePin(led2, 0);
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_SET_METRO)
        {
            if(gWorkType == TYPE_CARTOGRAPH && UartCmdExt.len ==(sizeof(Metro)*2+5)) //1536+5
            {
                sz = sizeof(struct METROLOGY_CARTOGRAPH);
                PackBuffer((uint16_t*)(&(UartCmdExt.buf[3])), (uint16_t*)(&Metro), sz);
            }
            else if(gWorkType == TYPE_LWD_4TX && UartCmdExt.len == (sizeof(MetroGp)*2+5)) //245
            {
                sz = sizeof(struct METROLOGY_GP) + 1;
                PackBuffer((uint16_t*)(&(UartCmdExt.buf[3])), (uint16_t*)(&Metro), sz);
            }
            else
            {
                gErrorCnt++;
                GPIO_writePin(led3, 1);
                GPIO_writePin(led1, 0);
                return;
            }
            //нули метрологии
            /*air_write_flg = true; //сброс
            for(int freq=0; freq<2; freq++)
            {
                for(int n=0; n<4; n++)
                {
                    if(Metro.air_ph[freq][n] != 0)
                    {
                        air_write_flg = false;
                    }
                }
            }
            //заменяем нули воздуха в старой метрологии
            if(air_write_flg == true) //пишем старую метрологию с измененными нулями воздуха
            {
                GPIO_writePin(led1, 1);
                GPIO_writePin(led2, 1);
                GPIO_writePin(led3, 1);
                GPIO_writePin(led4, 1);
                for(int freq_idx = 0; freq_idx < 2; freq_idx++)
                {
                    for (int Tx = 0; Tx < 4; Tx++)
                    {
                        Metro.air_ph[freq_idx][Tx] = (int16_t)((air_summ[freq_idx][PH][Tx]/air_aver)*57297.0); //-?
                        Metro.air_att_dB[freq_idx][Tx] = air_summ[freq_idx][ATT][Tx]/air_aver; //-?
                    }
                }
            }*/

            if(CheckMetroData() == 1)
            {
               //CPUTimer_disableInterrupt(TIMER0_1MS_BASE);
               //controller_state = STATE_IDLE;
               FLASH_WriteMetrology(&Metro);
               delay_ms(20);
               GPIO_writePin(led1, 0);
               GPIO_writePin(led2, 0);
               GPIO_writePin(led3, 0);
               GPIO_writePin(led4, 0);
               //CPUTimer_enableInterrupt(TIMER0_1MS_BASE);
               //controller_state = STATE_NORMAL;

               ticket[0] = EXT_CMD_SET_METRO;
               ticket[1] = 5;
               ticket[2] = 0;
               crc = CRC16(ticket, 3);
               ticket[4] = crc&0xFF;
               ticket[3] = (crc>>8)&0xFF;
               SendData16Ext(ticket, 5); //шлем квитанцию
            }
            else
            {
                GPIO_writePin(led1, 0);
                GPIO_writePin(led2, 0);
                GPIO_writePin(led3, 1);
                GPIO_writePin(led4, 0);
                FLASH_ReadMetrology(&Metro); //загружаем старую метрологию из флеша
                if(CheckMetroData() == 0)
                {
                  controller_state = STATE_IDLE;
                  DefaultMetro();
                }
            }
            //сброс
            memset(&AllData[bFreq], 0x00, sizeof(struct ALLDATA));
            memset(&AllRxDataRAW, 0x00, sizeof(AllRxDataRAW));
            memset(&GPData, 0x00, sizeof(GPData));
            frame = 1;
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_SET_KEY)
        {
            //CPUTimer_disableInterrupt(TIMER0_1MS_BASE);
            //controller_state = STATE_IDLE;
            GPIO_writePin(led1, 1);
            GPIO_writePin(led2, 1);
            GPIO_writePin(led3, 1);
            GPIO_writePin(led4, 1);

            Metro.select_key = 0;
            Metro.select_key = (uint32_t)UartCmdExt.buf[3];
            Metro.select_key |= (uint32_t)(UartCmdExt.buf[4]) << 8;
            Metro.select_key |= (uint32_t)(UartCmdExt.buf[5]) << 16;
            Metro.select_key |= (uint32_t)(UartCmdExt.buf[6]) << 24;

            if(Metro.select_key & 0x3000000)
            {
               GPIO_writePin(led3, 0);
               FLASH_WriteMetrology(&Metro);
               ticket[0] = EXT_CMD_SET_KEY;
               ticket[1] = 5;
               ticket[2] = 0;
               crc = CRC16(ticket, 3);
               ticket[4] = crc&0xFF;
               ticket[3] = (crc>>8)&0xFF;
               SendData16Ext(ticket, 5);
            }
            else
            {
                GPIO_writePin(led3, 1);
                delay_ms(100);
            }
            GPIO_writePin(led1, 0);
            GPIO_writePin(led2, 0);
            GPIO_writePin(led4, 0);
            //CPUTimer_enableInterrupt(TIMER0_1MS_BASE);
            //controller_state = STATE_NORMAL;
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_GET_METRO)
        {
            SendMetroExt(&Metro);
        }
        ///PRESETS + SERVICE COMMANDS/////////////////////////////////////////////////////////////////
        else if(UartCmdExt.buf[0] == EXT_CMD_SET_PRESETS) //set of presets write
        {
            //CPUTimer_disableInterrupt(TIMER0_1MS_BASE);
            //controller_state = STATE_IDLE;

            if(UartCmdExt.buf[3] == 0xA5 && UartCmdExt.buf[4] == 0x5A)
            {
                PackBuffer((uint16_t*)(&(UartCmdExt.buf[3])), (uint16_t*)(&SetOfPresets), sizeof(struct SET_OF_PRESETS));
                SetOfPresets.Signature = 0x5AA5; //todo: сигнатура всегда одинакова
                FLASH_WriteSetOfPresets(&SetOfPresets);
                gEmulRotateSpeed = (SetOfPresets.DebugSession >> 8)&0xFF;
                ticket[0] = EXT_CMD_SET_PRESETS;
                ticket[1] = 5;
                ticket[2] = 0;
                crc = CRC16(ticket, 3);
                ticket[4] = crc&0xFF;
                ticket[3] = (crc>>8)&0xFF;
                SendData16Ext(ticket, 5);
            }
            else if(UartCmdExt.buf[3] == 0x11 && UartCmdExt.buf[4] == 0x11) //без сохранения
            {
                PackBuffer((uint16_t*)(&(UartCmdExt.buf[3])), (uint16_t*)(&svc), sizeof(struct SERVICE));
                gEmulRotateSpeed = svc.Speed;
                if(gEmulRotateSpeed > 5000)
                    gEmulRotateSpeed = 5000;

                bPeriodicStart = svc.bPeriodicStart;

                ticket[0] = EXT_CMD_SET_PRESETS;
                ticket[1] = 5;
                ticket[2] = 0;
                crc = CRC16(ticket, 3);
                ticket[4] = crc&0xFF;
                ticket[3] = (crc>>8)&0xFF;
                SendData16Ext(ticket, 5);
            }
            else if(UartCmdExt.buf[3] == 0x11 && UartCmdExt.buf[4] == 0x00) //W ZERO
            {
                W_offset_sens.offset.X = W_raw.X;
                W_offset_sens.offset.Y = W_raw.Y;
                W_offset_sens.offset.Z = W_raw.Z;
                FLASH_WriteCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
                ticket[0] = EXT_CMD_SET_PRESETS;
                ticket[1] = 5;
                ticket[2] = 0;
                crc = CRC16(ticket, 3);
                ticket[4] = crc&0xFF;
                ticket[3] = (crc>>8)&0xFF;
                SendData16Ext(ticket, 5);
            }
            else if(UartCmdExt.buf[3] == 0x11 && UartCmdExt.buf[4] == 0x01) //W SensZZ
            {
                PackBuffer((uint16_t*)(&(UartCmdExt.buf[5])), (uint16_t*)(&KWCorr), sizeof(float));
                if(KWCorr > 999.0) //сброс
                {
                    W_offset_sens.offset.X = 0;
                    W_offset_sens.offset.Y = 0;
                    W_offset_sens.offset.Z = 0;
                    W_offset_sens.sens.XX = 0.001221726;
                    W_offset_sens.sens.YY = 0.001221726;
                    W_offset_sens.sens.ZZ = 0.001221726;
                    //W_offset_sens = { 0,0,0,0.001221726,0,0,0,0.001221726,0,0,0,0.001221726 };
                }
                else
                {
                   W_offset_sens.sens.ZZ *= KWCorr;
                }
               //FLASH_WriteCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
                ticket[0] = EXT_CMD_SET_PRESETS;
                ticket[1] = 5;
                ticket[2] = 0;
                crc = CRC16(ticket, 3);
                ticket[4] = crc&0xFF;
                ticket[3] = (crc>>8)&0xFF;
                SendData16Ext(ticket, 5);
            }
            else if(UartCmdExt.buf[3] == 0xAA && UartCmdExt.buf[4] == 0xAA) //GET Speed
            {
                CORRECTION_TIME = 1000000000; //время коррекции увеличено
                uint32_t tmp32 = 0;
                float wtm = Wg*30.0/M_PI;
                ticket[0] = EXT_CMD_SET_PRESETS;
                ticket[1] = 11;
                ticket[2] = 0;
                ticket[3] = 0xAA; //int cmd code
                ticket[4] = 0xAA; //int cmd code
                //w
                memcpy(&tmp32, &wtm, sizeof(wtm));
                ticket[5] = tmp32 & 0x000000FF;
                ticket[6] = (tmp32 >> 8) & 0x000000FF;
                ticket[7] = (tmp32 >> 16) & 0x000000FF;
                ticket[8] = (tmp32 >> 24) & 0x000000FF;
                //mtf
                memcpy(&tmp32, &MTF_deg, sizeof(MTF_deg));
                ticket[9] = tmp32 & 0x000000FF;
                ticket[10] = (tmp32 >> 8) & 0x000000FF;
                ticket[11] = (tmp32 >> 16) & 0x000000FF;
                ticket[12] = (tmp32 >> 24) & 0x000000FF;
                //gtf
                memcpy(&tmp32, &angle_aps_deg, sizeof(angle_aps_deg));
                ticket[13] = tmp32 & 0x000000FF;
                ticket[14] = (tmp32 >> 8) & 0x000000FF;
                ticket[15] = (tmp32 >> 16) & 0x000000FF;
                ticket[16] = (tmp32 >> 24) & 0x000000FF;

                crc = CRC16(ticket, 17);
                ticket[18] = crc&0xFF;
                ticket[17] = (crc>>8)&0xFF;
                SendData16Ext(ticket, 19);
            }
            else if(UartCmdExt.buf[3] == 0xAA && UartCmdExt.buf[4] == 0xBB) //GMW GET point
            {
                GMW.G = GBuff;
                GMW.M = MBuff;
                GMW.W = WBuff;
                TransmitDataIncluded((uint16_t*)&GMW, sizeof(GMW), 0xBBAA);
            }
            else if(UartCmdExt.buf[3] == 0xAA && UartCmdExt.buf[4] == 0xCA) //GET AXEL Calibration
            {
                TransmitDataIncluded((uint16_t*)&G_offset_sens, sizeof(G_offset_sens), 0xCAAA);
            }
            else if(UartCmdExt.buf[3] == 0xAA && UartCmdExt.buf[4] == 0xDA) //SET AXEL Calibration
            {
                PackBuffer((uint16_t*)&UartCmdExt.buf[5], (uint16_t*)&G_offset_sens, sizeof(G_offset_sens));
                FLASH_WriteCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);

                ticket[0] = EXT_CMD_SET_PRESETS;
                ticket[1] = 5;
                ticket[2] = 0;
                crc = CRC16(ticket, 3);
                ticket[4] = crc&0xFF;
                ticket[3] = (crc>>8)&0xFF;
                SendData16Ext(ticket, 5);
            }
            else if(UartCmdExt.buf[3] == 0xAA && UartCmdExt.buf[4] == 0xCC) //GET Magn Calibration
            {
                TransmitDataIncluded((uint16_t*)&M_offset_sens, sizeof(M_offset_sens), 0xCCAA);
            }
            else if(UartCmdExt.buf[3] == 0xAA && UartCmdExt.buf[4] == 0xDD) //SET Magn Calibration
            {

                PackBuffer((uint16_t*)&UartCmdExt.buf[5], (uint16_t*)&M_offset_sens, sizeof(M_offset_sens));
                FLASH_WriteCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);

                ticket[0] = EXT_CMD_SET_PRESETS;
                ticket[1] = 5;
                ticket[2] = 0;
                crc = CRC16(ticket, 3);
                ticket[4] = crc&0xFF;
                ticket[3] = (crc>>8)&0xFF;
                SendData16Ext(ticket, 5);
            }

            gWlim = GetWlim();
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_GET_PRESETS)
        {
            SendSetOfPresetsExt(&SetOfPresets);
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_GET_SIGNATURE)
        {
            SendSignatureExt(Metro.signature);
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_ENTER_BOOTLOADER)
        {
            ticket[0] = EXT_CMD_ENTER_BOOTLOADER;
            ticket[1] = 5;
            ticket[2] = 0;
            crc = CRC16(ticket, 3);
            ticket[4] = crc&0xFF;
            ticket[3] = (crc>>8)&0xFF;
            SendData16Ext(ticket, 5);

            CPUTimer_disableInterrupt(TIMER0_1MS_BASE);
            controller_state = STATE_IDLE;
            FLASH_Erase(PAGE_AB);
            SysCtl_enableWatchdog(); //СБРОС
            while(true);
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_SET_WORKTYPE)
        {
            frame = 1;
            uint32_t profile = 0;
            profile = (uint32_t)UartCmdExt.buf[3];
            profile |= (uint32_t)(UartCmdExt.buf[4]) << 8;
            profile |= (uint32_t)(UartCmdExt.buf[5]) << 16;
            profile |= (uint32_t)(UartCmdExt.buf[6]) << 24;
            if(SetWorkProfile(profile) == 1)
            {
                ticket[3] = profile & 0xFF;
                ticket[4] = (profile >> 8) & 0xFF;
                ticket[5] = (profile >> 16) & 0xFF;
                ticket[6] = (profile >> 24) & 0xFF;
            }
            else
            {
                ticket[3] = 0x55;
                ticket[4] = 0x55;
                ticket[5] = 0x55;
                ticket[6] = 0x55;
                GPIO_writePin(led3, 1);
            }
            ticket[0] = EXT_CMD_SET_WORKTYPE;
            ticket[1] = 9;
            ticket[2] = 0;
            crc = CRC16(ticket, 7);
            ticket[8] = crc&0xFF;
            ticket[7] = (crc>>8)&0xFF;
            SendData16Ext(ticket, 9);
            //сброс
            memset(&AllData[bFreq], 0x00, sizeof(struct ALLDATA));
            memset(&AllRxDataRAW, 0x00, sizeof(AllRxDataRAW));
            memset(&GPData, 0x00, sizeof(GPData));
            frame = 1;
            delay_ms(100);
        }
        else if(UartCmdExt.buf[0] == EXT_CMD_SET_ZERO)
        {
            GPIO_writePin(led1, 1);
            GPIO_writePin(led2, 1);
            GPIO_writePin(led3, 1);
            GPIO_writePin(led4, 1);
            for(int freq_idx = 0; freq_idx < 2; freq_idx++)
            {
                for (int Tx = 0; Tx < 4; Tx++)
                {//
                    Metro.air_ph[freq_idx][Tx] = (int16_t)((air_summ[freq_idx][PH][Tx]/air_aver)*57297.0);
                    Metro.air_att_dB[freq_idx][Tx] = air_summ[freq_idx][ATT][Tx]/air_aver;
                }
            }

            FLASH_WriteMetrology(&Metro);
            delay_ms(20);

            ticket[0] = EXT_CMD_SET_ZERO;
            ticket[1] = 5;
            ticket[2] = 0;
            crc = CRC16(ticket, 3);
            ticket[4] = crc&0xFF;
            ticket[3] = (crc>>8)&0xFF;
            SendData16Ext(ticket, 5);

            GPIO_writePin(led1, 0);
            GPIO_writePin(led2, 0);
            GPIO_writePin(led3, 0);
            GPIO_writePin(led4, 0);
        }
        else
        {
            gErrorCnt++;
        }
        GPIO_writePin(led1, 0);
    }
}
//---------------------------------------------------------------------------------
struct ID get_sonde_id(uint32_t signature)
{
    struct ID tool;
    tool.type = (signature % 1000000) / 1000;
    tool.type_ = (signature % 1000000) / 100000;
    tool.N_Tx = (signature % 100000) / 10000;
    tool.mod = (signature % 10000) / 1000;
    tool.number = (signature % 1000);
    tool.struct_size = signature / 1000000;// добавлено - размер структуры
    return tool;
}
//---------------------------------------------------------------------------------
uint16_t SetWorkProfile(uint32_t profile)
{
uint32_t serial = Metro.signature % 1000;
uint16_t framesize;
    if(profile == PROFILE_CARTOGRAPH)
    {
        gWorkType = TYPE_CARTOGRAPH;
        framesize = sizeof(struct ALLDATA)*2;
        SetOfPresets.DebugSession = 0;
        FLASH_WriteSetOfPresets(&SetOfPresets);
        Metro.signature = framesize*1000000 + (uint32_t)PROFILE_CARTOGRAPH*1000 + serial;
        FLASH_WriteMetrology(&Metro);

        return 1;
    }
    else if(profile == PROFILE_CARTOGRAPH_RAW)
    {
        gWorkType = TYPE_CARTOGRAPH;
        framesize = sizeof(struct ALLDATA)*2 + sizeof(struct raw_sector_data_all)*2;
        SetOfPresets.DebugSession = 1;
        FLASH_WriteSetOfPresets(&SetOfPresets);
        Metro.signature = framesize*1000000 + (uint32_t)PROFILE_CARTOGRAPH_RAW*1000 + serial;
        FLASH_WriteMetrology(&Metro);
        return 1;
    }
    else if(profile == PROFILE_LWD_4TX)
    {
        gWorkType = TYPE_LWD_4TX;
        framesize = sizeof(struct GP_DATA)*2;
        Metro.signature = framesize*1000000 + (uint32_t)PROFILE_LWD_4TX*1000 + serial;
        FLASH_WriteMetrology(&Metro);
        return 1;
    }
    else
    {
        return 0;
    }
}
