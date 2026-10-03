//#############################################################################
//
// FILE:   empty_sysconfig_main.c
//
// TITLE:  Empty Pinmux Example
//
// Empty SysCfg & Driverlib Example
//
// This example is an empty project setup for SysConfig and Driverlib 
// development.
//
//#############################################################################
//
// $Release Date: $
// $Copyright:
// Copyright (C) 2014-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//

#include "_globals.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "MMC5983_spi.h"
#include "spil3gd20.h"
#include "spiADXL355.h"
#include "uart.h"
#include <stdio.h>
#include <math.h>
#include "math_angles.h"
#include "CRC16ModBus.h"
//
// Main
//

#pragma SET_DATA_SECTION("bootflag")
const uint16_t bootflag[8] = {0,};
#pragma SET_DATA_SECTION()

volatile uint64_t Now = 0;
uint32_t timeout; //��� i2c
uint16_t gWorkType;

uint16_t status;
char str[128];
float aps_arr[17];
float aps_m_arr[17]; //���������� ���������� �������� aps_m_corr � 16 ������ �����
float aps_m_arr_deg[17] = {0.0,}; //dbg
uartPrintMode uPm = STOP;//STOP
apsMeasState apsMMode = APS_IDLE;
extern uint16_t aps_idx, flg_0;

TSTATE controller_state;
extern struct TOneWorkCycle wc;
//#pragma DATA_SECTION (Metro, "ram2balign")
struct METROLOGY_CARTOGRAPH Metro; //ram2balign
struct METROLOGY_GP MetroGp;

#pragma DATA_ALIGN(Metro,2);

struct raw_sector_data_all AllRxDataRAW;
struct ALLDATA AllData[2];
struct GP_DATA GPData;

extern float slo_modul_g, slo_modul_m, slo_modul_w, rot;
extern int N, K;
extern float aps_point, G_M_angle, MA_delay;

float S_x[130][5];

boolean bAPS2MTF_C = false; //������ � CAN MTF_C ������ AZM
boolean continue_aps = false;

uint16_t gSectorIdx; //����� �������� ������� � ����� Pi/8 0=0...22.5; 1=22.5...45 ���.
uint16_t gEmulRotateSpeed = 0; // ��/���; >0 ��������, =0 ������� �����

bool bExtCmdGetData;
struct SET_OF_PRESETS SetOfPresets;
uint16_t bPeriodicStart = 0;
extern bool bAPSPrinted;
//-----------------------------------------------------------------------------
__interrupt void INT_TIMER1_1MS_ISR(void) //1�� ���������� ������� �������
{
    Now++;
   //Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
//-----------------------------------------------------------------------------
uint64_t GetNow(void)
{
    uint64_t t;
    DINT;
    t = Now;
    EINT;
    return t;
}
//-----------------------------------------------------------------------------
/*__interrupt void INT_TIMER_US_ISR(void) //10�� ������2
{
    CPUTimer_stopTimer(TIMER_US_BASE);
}*/
//-----------------------------------------------------------------------------
uint16_t GetWorkType(uint32_t signature)
{
    uint16_t worktype = 0xFFFF;
    int tool = 0;
    //uint32_t buff = signature & 0x000FFFFF;
    tool = (signature / 1000) % 1000;

    if(tool == PROFILE_LWD_4TX)
        worktype = 1;
    else if(tool == PROFILE_CARTOGRAPH_RAW || tool == PROFILE_CARTOGRAPH)
        worktype = 0;

    return worktype;
}
//-----------------------------------------------------------------------------
uint16_t CheckMetroData(void) //1=ok 0=������
{
    if(Settings->MA_WINDOW_SIZE > 64 || Settings->MA_WINDOW_SIZE < 4)
        return 0;
    if(Settings->MLD_WINDOW_SIZE > 64 || Settings->MLD_WINDOW_SIZE < 4)
        return 0;
    if(Settings->N > 256 || Settings->N < 4)
        return 0;
    if(Settings->K > 256 || Settings->K < 4)
        return 0;
    if(Settings->auto_delta > 1)
        return 0;
    if(Settings->INTERRUPT_BY_ANGLE_PATH > 1)
        return 0;
    if(Settings->W_MIN < 0 || Settings->W_MIN > 50000)
        return 0;
    if(Settings->W_MAX < 0 || Settings->W_MAX > 50000)
        return 0;
    if(Settings->K_predict < 0 || Settings->K_predict > 100)
        return 0;
    if(Settings->APS_DELTA < 0 || Settings->APS_DELTA > PI)
        return 0;
    if(Settings->history_angle < 0 || Settings->history_angle > PI)
        return 0;
    if(Settings->K_delta < 0 || Settings->K_delta > 100)
        return 0;

    if(((Metro.select_key >> 24) & 0x0003) == 0) //����� ������
        return 0;
    if(Metro.F[0]>1000)
        return 0;
    if(Metro.F[1]<1000)
        return 0;

    uint32_t worktype = GetWorkType(Metro.signature);
    if(worktype != TYPE_CARTOGRAPH && worktype != TYPE_LWD_4TX ) //�������������� ���? TYPE_CARTOGRAPH=351 TYPE_LWD_4TX=349
        return 0;

    for(uint16_t i=0; i<4; i++)
    {
       if(Metro.L1[i] < 10 || Metro.L1[i] > 5000)
           return 0;
       if(Metro.L2[i] < 10 || Metro.L2[i] > 5000)
           return 0;
      // if(Metro.L_geo[i] < 10 || L_geo[i] > 5000)
      //     return 0;
    }

   // if(Metro.D_sonde_mm < 80 || Metro.D_sonde_mm > 300)
    //    return 0;

    if(Metro.F[0] < 50 || Metro.F[0] > 3000)
        return 0;

    if(Metro.F[1] < 50 || Metro.F[1] > 3000)
        return 0;

    return 1;
}
//-----------------------------------------------------------------------------
void ReadCal()
{
   FLASH_ReadCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
   //FLASH_ReadSet

   if(isnan(G_offset_sens.offset.X)) G_offset_sens.offset.X = 3909340;
   if(isnan(G_offset_sens.offset.Y)) G_offset_sens.offset.Y = 3912131;
   if(isnan(G_offset_sens.offset.Z)) G_offset_sens.offset.Z = 3944069;
   if(isnan(G_offset_sens.sens.XX)) G_offset_sens.sens.XX = 1;
   if(isnan(G_offset_sens.sens.YX)) G_offset_sens.sens.YX = 0;
   if(isnan(G_offset_sens.sens.ZX)) G_offset_sens.sens.ZX = 0;
   if(isnan(G_offset_sens.sens.XY)) G_offset_sens.sens.XY = 0;
   if(isnan(G_offset_sens.sens.YY)) G_offset_sens.sens.YY = 1;
   if(isnan(G_offset_sens.sens.ZY)) G_offset_sens.sens.ZY = 0;
   if(isnan(G_offset_sens.sens.XZ)) G_offset_sens.sens.XZ = 0;
   if(isnan(G_offset_sens.sens.YZ)) G_offset_sens.sens.YZ = 0;
   if(isnan(G_offset_sens.sens.ZZ)) G_offset_sens.sens.ZZ = 1;

   if(isnan(M_offset_sens.offset.X)) M_offset_sens.offset.X = -4910770;
   if(isnan(M_offset_sens.offset.Y)) M_offset_sens.offset.Y = -3802600;
   if(isnan(M_offset_sens.offset.Z)) M_offset_sens.offset.Z = -3421118;
   if(isnan(M_offset_sens.sens.XX)) M_offset_sens.sens.XX = 1;
   if(isnan(M_offset_sens.sens.YX)) M_offset_sens.sens.YX = 0;
   if(isnan(M_offset_sens.sens.ZX)) M_offset_sens.sens.ZX = 0;
   if(isnan(M_offset_sens.sens.XY)) M_offset_sens.sens.XY = 0;
   if(isnan(M_offset_sens.sens.YY)) M_offset_sens.sens.YY = 0.897;
   if(isnan(M_offset_sens.sens.ZY)) M_offset_sens.sens.ZY = 0;
   if(isnan(M_offset_sens.sens.XZ)) M_offset_sens.sens.XZ = 0;
   if(isnan(M_offset_sens.sens.YZ)) M_offset_sens.sens.YZ = 0;
   if(isnan(M_offset_sens.sens.ZZ)) M_offset_sens.sens.ZZ = 1;

   if(isnan(W_offset_sens.offset.X)) W_offset_sens.offset.X = 0;
   if(isnan(W_offset_sens.offset.Y)) W_offset_sens.offset.Y = 0;
   if(isnan(W_offset_sens.offset.Z)) W_offset_sens.offset.Z = 0;
   if(isnan(W_offset_sens.sens.XX)) W_offset_sens.sens.XX = 0.001221726;
   if(isnan(W_offset_sens.sens.YX)) W_offset_sens.sens.YX = 0;
   if(isnan(W_offset_sens.sens.ZX)) W_offset_sens.sens.ZX = 0;
   if(isnan(W_offset_sens.sens.XY)) W_offset_sens.sens.XY = 0;
   if(isnan(W_offset_sens.sens.YY)) W_offset_sens.sens.YY = 0.001221726;
   if(isnan(W_offset_sens.sens.ZY)) W_offset_sens.sens.ZY = 0;
   if(isnan(W_offset_sens.sens.XZ)) W_offset_sens.sens.XZ = 0;
   if(isnan(W_offset_sens.sens.YZ)) W_offset_sens.sens.YZ = 0;
   if(isnan(W_offset_sens.sens.ZZ)) W_offset_sens.sens.ZZ = 0.001221726;

   /*if(Settings.MA_WINDOW_SIZE == 0xFFFFFFFF) Settings.MA_WINDOW_SIZE = 32;
   if(Settings.MLD_WINDOW_SIZE == 0xFFFFFFFF) Settings.MLD_WINDOW_SIZE = 32;
   if(Settings.N == 0xFFFFFFFF) Settings.N = 128;
   if(Settings.K == 0xFFFFFFFF) Settings.K = 128;
   if(Settings.auto_delta == 0xFFFFFFFF) Settings.auto_delta = 1;
   if(Settings. INTERRUPT_BY_ANGLE_PATH == 0xFFFFFFFF) Settings.INTERRUPT_BY_ANGLE_PATH = 0;
   if(isnan(Settings.W_MIN)) Settings.W_MIN = 3.14;
   if(isnan(Settings.W_MAX)) Settings.W_MAX = 18.84;
   if(isnan(Settings.K_predict)) Settings.K_predict = 0.5;
   if(isnan(Settings.APS_DELTA)) Settings.APS_DELTA = PI/360.0;
   if(isnan(Settings.history_angle)) Settings.history_angle = (20.0*PI)/360.0;
   if(isnan(Settings.K_delta)) Settings.K_delta = 2.5;*/
}
//-----------------------------------------------------------------------------
void DefaultMetro(void)
{
    Settings->MA_WINDOW_SIZE = 32;
    Settings->MLD_WINDOW_SIZE = 32;
    Settings->N = 128;
    Settings->K = 128;
    Settings->auto_delta = 1;
    Settings->INTERRUPT_BY_ANGLE_PATH = 0;
    Settings->W_MIN = 0;
    Settings->W_MAX = 20.0;
    Settings->K_predict = 0.5;
    Settings->APS_DELTA = PI/360.0;
    Settings->history_angle = (20.0*PI)/360.0;
    Settings->K_delta = 2.5;

    Metro.signature = 512351001;
    gWorkType = GetWorkType(Metro.signature);
    //Metro.L  10...5000
    //lgeo 10...5000
    //dsonde 80...300;
    //f 50...3000

    GPIO_writePin(led3, 1);
}
//-----------------------------------------------------------------------------
void main(void)
{
    uint16_t rc;

    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
	Board_init();
	GPIO_setPadConfig(43, GPIO_PIN_TYPE_PULLUP); //gpio43
	GPIO_setPadConfig(62, GPIO_PIN_TYPE_PULLUP); //gpio62

	GPIO_writePin(led1, 1); //Data Rq, Points
	GPIO_writePin(led2, 1); //Cycl
	GPIO_writePin(led3, 1); //ERROR Wlim, ERROR Metro
	GPIO_writePin(led4, 1); //ERROR CAN
	delay_ms(200);
    GPIO_writePin(led1, 0);
    GPIO_writePin(led2, 0);
    GPIO_writePin(led3, 0);
    GPIO_writePin(led4, 0);

	volatile uint32_t f = SysCtl_getClock(25000000);
	f = SysCtl_getAuxClock(25000000);

	//while(1){ test(); };

	uint64_t TimeCmd = 0;
	uint64_t TimePrint = 0;
	controller_state = STATE_IDLE;
	wc.currSubState = DMCSS_DONE; //1.12.2025
	ReadCal();
	FLASH_ReadMetrology(&Metro);

	if(CheckMetroData() == 0) //
	{
	    DefaultMetro();
        GPIO_writePin(led3, 1);
	}
	else
	{
	    controller_state = STATE_NORMAL;
	    AllData[0].signature = Metro.signature;
	    AllData[1].signature = Metro.signature;
	    GPData.signature = Metro.signature;
	}
	gWorkType = GetWorkType(Metro.signature);

	FLASH_ReadSetOfPresets(&SetOfPresets);
	//uint32_t profile = (Metro.signature & 0x000FFFFF)/1000;
	uint32_t profile = ((Metro.signature)/1000)%1000;
	gEmulRotateSpeed = (SetOfPresets.DebugSession >> 8)&0xFF;
	if((SetOfPresets.DebugSession & 0x00FF) > 1)
	{
	    if(profile == PROFILE_CARTOGRAPH_RAW)
	        SetOfPresets.DebugSession = 1;
	    else
	        SetOfPresets.DebugSession = 0;

	   GPIO_writePin(led3, 1);
	}
	if(gEmulRotateSpeed > 200)
    {
       gEmulRotateSpeed = 0;
       GPIO_writePin(led3, 1);
    }

	if(SetOfPresets.MinRotSpeed > 60000) //todo �������� �������� ��� ��������
	{
	   SetOfPresets.MinRotSpeed = 20;
	   GPIO_writePin(led3, 1);
	}

	if(SetOfPresets.TBTIndex > 2)
	{
	   SetOfPresets.TBTIndex = 0;
	   GPIO_writePin(led3, 1);
	}


	//������� ���� ��� ������ ��� ���� �!!!!!!!!!!!
	    for(int i = 0; i < 130; i++){
	        S_X(S_x[i], i);
	    }

	MMC5983_Init();
	L3GD20_Init();
	ADXL355_Reset();
	delay_ms(1);
	ADXL355_Init();
	delay_ms(1);

	CAN_enableController(CAN_A_BASE);
	CAN_disableRetry(CAN_A_BASE);
	//0x100 - �����������������
	//0x101 - ������ ������ � ����������
    //0x200...0x20f - ������
	//0x210 - CRC � ������
    for(int i=0; i<24; i++)
    {
        CAN_setupMessageObject(CAN_A_BASE, 3+i, 0x200+i, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, 0, 8); //3-26
    }
    CAN_setupMessageObject(CAN_A_BASE, 32, 0x21E, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, 8); //32
    //��� �������� ������ ������ �������
    CAN_setupMessageObject(CAN_A_BASE, 31, 0x21F, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE, 8); //31
	EINT;
	ERTM;

	//controller_state = STATE_NORMAL;

	while(1)
	{
        ParseDebugCmd(); //���������� ������� �� Bluetooth
        ParseExtCmd();

        rc = SCI_getRxStatus(SCIC_BASE); //��������� ������ �����
        if(rc & (SCI_RXSTATUS_PARITY|SCI_RXSTATUS_OVERRUN|SCI_RXSTATUS_FRAMING|SCI_RXSTATUS_BREAK)) //0x3c
        {
            SCI_performSoftwareReset(SCIC_BASE);
            SCI_init();
            delay_ms(50);
        }

        rc = SCI_getRxStatus(SCIA_BASE); //��������� ������ �����
        if(rc & (SCI_RXSTATUS_PARITY|SCI_RXSTATUS_OVERRUN|SCI_RXSTATUS_FRAMING|SCI_RXSTATUS_BREAK)) //0x3c
        {
            SCI_performSoftwareReset(SCIA_BASE);
            SCI_init();
            delay_ms(50);
        }

        if(controller_state == STATE_NORMAL)
            WorkCyclogram();

        if(GetNow() > TimePrint)
        {
            PrintDebugInfo();
            TimePrint = GetNow() + 50;
        }
        if(apsMMode == APS_COMPLETE && uPm == PRINT_APS)
        {
           PrintDebugInfo();
        }

        if(GetNow() > TimeCmd && bPeriodicStart)
        {
           TimeCmd = GetNow() + 2000;
           TOneWorkCycle_Create();
           bExtCmdGetData = true;
           GPIO_writePin(led2, 0);
        }

	}//end main loop
}

//
// End of File
//
