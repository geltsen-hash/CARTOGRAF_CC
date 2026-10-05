/*
 * int_data_ready.c
 *
 *  Created on: 19 ����. 2024 �.
 *      Author: user
 */
#include "driverlib.h"
#include "_globals.h"
#include "device.h"
#include "board.h"
#include "MMC5983_spi.h"
#include "spil3gd20.h"
#include "spiADXL355.h"
#include "math_angles.h"
#include <math.h>
#include <stdio.h>
#include <complex.h>
//#include "sw_prioritized_isr_levels.h"

uint16_t dbgSkipSecorNum = 0; //����� ������
uint16_t dbgSkipCnt = 0; //������� ��� ����������

volatile float Wg;
float Wg_offset = 0;
float Wg_1000 = 0;

float complex Ztarget; //�������
float complex Zmtf; //�������
float complex Zdelta; //���������� �� ��������

extern float aps_m_arr[17];
extern float aps_m_arr_deg[17];
extern uartPrintMode uPm;
extern apsMeasState apsMMode;
extern float aps_arr[17];
volatile boolean bIsMoving = true;
extern bool bAPSPrinted;
//�������� �� ����������, ���� ������� � �������� �������� �� ����
int MA_WINDOW_SIZE;  //���� ����������� �������� ��� ������ � ��� 32
int MLD_WINDOW_SIZE ; //���� C��
int delta; //����������� 1, ���� ������� ���������� �� ���������� ������� ����
float APS_DELTA = PI/360.0; // - 1 ������ (� ����� ��������� ������������ ������ ������)
//float WMIN = 0.0;// rad/c
//float WMAX = 18;// 6 ��.�
uint64_t CORRECTION_TIME = 2000; //����� ��� ��������, �� ��������� �������� ����������� ��������� MTF = GTF (��)
volatile float fMTFIntrvalBegin, fMTFIntervalEnd; //������ � ����� "����� MTF"
// ��� �����, ����� ����������� �������� � ������������� ������ � ����������� ��������
int16_t Wxyz[3];    //����� ��� ������ � �������
long Mxyz[3];    //����� ��� ������ � �������
int32_t Gxyz[3];    //����� ��� ������ � �������
struct Vec G_raw, M_raw, W_raw, G_ma, M_ma, W_ma, G, M, W;
struct Vec GBuff, MBuff, WBuff; //��� ������
struct Vec G_Summa = {0,0,0}, M_Summa = {0,0,0}, W_Summa = {0,0,0}, G_Data[64], M_Data[64], W_Data[64];

//��� ����������
struct Cal G_offset_sens = { 3909340,  3912131,  3944069,1,0,0,0,1,0,0,0,1 },
           M_offset_sens = { -4910770, -3802600, -3421118,1,0,0,0,0.897,0,0,0,1 },
           W_offset_sens = { 0,0,0,0.001221726,0,0,0,0.001221726,0,0,0,0.001221726 },//rad per digit

           G_offset_sens_add = { 0,0,0,1,0,0,0,1,0,0,0,1 },
           M_offset_sens_add = { 0,0,0,1,0,0,0,1,0,0,0,1 },
           W_offset_sens_add = { 0,0,0,1,0,0,0,1,0,0,0,1 };
//���������
struct INC_SET *Settings = (struct INC_SET*)&Metro.inc_set;

//����
float angle_aps, angle_zen, angle_azm, angle_aps_m, MTF,MTF_m_p, start_APS = 0;
float angle_zen_deg, angle_aps_deg, angle_azm_deg, angle_aps_m_deg, MTF_deg;

//��� ��������� ��������� � ���
float G_modul, M_modul, W_modul;
float omega;
float g_modul_buff[64], m_modul_buff[64], w_modul_buff[64], slo_modul_g, slo_modul_m, slo_modul_w;//��� ���
float MTF_buff[128];//����� ��� ��������� ���������
float abc[3] = {0,};//�����. ������. �������� 2�� ������� abc[0]*�*� + abc[1]*� + abc[2]
float history_angle = (20*PI) / 180; //������� �������� ������ ������������
float prediction;// ��� �������������� ��������
int N = 128, K = 128;
float K_predict = 0.5;
extern float S_x[130][5];//�������������� ����������� ����� ��� ��������
float aps_point = 0;
float aps_point_arr[16] = {0.0,};
float aps_point_arr_deg[16] = {0.0,};//dbg
volatile float G_M_angle = -PI; //
uint16_t aps_idx = 0;
int start_aps_idx = 0;
float sector = PI/8;
float idx;
//float rot = 1;
volatile uint32_t  tim = 0, pulse = 0, no_mov_delay_tim = 0;
volatile float angle_path_wg = 0;
uint32_t pulse_width = 10;//������ �������� � ��
volatile bool start_circle;
float MA_delay = 0;
volatile float Delay = 0;
bool interrupt_by_angle_path = false;
volatile uint64_t NextCorrTime = 0;

//-----------------------------------------------------------------------------
__interrupt void INT_TIMER0_1MS_ISR(void) //����� ���� �������� � �������� 1��
{
   /* //==========================================================================
    // Save IER register on stack
    //
    volatile uint16_t tempPIEIER = HWREGH(PIECTRL_BASE + PIE_O_IER1);

    //
    // Set the global and group priority to allow CPU interrupts
    // with higher priority
    //
    IER |= M_INT1;
    IER &= MINT1;
    HWREGH(PIECTRL_BASE + PIE_O_IER1) &= MG1_7;

    //
    // Enable Interrupts
    //
    Interrupt_clearACKGroup(0xFFFFU);
    __asm("  NOP");
    EINT;
//==============================================================================*/

static uint16_t i_filter = 0;

  N = Settings->N;
  K = Settings->K;
  K_predict = Settings->K_predict;
  MA_WINDOW_SIZE = Settings->MA_WINDOW_SIZE;
  MLD_WINDOW_SIZE = Settings->MLD_WINDOW_SIZE;
  //WMIN = Settings.W_MIN;// rad/c
  //WMAX = Settings.W_MAX;// rad/c
  APS_DELTA = Settings->APS_DELTA;
  history_angle = Settings->history_angle;

        //todo: ��������� ������ � �����
#ifdef EMUL

        G_raw.X = -6400;  //0
        G_raw.Y = 250000;  //2
        G_raw.Z = -3300;  //1

        M_raw.X = 4580;
        M_raw.Y = 8251; //-
        M_raw.Z = -908;

        W_raw.X = 3;
        W_raw.Y = 9;
        W_raw.Z = -14;
#else
        L3GD20_ReadXYZ(Wxyz); //������ ��������
        ADXL355_ReadXYZ(Gxyz);
        MMC5983_ReadXYZ(Mxyz);

        //����� � ������ ��� ����� ������
        G_raw.X = Gxyz[1];  //0
        G_raw.Y = Gxyz[2];  //2
        G_raw.Z = Gxyz[0];  //1

        M_raw.X = Mxyz[1];
        M_raw.Y = -Mxyz[2]; //-
        M_raw.Z = Mxyz[0];

        W_raw.X = Wxyz[0];
        W_raw.Y = Wxyz[2];
        W_raw.Z = Wxyz[1];
#endif

        //���������� �������
        //��������  ��������   i ������ ������� ���� �� ����� ���� �������� ���� ����������� ��������
        G_Summa  = vctr_diff(G_Summa, G_Data[i_filter]);
        M_Summa  = vctr_diff(M_Summa, M_Data[i_filter]);
        W_Summa  = vctr_diff(W_Summa, W_Data[i_filter]);
        //��������� i ������ ������� ���� ����������� ��������
        G_Data[i_filter] = G_raw;
        M_Data[i_filter] = M_raw;
        W_Data[i_filter] = W_raw;
        // ������� �����  G M
        G_Summa = vctr_summ(G_Summa, G_Data[i_filter]);
        M_Summa = vctr_summ(M_Summa, M_Data[i_filter]);
        W_Summa = vctr_summ(W_Summa, W_Data[i_filter]);
        // ���������� �����������  ��������i ������ ������� � ����� ���� �������� ���� ����������� ��������
        G_ma = vctr_mltp_n(1.0/MA_WINDOW_SIZE, G_Summa);
        M_ma = vctr_mltp_n(-1.0/MA_WINDOW_SIZE, M_Summa);
        W_ma = vctr_mltp_n(1.0/MA_WINDOW_SIZE, W_Summa);

        G = G_ma;
        M = M_ma;
        W = W_ma;

       //������������� ����������
       //������ �������� offset � �������� �� �������, ��� ������� ��������� - ��� �������� ����,
       //XY=YX,XZ=ZX,ZY=YZ - ����� ���������, ���������� �� ���������a������� ����.
       G = callibrate(G, G_offset_sens);
       M = callibrate(M, M_offset_sens);
       W = callibrate(W, W_offset_sens);

       GBuff = G; //������
       MBuff = M;
       WBuff = W;


       if(gEmulRotateSpeed > 0) //��������
       {
           Wg = -(PI*gEmulRotateSpeed/30.0);
       }
       else //������
       {
           Wg = -W.Z;//rad/c
           Wg -= Wg_offset;
       }
       #define SENSOR_SAMPLE_RATE_HZ 950.0f
       Wg_1000 = Wg / SENSOR_SAMPLE_RATE_HZ; ///rad per 1 sample (T=1.0526ms)

       //��������� ���, ����� ��������� �a� ����������
       G_modul = modul(G); M_modul = modul(M); W_modul = Wg;
       //��������� ��� ������� ���������� ���� ����� �� 32 ��������� ���������.
       //� ���������� ����������� ������������ ��� ���������� ���������� �����
       //K_predict ��� ��������� ����(������) �� ������������.
       // 10 mkc

       float aver_modul_m = 0, summ_modul_m = 0;
       float aver_modul_g = 0, summ_modul_g = 0;
       float aver_modul_w = 0, summ_modul_w = 0;
       slo_modul_g = 0; slo_modul_m = 0; slo_modul_w = 0;

       //������������ ����� �� �������
       for (int i = 0; i < MLD_WINDOW_SIZE-1; i++){
           m_modul_buff[i] = m_modul_buff[i+1];
           g_modul_buff[i] = g_modul_buff[i+1];
           w_modul_buff[i] = w_modul_buff[i+1];
       }
       //����� � ����� ������
       m_modul_buff[MLD_WINDOW_SIZE-1] = M_modul;
       g_modul_buff[MLD_WINDOW_SIZE-1] = G_modul;
       w_modul_buff[MLD_WINDOW_SIZE-1] = W_modul;
       //������� �������
       for (int i = 0; i < MLD_WINDOW_SIZE; i++){
           aver_modul_m +=  m_modul_buff[i];
           aver_modul_g +=  g_modul_buff[i];
           aver_modul_w +=  w_modul_buff[i];
       }
       aver_modul_m /= MLD_WINDOW_SIZE;
       aver_modul_g /= MLD_WINDOW_SIZE;
       aver_modul_w /= MLD_WINDOW_SIZE;
       //������� �������������� ����������
       for (int i = 0; i < MLD_WINDOW_SIZE; i++){
           summ_modul_m += fabs(m_modul_buff[i] - aver_modul_m);
           summ_modul_g += fabs(g_modul_buff[i] - aver_modul_g);
           summ_modul_w += fabs(w_modul_buff[i] - aver_modul_w);
       }
       slo_modul_m = summ_modul_m / MLD_WINDOW_SIZE;
       slo_modul_g = summ_modul_g / MLD_WINDOW_SIZE;
       slo_modul_w = summ_modul_w / MLD_WINDOW_SIZE;

       // ��������� ��� ��� �������� ��������
       if(slo_modul_g < MOVEMENT_CMP_VALUE){
           no_mov_delay_tim++;
       }
       else{
           no_mov_delay_tim = 0;
           bIsMoving = true;
       }

       if(no_mov_delay_tim > 1000){
           no_mov_delay_tim = 0;
           bIsMoving = false;
       }

       if(gEmulRotateSpeed > 0)//��������
           bIsMoving = true;

       //��������� ����
       angle_aps = atan2(G.Y, G.X);
       angle_aps_m = atan2(M.Y, M.X);

       // �� �������
       if(!bIsMoving && (GetNow() > NextCorrTime))
       {
           //GPIO_writePin(led3, 0); //dbg
           NextCorrTime = GetNow() + CORRECTION_TIME;
           G_M_angle = angle_aps_m - angle_aps;//���������� ������� ����� �������� � �������������� ������������ ������
           if(fabs(W.Z) < 0.1)
               //Wg_offset = W.Z;// ���������� �������� ���� ���������
               Wg_offset = aver_modul_w;
       }
       else
       {
           //GPIO_writePin(led3, 1); //dbg
       }

       if(gEmulRotateSpeed > 0) //todo: ROTATE
       {
         MTF = MTF-((gEmulRotateSpeed*PI)/(30.0*SENSOR_SAMPLE_RATE_HZ));
       }
       else
       {
           MTF = angle_aps_m - G_M_angle;
       }
       while(MTF > PI) MTF -= 2*PI;
       while(MTF <= -PI)  MTF += 2*PI;

       //�������� ����
       angle_zen = atan2(sqrt(G.X*G.X + G.Y * G.Y), G.Z);
       //���� ���������� �� ��������� ��������� 6 ��������
       if (fabs(angle_zen) >= 0.1)//rad
           angle_azm = PI - atan2((M.Y*G.X - M.X*G.Y)*G_modul, (M.X*G.X*G.Z + M.Y*G.Y*G.Z - M.Z*G.X*G.X - M.Z*G.Y*G.Y)); //pi-
       //���� �����������, �� ������ ������� � ����������� ���������
       else angle_azm = angle_aps_m;

       angle_aps_deg =   (angle_aps*57.296);
       angle_aps_m_deg = (angle_aps_m*57.296);
       angle_zen_deg =   (angle_zen*57.296);
       angle_azm_deg =   (angle_azm*57.296);
       MTF_deg = (MTF*57.296); //-180...180
       if(MTF_deg < 0)
       {
           MTF_deg = MTF_deg + 360.0; //0...360
       }
       //gSectorIdx = (uint16_t)(MTF_deg/22.5); //� ����� ������� ���������

//�������� ��������
//�������������� ���������� � ����� ������������������ MTF (MTF_C) AX2+BX+C
//� ������ ������� ��� ������� �����. � ����������� ����� �������, �����
//� �������[K] �������������� ���������� ������� ��������� dFi rad
//(������ dFi = history_angle = 22 ���� = PI / 8), ���������� �� �������� ��������.
//��� ����� ������� ������ �������� omega � �������� �� tick(� ��� 1 ��).
//� ������������ ��� ����� ����� ������ ��������� dFi/omega. ���� � ���������
//������ ������ (��� omega < PI / 8*128 ��� < ~ 0.175 ����/��) �� � ��������� =
//������� ������.
//������� �������� �� ��� Z ������� �������������� � �������� 30-180 ��/���,
//��� 180-1080 ����/�, ��� PI - 6*PI rad/c, ��� ~ 0.0031416-0,0188496 rad/tick
//��� ����� ��������� ��������� � ����� ������ � �������� 128 - 22.

    //������������ ����� �� �������
    for (int i = 0; i < N-1; i++) {
        MTF_buff[i] = MTF_buff[i + 1];//??? ������ �������� MTF[N-1] MTF_C[N-2]
    }
    // ���� ������������� ����� +_ PI ������� ���� ����� �� 2PI
    if (MTF_buff[N - 1] -  MTF < - PI) {
        for (int i = 0; i < N - 1; i++) {
           MTF_buff[i] += 2 * PI;
        }
    }
    //����� � ����� ������ ������� MTF_C
    MTF_buff[N-1] = MTF;

    //������� �
    omega = (MTF_buff[N-5] - MTF_buff[N-1]) / 4;
    //K = (int)fabs(history_angle / omega);
    K = (int)fabs(history_angle / Wg_1000);//rad per 1mc
    MA_delay = Wg_1000*MA_WINDOW_SIZE/4.0; //rad ������� �������� �� ���������� ������� ������� �� ��������
    if (K < 1)K = 1;
    if (K > N)K = N-1;
    // ����������� ����� ������� ������� � � ������� ��� ��������
    prediction = predict(abc, &MTF_buff[N-K-1], S_x[K], K);
 /////����� ��������� ��������� //////////////////////////////////////////////////////////////////////////////////////////////

    if(Settings->auto_delta == 1)
        APS_DELTA = Settings->APS_DELTA + fabs(Wg_1000 * Settings->K_delta);
    //��������� ���������� � ����������������� ���� � ��������� � ����� K_predict
    //� �������� ������� ���������� �� �������
    if(fabs(MTF-prediction) > APS_DELTA)
        K_predict = 1;
    else
        K_predict = 0.5;

    MTF_m_p = (MTF*(1 - K_predict) + K_predict * prediction);

       if(apsMMode == APS_IDLE)
       {

       }
       else if(apsMMode == APS_READY) //�������� ����
       {
           bAPSPrinted = false;
           tim = 0;//
           GPIO_writePin(led1, 0);
           apsMMode = APS_WAIT_NEXT_POINT;
           Delay = MA_delay;//������� ��������
           angle_path_wg = -PI/8.0; //������������� � ������ ����� � ������ ������ +1 ������. � ��������� ������ = 0

           gSectorIdx = (uint16_t)(MTF_deg/22.5); //� ����� ������� ���������
           start_APS = gSectorIdx*PI/8.0 - PI/8.0 ;// ��������� ���� + 1 ������ �����

           if(gSectorIdx > 0) //� alldata
               gStartSectorIdx = gSectorIdx-1;
           else
               gStartSectorIdx = 15;

           while(start_APS > PI) //[-pi...+pi]
               start_APS -= 2.0*PI;
           //��������� ����� ��� 16 ������� ��������� ��� ������ ���������� �� ���������
           // � �������� �� �������� � ��������� �� -PI �� PI �������� PI (�� �������� atan2)
           for(int idx = 0; idx < 16; idx++)
           {
               aps_point_arr[idx] = start_APS - idx*PI/8.0;//��� Z-���� � ��������, �������� �� �������->MTF ������� 360->0
               aps_point_arr_deg[idx] = start_APS*57.296 - idx*PI*57.296/8.0;//dbg
               while(aps_point_arr[idx] <= -PI)  aps_point_arr[idx] += 2.0*PI;
               aps_arr[idx] = aps_point_arr[idx]; //dbg
           }

           aps_point = aps_point_arr[0];//aps_point - ����������� � uart ����� ������ �o��� �������
           GPIO_writePin(led1, 1);
           pulse = 0;//���������� �������
           aps_idx = 0;
       }
       else if(apsMMode == APS_WAIT_NEXT_POINT)
       {
           pulse++;//������������ �������� �� ���������� � �� - ��������������- � � ���� ���������� 1 ��
           if(pulse > pulse_width) //pulse_width - ������������ �������� �� ����������
           {
               GPIO_writePin(led1, 0); //���� ������� ������ ������ ���������
           }

            angle_path_wg +=  -Wg_1000;  //������� ���� �� ������ ������� � �������� (Wg_1000 - ������� �������� ���/�������)

           if(angle_path_wg > (PI/8.0 + 4.0*APS_DELTA) && Settings->INTERRUPT_BY_ANGLE_PATH == 1)
           {
               interrupt_by_angle_path = true;//��������� �������
               angle_path_wg = 0;//�������� ������� ���� � ������ ����������
           }

           aps_point = aps_point_arr[aps_idx];//aps_point - ����������� � uart ����� ������ �o��� �������
           //----------------------------------��������� � �����----------------------------------
           Zmtf = CMPLXF(cos(MTF_m_p), sin(MTF_m_p));
           Ztarget = CMPLXF(cos(aps_point_arr[aps_idx]), sin(aps_point_arr[aps_idx]));
           Zdelta = Ztarget - Zmtf;
           if(cabs(Zdelta) < sin(APS_DELTA) || interrupt_by_angle_path) //����� ������ ���� ~ ����
           {
               GPIO_writePin(led1, 1);
               gSectorIdx = (uint16_t)(MTF_deg/22.5); //� ����� ������� ��������� //
               //==============================================================================================
               /*if(gSectorIdx != dbgSkipSecorNum)//dbg ��� �������� ��������
               {
                   wc.wasNewSectorInt = true;
               }
               else
               {
                   if(dbgSkipCnt < 3)
                   {
                       dbgSkipCnt++;
                   }
                   else
                   {
                       dbgSkipCnt = 0;
                       if(dbgSkipSecorNum < 15)
                       {
                           dbgSkipSecorNum++;
                       }
                       else
                       {
                           dbgSkipSecorNum = 0;
                       }
                   }
               }*/
               //==============================================================================================
               wc.wasNewSectorInt = true; //todo �������� ����� ������� ��������!!!
               aps_m_arr[aps_idx] = MTF_m_p;
               aps_m_arr_deg[aps_idx] = MTF_m_p*57.296;//dbg
               pulse = 0;//���������� ������� ������� ��� �������� ����������
               interrupt_by_angle_path = false;
               angle_path_wg = 0;//�������� ������� ���� � ������ ����������
               aps_idx++;// ������������� ������� ������� ������� ���������

               if(aps_idx == 16)//���� ������� ������ ������
               {
                   aps_m_arr[16] = APS_DELTA*57.296;//����� ������ �o��� �������
                   aps_idx = 0;
                   angle_path_wg = 0;
                   apsMMode = APS_COMPLETE;
              }
           }
           else //��������� ��� �� ���������� �����
           {
            // if(MTF_m_p < aps_point_arr[aps_idx] - APS_DELTA )
           }
       }
       else if(apsMMode == APS_COMPLETE)
       {
           GPIO_writePin(led2, 0);
           pulse++;
           if(pulse > pulse_width) //pulse_width - ������������ �������� �� ����������
           {
               GPIO_writePin(led1, 0); //���� ������� ������ ������ ���������
           }
       }
////////////////////////////////////////////////////////////////////////////////////////////////////

    if(i_filter < (MA_WINDOW_SIZE-1))
      i_filter++;
    else i_filter = 0;

    wc.wasADCInt = true;
   /*//===============================================
    DINT;
    HWREGH(PIECTRL_BASE + PIE_O_IER1) = tempPIEIER;
   //===============================================*/
   Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1); //
}
