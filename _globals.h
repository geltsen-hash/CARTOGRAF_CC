/*
 * inc_defs.h
 *
 *  Created on: 16 ��� 2018 �.
 *      Author: 1
 */

//#ifndef INC_DEFS_H_
//#define INC_DEFS_H_

#ifndef _GLOBALS_H_
#define _GLOBALS_H_

//����� �������� (��� �������� � ������ �� CAN)
//#define EMUL

#include "driverlib.h"
#include "board.h"
#include "F021_F2837xS_C28x.h"
#include <stdio.h>
#include <math.h>
#include <complex.h>


#define PI 3.14159265359
extern volatile uint64_t Now; //����� � ��
uint64_t GetNow(void);

extern uint16_t gStartSectorIdx;
extern uint16_t gWorkType;

#define AXEL_AVG_CNT 8
#define AXEL_FILTER_CNT 8
#define MOVEMENT_CMP_VALUE 1000 //300

#define PWM_PERIOD          3599
#define PWM_SYNC_PERIOD     90 //550000 ��
#define PWM_SYNC_CMP        45 // 50% ����������

#define delay_s(x) SysCtl_delay(((((long double)(x)) / (1.0L /  \
                              (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)
#define delay_ms(x) SysCtl_delay(((((long double)(x)) / (1000.0L /  \
                              (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)
#define delay_us(x) SysCtl_delay(((((long double)(x)) / (1000000.0L /  \
                              (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define MIN_W_ERR_BIT 0
#define TIMEOUT_ERR_BIT 1
#define VIBRATION_BIT 2
#define NOT_REACH_END 3
#define K_PREDIKT_1 4

extern struct Vec G_raw, M_raw, W_raw, G_ma, M_ma, W_ma, G, M, W;
extern float slo_modul_g;
extern float omega;
extern float angle_zen_deg, angle_aps_deg, angle_azm_deg, MTF, MTF_deg, Wm;
extern volatile float Wg;
extern float prediction;
extern int N, K;

extern uint16_t gSectorIdx;
extern uint16_t gEmulRotateSpeed;
extern float gWlim;
extern uint64_t CORRECTION_TIME;

extern volatile uint16_t gExtPktsRq;
extern volatile uint16_t gExtPktsAns;
extern volatile uint16_t gErrorCnt;
extern uint32_t frame;

struct Vec
{
    float X;
    float Y;
    float Z;
};

struct MATRIX_3_3{
    float XX; float YX; float ZX;
    float XY; float YY; float ZY;
    float XZ; float YZ; float ZZ;
};

struct Cal {
    struct Vec offset;
    struct MATRIX_3_3 sens;
};
extern struct Cal G_offset_sens, M_offset_sens, W_offset_sens, G_offset_sens_add, M_offset_sens_add, W_offset_sens_add;

enum
{
    TYPE_CARTOGRAPH = 0,
    TYPE_LWD_4TX = 1
};

//------------------------------------------------------------------------------------
//FLASH
extern bool bFlashError; // =1 ���� ��������� ������ ������/������
void Example_Error(Fapi_StatusType status); //��� �������
void FLASH_WriteCal(struct Cal *pG, struct Cal *pM, struct Cal *pW); //������� �������� W � ����� 3 ��������� ������ �� flash
void FLASH_ReadCal(struct Cal *pG, struct Cal *pM, struct Cal *pW); //������
//void FLASH_WriteSet(struct Set *pSet); //���� ��� Set, ������ � ������ X
struct INC_SET;
void FLASH_ReadSet(struct INC_SET *pSet);
struct METROLOGY_CARTOGRAPH;
void FLASH_WriteMetrology(struct METROLOGY_CARTOGRAPH *pMetro); //� ������ X
void FLASH_ReadMetrology(struct METROLOGY_CARTOGRAPH *pMetro);
struct SET_OF_PRESETS;
void FLASH_WriteSetOfPresets(struct SET_OF_PRESETS *pSetOfPresets); //� ������ Y
void FLASH_ReadSetOfPresets(struct SET_OF_PRESETS *pSetOfPresets);

enum
{
  PAGE_W = 1,
  PAGE_X = 2,
  PAGE_Y = 3,
  PAGE_AB = 4
};
void FLASH_Erase(uint16_t page); //������� �������� W/X/Y
//------------------------------------------------------------------------------------
//��������� � �������
typedef enum
{
    PRINT_GMW,
    PRINT_APS,
    PRINT_RAW,
    PRINT_ANG,
    STOP
}
uartPrintMode;

typedef enum
{
    APS_IDLE, //�� ����������
    APS_READY, //��������� ������� �� ���������, ���� ��������� ���� �� ���� �������� � ���� ���
    APS_WAIT_NEXT_POINT, //�������� �����
    APS_COMPLETE //������ ������ ��������
}
apsMeasState;
extern apsMeasState apsMMode;

typedef enum //���������
{
    STATE_IDLE,
    STATE_NORMAL, //����������� ��������
    STATE_NOT_CALIBRATED //����� bluetooth
}TSTATE;

extern TSTATE controller_state;

void PrintDebugInfo(void);
void ParseDebugCmd(void);

extern uint16_t gTxDataCAN[8];
extern uint16_t gRxDataCAN[8];
extern volatile uint16_t gCANError;
extern volatile uint16_t gCANRxPktCnt;
extern uint16_t gCANBuffer[256];


enum
{
   CAN_ERROR_NOERROR,
   CAN_ERROR_TX_FAILED,
   CAN_ERROR_RX_DATA_TIMEOUT,
   CAN_ERROR_CRC
};
//--------------------------------------------------------------------------------------------
//���������� ������ ������ �� ����� ������������ ���������:
typedef enum E_DMC_SUBSTATES { // DMC = Direct Measurements Cycle
  DMCSS_WaitForRequest = 0x0000, // ������� ������� ������ �� �����
  DMCSS_SilentTxState = 0x0001, // ��������� ����� �������� ������������, ����� �� �������
  DMCSS_IntermedialState = 0x0002, // ��������� ��������� �����, ����� �� �������
  DMCSS_PreparingData = 0x0003, // ���� ����� �� ���������� ������ �� ����� �����
  DMCSS_GettingData = 0x0004, // ��������� ������ �� ����������� Rx (����� ����� ��������� - ���� �� ��������� ��� Rx)
  DMCSS_Perform2ndCycle = 0x0005, //��� ���������� 2 ������ � �������������� ������ (����� �������� �������������)
  DMCSS_DONE = 0x0006 //���������
} EDMCSubStates;


#define SECTORS_COUNT 16
#define MAX_CYCLE_TIME_MS 1600
#define RX_PREPARETIME_US 6000//128
//extern const uint16_t prepareDataTimeMcs; // ����� �������� ���������� ������ �� ������������ ���������� � ���,
                                          // ������ 64���, �� ����� ��������� ����� ��������
// ����������� �������� ������������ �� �������� � ������� ������ �� ������������
// ��� ������������ �� ���������. ������� - �������� � ����� ����������
/*extern const uint16_t constTx_undirect[6];
extern const uint16_t constTxByTurn[5][6];
extern const uint16_t constTxWorkTime[7];*/
typedef enum TFREQ_MODE
{
  FMODE_400_ONLY = 1,
  FMODE_2000_ONLY = 2,
  FMODE_BOTH = 3,
  FMODE_ERROR = 0
}FMode;

struct TOneWorkCycle {
  uint32_t TxByTurn[3][5][6]; // �� ���������� � ����� {1,3,4,7,7,7},{2,5,7,7,7,7},...{7,7,7,7,7,7},...}
                           // 7 - ������� ���������� �����������; max - 6 Tx
  uint32_t Tx_undirect[6];   // �� ���������� ���������� �����������, ��� �������������� ���������
  uint32_t TxWorkTime[7];// �� ����������, � ���-�� �������� ������ ����������� ����� ������ Rx (��������� 4, 8, 12, ...)
                         // ���������� ����� �������� ����� �� 1�� ������;
                         // ������ ��� ������� �����������, ������� � �������� (���������) � ����� {4,4,4,8,8,4,0};
                         // ���� � �������� ��������, ��� ������ ����������� ��� � �������.

  bool isDirectCycle; // ������ ���� - ������������ (true) ��� �������������� (false), ��������������� � ������������
                      // � ������ ����� ����� ������
  uint16_t currFreq;  // 400 ��� 2000 ( <1000 - �����. 400k ; >=1000 ����� 2000k ); ����. ����� �� ����������
  uint16_t inclinoSector; // ������ �� ������������
  uint16_t turnsCount; // ����� �������� ��� ������ �������� - �� ������� TxByTurn
  uint16_t turnCounter; // ������� �������� � ������������ � TxByTurn
  uint16_t indexTxByTurn; // ������ ����������� � ������� TxByTurn ��� ������ ��������
  uint16_t sectorCounter; // ������� ��������
  volatile uint16_t timeUnitCounter; // ������� ����� ������������ ��� ��������� ������� ��������
  uint16_t allTimeUnits; // ����� ����� ����� ������������ ��� ������� ����������� �� TxWorkTime + 1
  volatile uint32_t cycleStartTime_ms; // ����� ������ ����� � ��
  uint16_t currTx; // ������� ����� �����������
  uint16_t prepareDataTimeMcsCounter; // ������� ����������� �� ��������� ������ ������ ����������
  bool bTxActive;
  bool bFinishFlag; //��� �������� ������� �����


  volatile bool waitNextSector; // ��������� ����������, ������� ��������������� � ������� ���� ��� ��������
                       // ��� ��������� ������ � ������ �������� �������, �� ���� ���������� ��� �� ������

  volatile bool wasNewSectorInt;  // ���� ���������� �� ������������ �� ������ ���������� �������
  volatile bool wasADCInt; // ���� ���������� �� ������������ �� ��� - ��� �������� ��� ������� � �����������

  EDMCSubStates currSubState; // ������� ��������� �����������
  FMode freqMode;
  uint16_t delay2ndCycle;
};
extern struct TOneWorkCycle wc; // ���������� ����������
enum
{
    PROFILE_LWD_4TX = 349,
    PROFILE_CARTOGRAPH = 352,
    PROFILE_CARTOGRAPH_RAW = 351
};
//-------------------------------------------------------------------------------------------------------------
struct Inclinometer {
    float Azm;
    float Zen;
    float Aps;
};

struct direct_RX {//128  6.10.2025
    float complex Tx_0;//8
    float complex Geo[4];//32
    float amp_Vzz[4];//16
    float amp_Vzx[4];//16
    float ph_Vzx_Vzz[4];//16
    float dv[4];//16
    float border_angle[4];//16
    float temperature;//4
    uint32_t condition;//4
};

//undirect_RX  80+
struct undirect_RX {
    float complex Tx_0[2];
    float complex Rzz1[4];
    float complex Rzz2[4];
    float temperature;
    uint32_t condition;
};

enum T_ {
    T1, T2, T3, T4, T5
};

enum FREQ
{
    _400_kGz,
    _2000_kGz
};

//��������� ���  ������ � ���������� ������/////////////////////////////////////
struct PACKED
{
    uint16_t data;//����������� ������
    //���������� �������� ������� ���, ������� ��������� ������� ��� �������� ������
    //��������, ���� data 0b00000011 10101010 10 ��������  ���, �� bits = 10
    uint16_t bits;
};

struct COMPRESSED_1freq
{
    uint32_t frame;// ����� ����� �� ������ �����
    uint32_t dds_freq;//������� � ������ 1 �������� ��� 0b00000000 - 400 ��� 0b00000001 - 2000 ���
    struct PACKED GA[4];//��������� ���������� � ���������� � ������ ����
    struct PACKED GP[4];//���� ���������� � ���������� � ������ ����
    struct PACKED Ro[4];//��� � ������ ����
};

struct TO_PACK
{
    struct PACKED G[24];//��������� ���������� � ���������� � ������ ����
};

//// ��������� �������� ������������� ����� � ������������ �����������.
struct GP_DATA //320 byte
{
    uint32_t signature;
    uint32_t condition;
    uint32_t frame;
    float temperature;
    float rho_ph_smt[2][5];// ������� ���, ������������ �� ����������� [400, 2000][T1-T5]
    float phase_smt[2][5];// ���������������� ���� [400, 2000][T1-T5]
    float AM_RX_1[2][5];// ��������� �� ������ ��������� [400, 2000][T1-T5]
    float ZERO_AM_RX_1[2]; // ��������� �� ������ ��������� [400, 2000] ��� �������� ������������
    float AM_RX_2[2][5];// ��������� �� ������ ���������[400, 2000][T1 - T5]
    float ZERO_AM_RX_2[2];// ��������� �� ������ ��������� [400, 2000] ��� �������� ������������
    float DELTA_PH[2][5];// ����� ������� ��� [400, 2000][T1 - T5]
    float ZERO_dPH[2];// ������� ��� �������� ������������;
    //��������� ��� ����������� ��������� ���
    float rho_att_smt[2][5];//����������� ���, ������������ �� ����������� [400, 2000][T1-T5]
    float att_smt[2][5];// ���������������� ����������� ��������� [400, 2000][T1-T5]
};
extern struct GP_DATA GPData;

// ��������� ����������
struct ALLDATA  // ����� 512 ����.
{
    // ��������������� ����������//////////////////////////////////
    uint32_t signature;//������������� ���� ������� (��� ���������� 351 ��� 352,3,4,5-9)
    uint32_t frame;//����� �����                                              //
    uint32_t dds_freq;// ��������� ���, �������� 401 ��� 2000
    float ATT_dB_geo_signal_smt[4];//��������� � ���������� ������������ ���������������� ���������
    float PH_deg_geo_signal_smt[4];//���� � �������� ������������ ���������������� ���������
    float rho_ph_smt[4];//������� ��� ,������������ �� ����������� �������
    float phase_smt[4];//������������� ����, ������������ �� ����������� �������
    uint16_t out_arr[16];//������ ������ � �������� ����������� ������ ��� �������� � �����������
    uint32_t all_bit_cntr;//���������� �������� ��� � ���� �������
    uint32_t depth;//�������, �� ������� ���� ������� ���������(����������� ��� �������� � �������*)
    //���������� ��� �������////////////////////////////////////////
    struct direct_RX R_zx[2];// ������ �� ���� ������������ ���������� [0]�������(�����) � [1]������(������)
    struct undirect_RX R_zz;// ������ �� ������������ ���������
    struct Inclinometer INC;// ���� ABS, ZEN, AZM
    uint16_t  start_sector; // ��������� ������ ������������ ��������� 0-15
    uint16_t DataValid;//���������� ������������ ��������� (��� �������� � �����������)
    float Wg; //������� ��������  ��.���
    //��������� ��� ����������� ��������� ���
    float rho_att_smt[4];//��� ,������������ �� ����������� �������
    float att_smt[4];//������������� ���� ,�� ����������� �������
};
extern struct ALLDATA AllData[2];

struct _16_sector_data //RAW � ����. ���������� (3,4) �� ������ TX(1,2,4,5 ��� 1,2,3,5)
{
    float Re_sector[16];
    float Im_sector[16];
};

//raw_sector_data_all  1024 � uart
struct raw_sector_data_all
{
    struct _16_sector_data Rx_L[4];
    struct _16_sector_data Rx_R[4];
};
extern struct raw_sector_data_all AllRxDataRAW;
//-------------------------------------------------------------------------------------------------------------
struct INC_SET
{
    uint32_t MA_WINDOW_SIZE; //32
    uint32_t MLD_WINDOW_SIZE; //32
    uint32_t N; //128
    uint32_t K; //128
    uint32_t auto_delta; //1
    uint32_t INTERRUPT_BY_ANGLE_PATH; // 0
    float W_MIN; //3.14
    float W_MAX; //18.84
    float K_predict; //0.5
    float APS_DELTA; //PI/360
    float history_angle; //(20*PI)/180
    float K_delta; //2.5
};
extern struct INC_SET *Settings;

struct SET_OF_PRESETS //16
{
   uint16_t Signature; // 0x5AA5 - ��� ����������� �������� ����� �� �����
   uint16_t TBTIndex;  // ������ � ������� Tx_by_turn
   uint16_t MinRotSpeed; // ����������� �������� �������� ��� ������������ ��������� (���� - ��������� �� ��������������)
   uint16_t DebugSession; // byte1=�������� byte0=���������� ������ (��/���)
   uint64_t DateTime;
};
extern struct SET_OF_PRESETS SetOfPresets;

struct TURN_PRESETS
{
    uint32_t Tx_undirect[6];// ������� ������ ������������ � ������ �������������� ���������
    uint32_t Tx_by_turn[3][5][6]; // ������� ������ ������������ � ������ ������������ ��������� ��� ������ ��������� �������� ������� �������
    uint32_t tx_work_time_units[7];// = { 5,5,6,7,8,9,0 };//����� ������ ������ ����������� � �������� ����� ������ �������� ��������� (1 ��)
};

struct REAP_CONST
{
    float ATT_max[2][4];
    uint32_t ATT_grad[2][4];
    float PH_max[2][4];
    uint32_t PH_grad[2][4];
};
extern struct REAP_CONST reap;

struct GEO_CAL
{
    float Beta_Z[2][4];// ������������� ���� ������� ������� � �������� ��� ������������� 2 ������� 4 �����������
    float V_zx_colar_add[2][4];//���������� �������� � V_xz yf �� ������� �������, �������������� �� SSI 2 ������� 4 �����������
    float Vxz_Vzz_air_d_ph[2][4];//������� ��� ����� Vxz � Vzz �� ������� 2 ������� 4 �����������
};

struct TGMW
{
    struct Vec G;
    struct Vec M;
    struct Vec W;
};
extern struct TGMW GMW;
//---------------------------------------------------------------------------------------------------------------------------------
struct METROLOGY_GP
{
    uint32_t signature;
    uint32_t serial;
    uint16_t L1[5];//
    uint16_t L2[5];//
    uint16_t F[2];//
    //�������� ��������
    int16_t air_ph[2][5];///�������� ��� �� ������ ��� ���
    //�������� ��������
    int16_t min_amp[2][5];///����������� ���������, ������������ ����������������� ����������� ��� �������������� ��������
    uint32_t D_sonde_mm;
    uint32_t work_type;
    uint32_t Rx_Position;//0-DEFAULT R1->T1, 1-R1->T2;
    float air_att_dB[2][5];//��������� �������� �������� �� ������ ��� ���
    uint16_t service[58];//�� ������� �� 240 ���� !!!!
};
extern struct METROLOGY_GP MetroGp;

struct METROLOGY_CARTOGRAPH
{
 uint32_t signature;
 uint32_t serial;
 uint16_t L1[5];//
 uint16_t L2[5];//
 uint16_t F[2];//
 int16_t air_ph[2][5];//������ �������������� ���������
 int16_t min_amp[2][5];//����������� ���������, ������������ ����������������� ����������� ��� �������������� ���������
 uint32_t D_sonde_mm;//������� �������
 uint32_t work_type;// ���� 0 - �� ��������� (�� ���������), ���� 1- �� ������� ������������
 uint32_t Rx_Position;//0-DEFAULT R1->T1, 1-R1->T2;
 float air_att_dB[2][5];
 uint16_t service[58];//�� ������� �� 240 ���� !!!!
 //����� ��������� ����� �����////////////////////////////////////////////////////////////////////////////////////////
 uint32_t L_geo[5];//
 struct INC_SET inc_set;
 struct TURN_PRESETS turn_presets;
 uint64_t unix_epoh_data;//���� � ����� ����
 struct REAP_CONST reap;
 uint32_t select_key;// ����, ���������� �� �������  24 ������� ���� �������� �� ��������� �������(������)
 // ���������� ��� ������������ ���������, ������������ ����� �� ������������ ���������.
 // Left_, Right_   _400_kGz, _2000_kGz, T1, T2, T3, T4
 float betta_Z_deg[2][2][4];// ������������� ���� ������� ������� � �������� ��� ������������� 2 ��������� 2 ������� 4 �����������
 float V_zx_colar_add[2][2][4];//���������� �������� � V_xz yf �� ������� �������, �������������� �� SSI 2 ��������� 2 ������� 4 �����������
 float d_ph_Vzx_Vzz_mG[2][2][4];//������� ��� ����� Vxz � Vzz �� ������� � ������������ 2 ��������� 2 ������� 4 �����������
 uint16_t service1[242];//�� ������� �� 1536 ���� !!!!
};
extern struct METROLOGY_CARTOGRAPH Metro;

struct ID
{
    uint32_t struct_size;// ��������� ������ ��������� ������
    uint32_t type_;// ��� - ���� �����
    uint32_t N_Tx;//���������� ������������
    uint32_t mod;//�����������
    uint32_t number;//���������� �����
    uint32_t type;// ��� - ��� �����: ���������� ���, ���������� ������������ � �����������
    //  ����; 1 - ���������� 2 - LWD, 3 - ���������
};

enum Air_type {PH, ATT};

extern bool bExtCmdGetData;
extern uint64_t gPktTimeEXT;
extern struct TUartCMD UartCmd; //BT
extern struct TUartCMD UartCmdExt; //EXT

#define air_aver 30  // ����������
extern bool air_write_flg;
extern volatile uint16_t w_ma[2];//���� ����������� �������� ��� ���� ������
extern float air_buff[air_aver][2][2][5];//����� ����������� �������� ��� ���� ������, ATT � PH, ������� ������������
extern float air_summ[2][2][5];//      ��������� ����������� �������� ��� ���� ������, ATT � PH, ������� ������������

void TOneWorkCycle_Create();
//void ReadMetrology(uint16_t *metroData);
void GetCmd();
void WorkCyclogram();

void CAN_Send_Broadcast();
uint16_t CAN_GetRxdata(uint16_t rxNum);
uint16_t CAN_GetRxdataRAW(uint16_t rxNum, uint16_t txNum);
void CAN_Send(uint16_t *data);
void PackBuffer(uint16_t *src, uint16_t *dest, uint16_t n_words);
void ParseExtCmd(void);
void TransmitDataExt(uint16_t *pData, uint16_t words, uint16_t cmdcode);


uint16_t CheckMetroData(void);
float GetWlim(void);
void formula_simmetry(float K[5][5], uint16_t condition, uint16_t N_Tx);
void simmetry(struct ALLDATA *data, struct METROLOGY_CARTOGRAPH *metro, uint16_t freq_idx, uint16_t condition, uint16_t N_Tx);
uint16_t GetTXCondition(struct ALLDATA *measure_data, struct METROLOGY_CARTOGRAPH *metrology_data, uint16_t freq);
int32_t RO_ARG(struct METROLOGY_CARTOGRAPH *metrology, struct ALLDATA *Data, uint16_t n, uint16_t freq);
int32_t RO_ATT(struct METROLOGY_CARTOGRAPH *metro, struct ALLDATA *data, uint16_t n, uint16_t freq_idx);
complex float SIGNAL(struct METROLOGY_CARTOGRAPH *metro, uint16_t n, uint16_t freq_idx, float ro);
//void calc_geo_smt(struct ALLDATA* data);
void OutCompressedData(struct ALLDATA *data, struct METROLOGY_CARTOGRAPH *metro);
void calc_geo_signal_smt(struct METROLOGY_CARTOGRAPH *metro, struct ALLDATA * data , uint16_t freq_idx);
void transform_data(struct ALLDATA *data, struct GP_DATA *gp_data); //������ GP Data �� ALLDATA[0...1]
struct ID get_sonde_id(uint32_t signature);
void DefaultMetro();
uint16_t SetWorkProfile(uint32_t profile);
uint16_t GetWorkType(uint32_t signature);
void test();

#endif /* _GLOBALS_H_ */

