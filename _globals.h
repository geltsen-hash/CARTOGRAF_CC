/*
 * _globals.h
 *
 *  Created for: kg_controller
 *  Description: Global header facade aggregating modular type definitions,
 *               hardware definitions, externs and utility macros.
 */

#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include "driverlib.h"
#include "board.h"
#include "F021_F2837xS_C28x.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <complex.h>

#include "sensor_types.h"
#include "cyclogram_types.h"
#include "metro_types.h"

#define PI 3.14159265359

extern volatile uint64_t Now;
uint64_t GetNow(void);

extern uint16_t gStartSectorIdx;
extern uint16_t gWorkType;

#define AXEL_AVG_CNT 8
#define AXEL_FILTER_CNT 8
#define MOVEMENT_CMP_VALUE 1000

#define PWM_PERIOD          3599
#define PWM_SYNC_PERIOD     90
#define PWM_SYNC_CMP        45

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

extern struct Cal G_offset_sens, M_offset_sens, W_offset_sens, G_offset_sens_add, M_offset_sens_add, W_offset_sens_add;
extern struct TGMW GMW;

// FLASH
extern bool bFlashError;
void Example_Error(Fapi_StatusType status);
void FLASH_WriteCal(struct Cal *pG, struct Cal *pM, struct Cal *pW);
void FLASH_ReadCal(struct Cal *pG, struct Cal *pM, struct Cal *pW);
void FLASH_ReadSet(struct INC_SET *pSet);
void FLASH_WriteMetrology(struct METROLOGY_CARTOGRAPH *pMetro);
void FLASH_ReadMetrology(struct METROLOGY_CARTOGRAPH *pMetro);
void FLASH_WriteSetOfPresets(struct SET_OF_PRESETS *pSetOfPresets);
void FLASH_ReadSetOfPresets(struct SET_OF_PRESETS *pSetOfPresets);

enum
{
    PAGE_W = 1,
    PAGE_X = 2,
    PAGE_Y = 3,
    PAGE_AB = 4
};
void FLASH_Erase(uint16_t page);

typedef enum
{
    PRINT_GMW,
    PRINT_APS,
    PRINT_RAW,
    PRINT_ANG,
    STOP
} uartPrintMode;

extern apsMeasState apsMMode;
extern TSTATE controller_state;

void PrintDebugInfo(void);
void ParseDebugCmd(void);

extern uint16_t gTxDataCAN[8];
extern uint16_t gRxDataCAN[8];
extern volatile uint16_t gCANError;
extern volatile uint16_t gCANRxPktCnt;
extern uint16_t gCANBuffer[256];

extern struct TOneWorkCycle wc;
extern struct GP_DATA GPData;
extern struct ALLDATA AllData[2];
extern struct raw_sector_data_all AllRxDataRAW;
extern struct INC_SET *Settings;
extern struct SET_OF_PRESETS SetOfPresets;
extern struct REAP_CONST reap;
extern struct METROLOGY_GP MetroGp;
extern struct METROLOGY_CARTOGRAPH Metro;

extern bool bExtCmdGetData;
extern uint64_t gPktTimeEXT;
struct TUartCMD;
extern struct TUartCMD UartCmd;
extern struct TUartCMD UartCmdExt;

#define air_aver 30
extern bool air_write_flg;
extern volatile uint16_t w_ma[2];
extern float air_buff[air_aver][2][2][5];
extern float air_summ[2][2][5];

void TOneWorkCycle_Create();
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
void OutCompressedData(struct ALLDATA *data, struct METROLOGY_CARTOGRAPH *metro);
void calc_geo_signal_smt(struct METROLOGY_CARTOGRAPH *metro, struct ALLDATA * data , uint16_t freq_idx);
void transform_data(struct ALLDATA *data, struct GP_DATA *gp_data);
struct ID get_sonde_id(uint32_t signature);
void DefaultMetro();
uint16_t SetWorkProfile(uint32_t profile);
uint16_t GetWorkType(uint32_t signature);
void test();

#endif /* _GLOBALS_H_ */
