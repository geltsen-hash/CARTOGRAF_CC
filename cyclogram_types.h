/*
 * cyclogram_types.h
 *
 *  Created for: kg_controller
 *  Description: States, modes and timing structures for geophysical cyclogram
 */

#ifndef CYCLOGRAM_TYPES_H_
#define CYCLOGRAM_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    STATE_IDLE,
    STATE_NORMAL, //нормальный рабочий
    STATE_NOT_CALIBRATED //режим bluetooth
} TSTATE;

typedef enum E_DMC_SUBSTATES { // DMC = Direct Measurements Cycle
    DMCSS_WaitForRequest = 0x0000, // ожидаем команду начала от платы
    DMCSS_SilentTxState = 0x0001, // излучаем паузу переднего фронта, ждем ее отсчета
    DMCSS_IntermedialState = 0x0002, // излучаем следующий сектор, ждем ее отсчета
    DMCSS_PreparingData = 0x0003, // ждем время на подготовку данных по всему циклу
    DMCSS_GettingData = 0x0004, // забираем данные от приемников Rx
    DMCSS_Perform2ndCycle = 0x0005, //для повторного 2 сезона в автоматическом режиме
    DMCSS_DONE = 0x0006 //завершено
} EDMCSubStates;

typedef enum TFREQ_MODE
{
    FMODE_400_ONLY = 1,
    FMODE_2000_ONLY = 2,
    FMODE_BOTH = 3,
    FMODE_ERROR = 0
} FMode;

#define SECTORS_COUNT 16
#define MAX_CYCLE_TIME_MS 1600
#define RX_PREPARETIME_US 6000

struct TURN_PRESETS
{
    uint32_t Tx_undirect[6];
    uint32_t Tx_by_turn[3][5][6];
    uint32_t tx_work_time_units[7];
};

struct SET_OF_PRESETS
{
    uint16_t Signature; // 0x5AA5
    uint16_t TBTIndex;
    uint16_t MinRotSpeed;
    uint16_t DebugSession;
    uint64_t DateTime;
};

struct TOneWorkCycle
{
    uint32_t TxByTurn[3][5][6];
    uint32_t Tx_undirect[6];
    uint32_t TxWorkTime[7];

    bool isDirectCycle;
    uint16_t currFreq;
    uint16_t inclinoSector;
    uint16_t turnsCount;
    uint16_t turnCounter;
    uint16_t indexTxByTurn;
    uint16_t sectorCounter;
    volatile uint16_t timeUnitCounter;
    uint16_t allTimeUnits;
    volatile uint32_t cycleStartTime_ms;
    uint16_t currTx;
    uint16_t prepareDataTimeMcsCounter;
    bool bTxActive;
    bool bFinishFlag;

    volatile bool waitNextSector;
    volatile bool wasNewSectorInt;
    volatile bool wasADCInt;

    EDMCSubStates currSubState;
    FMode freqMode;
    uint16_t delay2ndCycle;
};

enum
{
    CAN_ERROR_NOERROR,
    CAN_ERROR_TX_FAILED,
    CAN_ERROR_RX_DATA_TIMEOUT,
    CAN_ERROR_CRC
};

#endif /* CYCLOGRAM_TYPES_H_ */
