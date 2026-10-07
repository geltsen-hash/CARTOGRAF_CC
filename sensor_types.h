/*
 * sensor_types.h
 *
 *  Created for: kg_controller
 *  Description: Data types for sensors, calibration matrices, vectors and inclinometry
 */

#ifndef SENSOR_TYPES_H_
#define SENSOR_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

struct Vec
{
    float X;
    float Y;
    float Z;
};

struct MATRIX_3_3
{
    float XX; float YX; float ZX;
    float XY; float YY; float ZY;
    float XZ; float YZ; float ZZ;
};

struct Cal
{
    struct Vec offset;
    struct MATRIX_3_3 sens;
};

struct TGMW
{
    struct Vec G;
    struct Vec M;
    struct Vec W;
};

struct Inclinometer
{
    float Azm;
    float Zen;
    float Aps;
};

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

typedef enum
{
    APS_IDLE, //не используется
    APS_READY, //готовность датчика на магнитах
    APS_WAIT_NEXT_POINT, //ожидание точки
    APS_COMPLETE //расчет углов завершен
} apsMeasState;

#endif /* SENSOR_TYPES_H_ */
