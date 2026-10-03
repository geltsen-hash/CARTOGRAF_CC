/*
 * math_angles.c
 *
 *  Created on: 1 июн. 2022 г.
 *      Author: 1
 */
#include "_globals.h"
#include "math_angles.h"
#include <math.h>
#include <tgmath.h>
#include <complex.h>
//#include <cmath>

// векторные функции
//-----------------------------------------------------------------------------------------------
//скал€рное произведение векторов
float sclr_mltp(struct Vec a, struct Vec b)
{
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
}
//-----------------------------------------------------------------------------------------------
//сумма векторов
struct Vec vctr_summ(struct Vec a, struct Vec b)
{
    struct Vec result;
    result.X = a.X + b.X;
    result.Y = a.Y + b.Y;
    result.Z = a.Z + b.Z;
    return result;
}

struct Vec vctr_diff(struct Vec a, struct Vec b) {
    struct Vec result;
    result.X = a.X - b.X;
    result.Y = a.Y - b.Y;
    result.Z = a.Z - b.Z;
    return result;
}

//-----------------------------------------------------------------------------------------------
//векторное произведение
struct Vec vctr_mltp(struct Vec a,  struct Vec b)
{
    struct Vec result;
    result.X = a.Y * b.Z - a.Z * b.Y;
    result.Y = a.Z * b.X - a.X * b.Z;
    result.Z = a.X * b.Y - a.Y * b.X;
    return result;
}
//-----------------------------------------------------------------------------------------------
//произведение числа на вектор
struct Vec vctr_mltp_n(float a, struct Vec b)
{
    struct Vec result;
    result.X = a * b.X;
    result.Y = a * b.Y;
    result.Z = a * b.Z;
    return result;
}
//-----------------------------------------------------------------------------------------------
//модуль вектора
float modul(struct Vec a)
{
    return sqrtf(a.X * a.X + a.Y * a.Y + a.Z * a.Z);
}
//-----------------------------------------------------------------------------------------------
//косинус угла между векторами
float vctr_cos(struct Vec a, struct Vec b)
{
    float denom = modul(a) * modul(b);
    if (denom < 1e-7f)
    {
        return 1.0f;
    }
    return sclr_mltp(a, b) / denom;
}

struct Vec mtrx_vctr_mltp(struct MATRIX_3_3 m, struct Vec v) {
    struct Vec result;
    result.X = v.X*m.XX + v.Y *m.YX + v.Z *m.ZX;
    result.Y = v.X*m.XY + v.Y *m.YY + v.Z *m.ZY;
    result.Z = v.X*m.XZ + v.Y *m.YZ + v.Z *m.ZZ;
    return result;
}

struct MATRIX_3_3 mtrx_1(struct MATRIX_3_3 m) {
    struct MATRIX_3_3 m_1;
    float det = m.XX * (m.YY * m.ZZ - m.ZY * m.YZ) -
        m.XY * (m.YX * m.ZZ - m.YZ * m.ZX) +
        m.XZ * (m.YX * m.ZY - m.YY * m.ZX);
    if (fabsf(det) < 1e-9f)
    {
        m_1.XX = 1.0f; m_1.XY = 0.0f; m_1.XZ = 0.0f;
        m_1.YX = 0.0f; m_1.YY = 1.0f; m_1.YZ = 0.0f;
        m_1.ZX = 0.0f; m_1.ZY = 0.0f; m_1.ZZ = 1.0f;
        return m_1;
    }
    float det_1 = 1.0f / det;
    m_1.XX = (m.YY*m.ZZ - m.ZY*m.YZ) * det_1;
    m_1.XY = (m.XZ*m.ZY - m.XY*m.ZZ) * det_1;
    m_1.XZ = (m.XY*m.YZ - m.XZ*m.YY) * det_1;
    m_1.YX = (m.YZ*m.ZX - m.YX*m.ZZ) * det_1;
    m_1.YY = (m.XX*m.ZZ - m.XZ*m.ZX) * det_1;
    m_1.YZ = (m.YX*m.XZ - m.XX*m.YZ) * det_1;
    m_1.ZX = (m.YX*m.ZY - m.ZX*m.YY) * det_1;
    m_1.ZY = (m.ZX* m.XY - m.XX*m.ZY) * det_1;
    m_1.ZZ = (m.XX* m.YY - m.YX*m.XY) * det_1;
    return m_1;
}

struct MATRIX_3_3 mtrx_mltp(struct MATRIX_3_3 m, struct MATRIX_3_3 m1 ) {
    struct MATRIX_3_3 result;
    result.XX = m.XX*m1.XX + m.YX*m1.XY + m.ZX*m1.XZ;
    result.XY = m.XY*m1.XX + m.YY*m1.XY + m.ZY*m1.XZ;
    result.XZ = m.XZ*m1.XX + m.YZ*m1.XY + m.ZZ*m1.XZ;

    result.YX = m.XX*m1.YX + m.YX*m1.YY + m.ZX*m1.YZ;
    result.YY = m.XY*m1.YX + m.YY*m1.YY + m.ZY*m1.YZ;
    result.YZ = m.XZ*m1.YX + m.YZ*m1.YY + m.ZZ*m1.YZ;

    result.ZX = m.XX*m1.ZX + m.YX*m1.ZY + m.ZX*m1.ZZ;
    result.ZY = m.XY*m1.ZX + m.YY*m1.ZY + m.ZY*m1.ZZ;
    result.ZZ = m.XZ*m1.ZX + m.YZ*m1.ZY + m.ZZ*m1.ZZ;
    return result;
}

struct MATRIX_3_3 mtrx_dv(struct MATRIX_3_3 m, struct MATRIX_3_3 m1) {
    return mtrx_mltp( m, (mtrx_1(m1)));
}

void S_X(float S_x[5], int N) {
    for (int i = 0; i < 5; i++) {
        S_x[i] = 0;
    }
    for (int x = 0; x < N; x++) {
        S_x[1] += x;
        S_x[2] += pow(x, 2);
        S_x[3] += pow(x, 3);
        S_x[4] += pow(x, 4);
    }
    S_x[0] = N * S_x[4] * S_x[2] + S_x[3] * S_x[2] * S_x[1] + S_x[3]* S_x[2] * S_x[1] - S_x[2] * S_x[2] * S_x[2] - S_x[4] * S_x[1] * S_x[1] - N * S_x[3] *S_x[3];
}

float predict(float *abc, float *data, float S_x[5], int N) {
    float S_y = 0, S_xy = 0, S_x2y = 0;//
    float Delta_1 = 0, Delta_2 = 0, Delta_3 = 0;//
    abc[0] = 0; abc[1] = 0; abc[2] = 0;

    for (int x = 0; x < N; x++) {
        S_y += data[x];
        S_xy += x * data[x];
        S_x2y += x * x * data[x];
    }
    // считаем главный и 3 частных оператора матрицы и вычисл€ем коэфф. при полиноме
    Delta_1 = N * S_x2y * S_x[2] + S_xy * S_x[1] * S_x[2] + S_y * S_x[3] * S_x[1] - S_y * S_x[2] * S_x[2] - S_x[1] * S_x[1] * S_x2y - N * S_x[3] * S_xy;
    Delta_2 = N * S_x[4] * S_xy + S_x[3] * S_x[2] * S_y + S_x[2] * S_x[1] * S_x2y - S_x[2] * S_x[2] * S_xy - S_x[4] * S_y * S_x[1] - N * S_x[3] * S_x2y;
    Delta_3 = S_x[4] * S_x[2] * S_y + S_x[3] * S_x[1] * S_x2y + S_x[3] * S_x[2] * S_xy - S_x[2] * S_x[2] * S_x2y - S_x[4] * S_x[1] * S_xy - S_x[3] * S_x[3] * S_y;
    abc[0] = Delta_1 / S_x[0]; abc[1] = Delta_2 / S_x[0]; abc[2] = Delta_3 / S_x[0];
    float prediction = ((N)*(N)*abc[0] + (N)* abc[1] + abc[2]);
    return prediction;
}

struct Cal add_cal(struct Cal first, struct Cal second) {
    struct Cal result;
    result.offset = vctr_summ(first.offset , (mtrx_vctr_mltp(mtrx_1(first.sens), second.offset)));
    result.sens = mtrx_mltp(second.sens, first.sens);
    return result;
}

struct Vec callibrate(struct Vec Data , struct Cal calib) {
return  mtrx_vctr_mltp(calib.sens, (vctr_diff(Data, calib.offset)));
}

