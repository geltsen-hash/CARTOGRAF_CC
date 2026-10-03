/*
 * test.c
 *
 *  Created on: 23 мар. 2026 г.
 *      Author: Admin
 */

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
#include <complex.h>
#include <stdint.h>
#include <string.h>
//#include "C28x_FPU_FastRTS.h"


#pragma DATA_SECTION(TRIG_LUT, ".lut");
float TRIG_LUT[64] = {
    1.0000000f,  0.0000000f,  1.0000000f,  0.0000000f, // n=0
    0.9238795f,  0.3826834f,  0.7071068f,  0.7071068f, // n=1
    0.7071068f,  0.7071068f,  0.0000000f,  1.0000000f, // n=2
    0.3826834f,  0.9238795f, -0.7071068f,  0.7071068f, // n=3
    0.0000000f,  1.0000000f, -1.0000000f,  0.0000000f, // n=4
   -0.3826834f,  0.9238795f, -0.7071068f, -0.7071068f, // n=5
   -0.7071068f,  0.7071068f,  0.0000000f, -1.0000000f, // n=6
   -0.9238795f,  0.3826834f,  0.7071068f, -0.7071068f, // n=7
   -1.0000000f,  0.0000000f,  1.0000000f,  0.0000000f, // n=8
   -0.9238795f, -0.3826834f,  0.7071068f,  0.7071068f, // n=9
   -0.7071068f, -0.7071068f,  0.0000000f,  1.0000000f, // n=10
   -0.3826834f, -0.9238795f, -0.7071068f,  0.7071068f, // n=11
    0.0000000f, -1.0000000f, -1.0000000f,  0.0000000f, // n=12
    0.3826834f, -0.9238795f, -0.7071068f, -0.7071068f, // n=13
    0.7071068f, -0.7071068f,  0.0000000f, -1.0000000f, // n=14
    0.9238795f, -0.3826834f,  0.7071068f, -0.7071068f  // n=15
};
//-------------------------------------------------------------------------------------------------------
#pragma CODE_SECTION(SLAE_5x5, ".TI.ramfunc");
int SLAE_5x5(float *a, float *b, float *x) {
    for (int k = 0; k < 5; k++) {
        int k_off = k * 5;
        float max_val = fabsf(a[k_off + k]);
        int r = k;
        for (int i = k + 1; i < 5; i++) {
            float abs_val = fabsf(a[i * 5 + k]);
            if (abs_val > max_val) { max_val = abs_val; r = i; }
        }
        if (r != k) {
            int r_off = r * 5;
            for (int j = 0; j < 5; j++) {
                float tmp = a[k_off+j]; a[k_off+j] = a[r_off+j]; a[r_off+j] = tmp;
            }
            float tmp_b = b[k]; b[k] = b[r]; b[r] = tmp_b;
        }
        if (fabsf(a[k_off + k]) < 1e-7f) return -1;
        float inv = 1.0f / a[k_off + k];
        for (int i = k + 1; i < 5; i++) {
            int i_off = i * 5;
            float M = a[i_off + k] * inv;
            for (int j = k; j < 5; j++) {
                a[i_off+j] -= M * a[k_off+j];
            }
            b[i] -= M * b[k];
        }
    }
    for (int i = 4; i >= 0; i--) {
        int i_off = i * 5;
        float s = 0.0f;
        for (int j = i + 1; j < 5; j++) s += a[i_off+j] * x[j];
        x[i] = (b[i] - s)/a[i_off+i];
    }
    return 0;
}
//-------------------------------------------------------------------------------------------------------
#pragma DATA_SECTION(B, ".lut");
float B[25] = {0}; // Одномерный массив
#pragma DATA_SECTION(D, ".lut");
float D[5];
#pragma DATA_SECTION(X, ".lut");
float X[5];

#pragma CODE_SECTION(LSM_mk, ".TI.ramfunc");
int LSM_mk(const float *sgn, complex float * harm) {
    // На уровне 0 эти переменные будут в стеке (медленно)
    // На уровне 4 они будут в регистрах R0H-R7H (мгновенно)
    float d0=0, d1=0, d2=0, d3=0, d4=0;
    float b0=0, b1=0, b2=0, b3=0, b4=0, b6=0, b7=0, b8=0, b9=0, b12=0, b13=0, b14=0, b18=0, b19=0, b24=0;
    int n_cntr = 0;

    for (int n = 0; n < 16; n++) {
        float s = sgn[n];
        if (s != 0.0f) {
            n_cntr++;
            int off = n << 2;
            const float v1 = TRIG_LUT[off];
            const float v2 = TRIG_LUT[off+1];
            const float v3 = TRIG_LUT[off+2];
            const float v4 = TRIG_LUT[off+3];

            d0 += s; d1 += s * v1; d2 += s * v2; d3 += s * v3; d4 += s * v4;
            b0 += 1.0f; b1 += v1; b2 += v2; b3 += v3; b4 += v4;
            b6 += v1*v1; b7 += v1*v2; b8 += v1*v3; b9 += v1*v4;
            b12 += v2*v2; b13 += v2*v3; b14 += v2*v4;
            b18 += v3*v3; b19 += v3*v4;
            b24 += v4*v4;
        }
    }

    /*// ЧАСТЬ 1: Считаем D и первые 5 элементов B (итого 10 переменных)
    for (int n = 0; n < 16; n++) {
        float s = sgn[n];
        if (s != 0.0f) {
            int off = n << 2;
            float v1 = TRIG_LUT[off];   float v2 = TRIG_LUT[off+1];
            float v3 = TRIG_LUT[off+2]; float v4 = TRIG_LUT[off+3];
            d0 += s; d1 += s * v1; d2 += s * v2; d3 += s * v3; d4 += s * v4;
            b0 += 1.0f; b1 += v1; b2 += v2; b3 += v3; b4 += v4;
        }
    }

    // ЧАСТЬ 2: Считаем остальные элементы B (итого 10 переменных)
    for (int n = 0; n < 16; n++) {
        float s = sgn[n];
        if (s != 0.0f) {
            int off = n << 2;
            float v1 = TRIG_LUT[off];   float v2 = TRIG_LUT[off+1];
            float v3 = TRIG_LUT[off+2]; float v4 = TRIG_LUT[off+3];
            b6 += v1*v1; b7 += v1*v2; b8 += v1*v3; b9 += v1*v4;
            b12 += v2*v2; b13 += v2*v3; b14 += v2*v4;
            b18 += v3*v3; b19 += v3*v4; b24 += v4*v4;
        }
    }*/

    if (n_cntr == 0) return 1;

   // float B[25] = {0}; // Одномерный массив
   // float D[5], X[5];

    // Присвоение накопленных значений
    D[0]=d0; D[1]=d1; D[2]=d2; D[3]=d3; D[4]=d4;
    B[0]=b0; B[1]=b1; B[2]=b2; B[3]=b3; B[4]=b4;
    B[6]=b6; B[7]=b7; B[8]=b8; B[9]=b9;
    B[12]=b12; B[13]=b13; B[14]=b14;
    B[18]=b18; B[19]=b19;
    B[24]=b24;

    // Зеркальное заполнение (нижний треугольник)
    B[5]=B[1]; B[10]=B[2]; B[15]=B[3]; B[20]=B[4];
    B[11]=B[7]; B[16]=B[8]; B[21]=B[9];
    B[17]=B[13]; B[22]=B[14];
    B[23]=B[19];

    if (SLAE_5x5(B, D, X) != 0) return 1;

    //harm[0] = {X[0], 0.0f};
    //harm[1] = {X[1], X[2]};
    //harm[2] = {X[3], X[4]};
    harm[0] = CMPLXF(X[0], 0.0f);
    harm[1] = CMPLXF(X[1], X[2]);
    harm[2] = CMPLXF(X[3], X[4]);
    return 0;
}
//-------------------------------------------------------------------------------------------------------
#pragma CODE_SECTION(test, ".TI.ramfunc");
void test()
{
    float sgn[16];
    float Amp_H0 = 10.0f;
    float Amp_H1 = 5.3f;
    float fi01 = PI / 4.0f;
    //fi01 = 0.0f;
    float Amp_H2 = 2.5f;
    float fi02 = PI / 8.0f;
    //fi02 = 0.0f;

    for (int fi = 0; fi < 16; fi++) {
        sgn[fi] = Amp_H0 + Amp_H1 * cos(fi*PI / 8.0f - fi01) + Amp_H2 * cos(2.0f*fi*PI / 8.0f - fi02);
    }

    sgn[5] =  0.0; sgn[10] = 0.0; sgn[13] = 0.0;
    complex float harm[3];
    GPIO_writePin(led1, 1);
    LSM_mk(sgn, harm);
    GPIO_writePin(led1, 0);

}
//-------------------------------------------------------------------------------------------------------
/*void test()
{
    //заглушка
}*/








