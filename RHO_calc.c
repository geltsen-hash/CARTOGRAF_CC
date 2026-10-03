/*
 * RHO_calc.c
 *
 *  Created on: 13 мая 2025 г.
 *      Author: user
 */

#include "_globals.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_ITER_RO 40

//дополнительные переменные для автоматического воздуха, объявляются глобально
bool air_write_flg = false;
volatile uint16_t w_ma[2] = {0, 0};//окно скользящего среднего для двух частот
float air_buff[air_aver][2][2][5];//буфер скользящего среднего для двух частот, ATT и PH, четырех передатчиков
float air_summ[2][2][5];//      результат скользящего среднего для двух частот, ATT и PH, четырех передатчиков

//const float mV = 4000.0f / pow(2.0f, 24.0f);
// float mG = 1000.0f * 180.0f / PI;
//const float Grad = 180.0f / PI;

// прямая задача
// n - номер передатчика 0-Т1, 1-Т2, 2-Т3, 3-Т4. freq - номер частоты 0-400кГц, 1-2000кГц
// расчитывает разницу фаз от УЭС для одной частоты от одного передатчика
// требуемые длины и частоту берет из структуры метрологии для данного передатчика
complex float SIGNAL(struct METROLOGY_CARTOGRAPH *metro, uint16_t n, uint16_t freq_idx, float ro)
{
    float L1 = metro->L1[n] / 1000.0;
    float L2 = metro->L2[n] / 1000.0;
    float omegamu0sigma = (0.0078957 * metro->F[freq_idx]) / ro;
    complex float ik = _Imaginary_I * csqrtf(_Imaginary_I * omegamu0sigma);
    complex float SGN = cexpf(ik*(L2 - L1)) * ((1.0 - ik * L2) / (1.0 - ik * L1));
    return SGN;
}

//Определяет работоспособность передатчиков по их амплитудам для одной частоты
uint16_t GetTXCondition(struct ALLDATA *data, struct METROLOGY_CARTOGRAPH *metro, uint16_t freq_idx) //25.08.26
{
    //0b00054321
    uint16_t condition = 0;
    const float mV = 4000.0f / pow(2.0f, 24.0f);
    //для каждого передатчика вычисляем амплитуду в мВ и сравниваем ее с пороговой, записанной в файле метрологии
    //00004321 - соответствие передатчика биту в condition
    for (int Tx = 0; Tx < 4; Tx++)
    {
        if(cabsf(data->R_zz.Rzz2[Tx])*mV > metro->min_amp[freq_idx][Tx])
            condition |= (1 << Tx);
    }
    //добавлено- сразу отправляем в data.R_zz.condition
    data->R_zz.condition = condition;
    return condition;
}

void simmetry(struct ALLDATA *data, struct METROLOGY_CARTOGRAPH *metro, uint16_t freq_idx, uint16_t condition, uint16_t N_Tx)
{
    float d_PH[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float att_dB[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float K[5][5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for (int Tx = 0; Tx < N_Tx; Tx++){
        //из сырых данных ненаправленных измерений получаем разницу фаз, нормированную к +-PI
        d_PH[Tx] = cargf(data->R_zz.Rzz2[Tx] / data->R_zz.Rzz1[Tx]);
        while(d_PH[Tx] > PI) d_PH[Tx] -= 2.0f*PI;
        while(d_PH[Tx] <= -PI) d_PH[Tx] += 2.0f*PI;
        //из сырых данных ненаправленных измерений получаем амплитудное затухание в дБ
        att_dB[Tx] = 20.0f*log10(cabsf(data->R_zz.Rzz2[Tx] / data->R_zz.Rzz1[Tx]));
    }

    // для автоматического воздуха
    for (int Tx = 0; Tx < N_Tx; Tx++){
        //скользящее среднее  для воздуха (теперь отдельно для амплиткдного затухания и фаз)
        air_summ[freq_idx][PH] [Tx] -= air_buff[w_ma[freq_idx]][freq_idx] [PH][Tx];
        air_summ[freq_idx][ATT][Tx] -= air_buff[w_ma[freq_idx]][freq_idx][ATT][Tx];
        air_buff[w_ma[freq_idx]][freq_idx][PH] [Tx] = d_PH  [Tx];
        float sign = (Tx % 2 == 0) ? 1.0f : -1.0f;
            air_buff[w_ma[freq_idx]][freq_idx][ATT][Tx] = sign*att_dB[Tx]; //dir?
        air_summ[freq_idx][PH] [Tx] += air_buff[w_ma[freq_idx]][freq_idx][PH] [Tx];
        air_summ[freq_idx][ATT][Tx] += air_buff[w_ma[freq_idx]][freq_idx][ATT][Tx];
    }
    // при каждом следующем измерении инкрементируем счетчик, при превышении сбрасываем
    w_ma[freq_idx]++;
    if(w_ma[freq_idx] == air_aver) w_ma[freq_idx] = 0;
    //конец для автоматического воздуха /////

    // нормализуем сигнал к воздуху. Z1 повернут T1 phRx2-phRx1
    for (int Tx = 0; Tx < N_Tx; Tx++){
        //Если Tx четный (0, 2, 4 соответствует T1, T3, T5), sign = 1. Если нечетный (1, 3 соответствует T2, T4), sign = -1.
        float sign = (Tx % 2 == 0) ? 1.0f : -1.0f;
        // получаем калиброванные на воздух фазы и амплитудное затухание в дб
        d_PH[Tx] = sign * (d_PH[Tx] - ((float)(metro->air_ph[freq_idx][Tx])) / 57297.0);
        att_dB[Tx] = sign * att_dB[Tx] - metro->air_att_dB[freq_idx][Tx];
    }

    //рассчитываем коэф. симметризации в зависимости от работоспособности передатчиков
    //производим симметризацию и записываем в структуру Data
    formula_simmetry(K, condition, N_Tx);
    for (int Tx = 0; Tx < N_Tx; Tx++) {
            data->phase_smt[Tx] = 0.0f;
            data->att_smt[Tx] = 0.0f;
        }
    for (int Tx = 0; Tx < N_Tx; Tx++) {
        for (int n = 0; n < N_Tx; n++) {
            data->phase_smt[Tx] += K[Tx][n] * d_PH[n];
            data->att_smt[Tx] += K[Tx][n] * att_dB[n];
        }
    }
}


// переворачивает фазы, нормирует к воздуху и симметризует по формуле для одной частоты
// структуру Data требуется подставить для нужной частоты!
void formula_simmetry(float K[5][5], uint16_t condition, uint16_t N_Tx) {
    memset(K, 0, 25 * sizeof(float));
    //00054321
    if (N_Tx == 5) {
        if (condition == 0b00011111) {// работают все 5 передатчиков
            K[T1][T1] = +0.75;  K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.25;  K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00;  K[T3][T2] = +0.25; K[T3][T3] = +0.50; K[T3][T4] = +0.25; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00;  K[T4][T2] = +0.00; K[T4][T3] = +0.25; K[T4][T4] = +0.50; K[T4][T5] = +0.25;
            K[T5][T1] = +0.00;  K[T5][T2] = +0.00; K[T5][T3] = -0.25; K[T5][T4] = +0.50; K[T5][T5] = +0.75;
        }
        else if (condition == 0b00011110) {// не работает  1й передатчик
            K[T1][T1] = +0.00;  K[T1][T2] = +1.25; K[T1][T3] = +0.50; K[T1][T4] = -0.75; K[T1][T5] = +0.00;
            K[T2][T1] = +0.00;  K[T2][T2] = +0.75; K[T2][T3] = +0.50; K[T2][T4] = -0.25; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00;  K[T3][T2] = +0.25; K[T3][T3] = +0.50; K[T3][T4] = +0.25; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00;  K[T4][T2] = +0.00; K[T4][T3] = +0.25; K[T4][T4] = +0.50; K[T4][T5] = +0.25;
            K[T5][T1] = +0.00;  K[T5][T2] = +0.00; K[T5][T3] = -0.25; K[T5][T4] = +0.50; K[T5][T5] = +0.75;
        }
        else if (condition == 0b00011101) {// не работает  2й передатчик
            K[T1][T1] = +1.25;  K[T1][T2] = +0.00; K[T1][T3] = -0.75; K[T1][T4] = +0.50; K[T1][T5] = +0.00;
            K[T2][T1] = +0.75;  K[T2][T2] = +0.00; K[T2][T3] = -0.25; K[T2][T4] = +0.50; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00;  K[T3][T2] = +0.00; K[T3][T3] = +0.75; K[T3][T4] = +0.50; K[T3][T5] = -0.25;
            K[T4][T1] = +0.00;  K[T4][T2] = +0.00; K[T4][T3] = +0.25; K[T4][T4] = +0.50; K[T4][T5] = +0.25;
            K[T5][T1] = +0.00;  K[T5][T2] = +0.00; K[T5][T3] = -0.25; K[T5][T4] = +0.50; K[T5][T5] = +0.75;
        }
        else if (condition == 0b00011011) {// не работает  3й передатчик
            K[T1][T1] = +0.50;  K[T1][T2] = +0.75; K[T1][T3] = +0.00; K[T1][T4] = -0.25; K[T1][T5] = +0.00;
            K[T2][T1] = +0.00;  K[T2][T2] = +1.25; K[T2][T3] = +0.00; K[T2][T4] = -0.75; K[T2][T5] = +0.50;
            K[T3][T1] = +0.00;  K[T3][T2] = +0.75; K[T3][T3] = +0.00; K[T3][T4] = -0.25; K[T3][T5] = +0.50;
            K[T4][T1] = +0.00;  K[T4][T2] = +0.25; K[T4][T3] = +0.00; K[T4][T4] = +0.25; K[T4][T5] = +0.50;
            K[T5][T1] = +0.00;  K[T5][T2] = -0.25; K[T5][T3] = +0.00; K[T5][T4] = +0.75; K[T5][T5] = +0.50;
        }
        else if (condition == 0b00010111) {// не работает  4й передатчик
            K[T1][T1] = +0.75;  K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.25;  K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = -0.25;  K[T3][T2] = +0.50; K[T3][T3] = +0.75; K[T3][T4] = +0.00; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00;  K[T4][T2] = +0.50; K[T4][T3] = -0.25; K[T4][T4] = +0.00; K[T4][T5] = +0.75;
            K[T5][T1] = +0.00;  K[T5][T2] = +0.50; K[T5][T3] = -0.75; K[T5][T4] = +0.00; K[T5][T5] = +1.25;
        }
        else if (condition == 0b00001111) {// не работает  5й передатчик
            K[T1][T1] = +0.75;  K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.25;  K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00;  K[T3][T2] = +0.25; K[T3][T3] = +0.50; K[T3][T4] = +0.25; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00;  K[T4][T2] = -0.25; K[T4][T3] = +0.50; K[T4][T4] = +0.75; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00;  K[T5][T2] = -0.75; K[T5][T3] = +0.50; K[T5][T4] = +1.25; K[T5][T5] = +0.00;
        }
        else if (condition == 0b00000000) {// выводим несимметризованные значения для всех передатчиков
            K[T1][T1] = +1.00; K[T1][T2] = +0.00; K[T1][T3] = +0.00; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.00; K[T2][T2] = +1.00; K[T2][T3] = +0.00; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00; K[T3][T2] = +0.00; K[T3][T3] = +1.00; K[T3][T4] = +0.00; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00; K[T4][T2] = +0.00; K[T4][T3] = +0.00; K[T4][T4] = +1.00; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +1.00;
        }
        else {
            // работает меньше четырех передатчиков
            //находим рабочие передатчики и для них выводим несимметризованные значения, для нерабочих фаза равна 0
            bool k1 = (condition >> 0) & 1u;
            bool k2 = (condition >> 1) & 1u;
            bool k3 = (condition >> 2) & 1u;
            bool k4 = (condition >> 3) & 1u;
            bool k5 = (condition >> 4) & 1u;
            K[T1][T1] = 1.00*k1; K[T1][T2] = 0.00;    K[T1][T3] = 0.00;    K[T1][T4] = 0.00;    K[T1][T5] = 0.00;
            K[T2][T1] = 0.00;    K[T2][T2] = 1.00*k2; K[T2][T3] = 0.00;    K[T2][T4] = 0.00;    K[T2][T5] = 0.00;
            K[T3][T1] = 0.00;    K[T3][T2] = 0.00;    K[T3][T3] = 1.00*k3; K[T3][T4] = 0.00;    K[T3][T5] = 0.00;
            K[T4][T1] = 0.00;    K[T4][T2] = 0.00;    K[T4][T3] = 0.00;    K[T4][T4] = 1.00*k4; K[T4][T5] = 0.00;
            K[T5][T1] = 0.00;    K[T5][T2] = 0.00;    K[T5][T3] = 0.00;    K[T5][T4] = 0.00;    K[T5][T5] = 1.00*k5;
        }
    }

    //00004321
    if (N_Tx == 4) {
        if (condition == 0b00001111) {// работают все 4 передатчика 5й не существует
            K[T1][T1] = +0.75;  K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.25;  K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00;  K[T3][T2] = +0.25; K[T3][T3] = +0.50; K[T3][T4] = +0.25; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00;  K[T4][T2] = -0.25; K[T4][T3] = +0.50; K[T4][T4] = +0.75; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00;  K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +0.00;
        }
        else if (condition == 0b00001110) {// не работает  1й передатчик 5й не существует
            K[T1][T1] = +0.00; K[T1][T2] = +1.25; K[T1][T3] = +0.50; K[T1][T4] = -0.75; K[T1][T5] = +0.00;
            K[T2][T1] = +0.00; K[T2][T2] = +0.75; K[T2][T3] = +0.50; K[T2][T4] = -0.25; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00; K[T3][T2] = +0.25; K[T3][T3] = +0.50; K[T3][T4] = +0.25; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00; K[T4][T2] = -0.25; K[T4][T3] = +0.50; K[T4][T4] = +0.75; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +0.00;
        }
        else if (condition == 0b00001101) {// не работает  2й передатчик 5й не существует
            K[T1][T1] = +1.25; K[T1][T2] = +0.00; K[T1][T3] = -0.75; K[T1][T4] = +0.50; K[T1][T5] = +0.00;
            K[T2][T1] = +0.75; K[T2][T2] = +0.00; K[T2][T3] = -0.25; K[T2][T4] = +0.50; K[T2][T5] = +0.00;
            K[T3][T1] = +0.25; K[T3][T2] = +0.00; K[T3][T3] = +0.25; K[T3][T4] = +0.50; K[T3][T5] = +0.00;
            K[T4][T1] = -0.25; K[T4][T2] = +0.00; K[T4][T3] = +0.75; K[T4][T4] = +0.50; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +0.00;
        }
        else if (condition == 0b00001011) {// не работает  3й передатчик 5й не существует
            K[T1][T1] = +0.50; K[T1][T2] = +0.75; K[T1][T3] = +0.00; K[T1][T4] = -0.25; K[T1][T5] = +0.00;
            K[T2][T1] = +0.50; K[T2][T2] = +0.25; K[T2][T3] = +0.00; K[T2][T4] = +0.25; K[T2][T5] = +0.00;
            K[T3][T1] = +0.50; K[T3][T2] = -0.25; K[T3][T3] = +0.00; K[T3][T4] = +0.75; K[T3][T5] = +0.00;
            K[T4][T1] = -0.50; K[T4][T2] = -0.75; K[T4][T3] = +0.00; K[T4][T4] = +1.25; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = -0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +0.00;
        }
        else if (condition == 0b00000111) {// не работает  4й передатчик 5й не существует
            K[T1][T1] = +0.75; K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.25; K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = -0.25; K[T3][T2] = +0.50; K[T3][T3] = +0.75; K[T3][T4] = +0.00; K[T3][T5] = +0.00;
            K[T4][T1] = -0.75; K[T4][T2] = +0.50; K[T4][T3] = +1.25; K[T4][T4] = +0.00; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +0.00;
        }
        else if (condition == 0b00000000) {// выводим несимметризованные значения для всех передатчиков
            K[T1][T1] = +1.00; K[T1][T2] = +0.00; K[T1][T3] = +0.00; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.00; K[T2][T2] = +1.00; K[T2][T3] = +0.00; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00; K[T3][T2] = +0.00; K[T3][T3] = +1.00; K[T3][T4] = +0.00; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00; K[T4][T2] = +0.00; K[T4][T3] = +0.00; K[T4][T4] = +1.00; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +1.00;
        }
        else {
            // работает меньше трех передатчиков
            //находим рабочие передатчики и для них выводим несимметризованные значения, для нерабочих фаза равна 0
            bool k1 = (condition >> 0) & 1u;
            bool k2 = (condition >> 1) & 1u;
            bool k3 = (condition >> 2) & 1u;
            bool k4 = (condition >> 3) & 1u;
            bool k5 = (condition >> 4) & 1u;
            K[T1][T1] = 1.00*k1; K[T1][T2] = 0.00;    K[T1][T3] = 0.00;    K[T1][T4] = 0.00;    K[T1][T5] = 0.00;
            K[T2][T1] = 0.00;    K[T2][T2] = 1.00*k2; K[T2][T3] = 0.00;    K[T2][T4] = 0.00;    K[T2][T5] = 0.00;
            K[T3][T1] = 0.00;    K[T3][T2] = 0.00;    K[T3][T3] = 1.00*k3; K[T3][T4] = 0.00;    K[T3][T5] = 0.00;
            K[T4][T1] = 0.00;    K[T4][T2] = 0.00;    K[T4][T3] = 0.00;    K[T4][T4] = 1.00*k4; K[T4][T5] = 0.00;
            K[T5][T1] = 0.00;    K[T5][T2] = 0.00;    K[T5][T3] = 0.00;    K[T5][T4] = 0.00;    K[T5][T5] = 1.00*k5;
        }
    }

    //00000321
    if (N_Tx == 3) {
        if (condition == 0b00000111) {// 4й и 5й передатчики  не существуют
            K[T1][T1] = +0.75; K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.25; K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = -0.25; K[T3][T2] = +0.50; K[T3][T3] = +0.75; K[T3][T4] = +0.00; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00; K[T4][T2] = +0.00; K[T4][T3] = +0.00; K[T4][T4] = +0.00; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +0.00;
        }
        else {// выводим несимметризованные значения для всех передатчиков
            K[T1][T1] = +1.00; K[T1][T2] = +0.00; K[T1][T3] = +0.00; K[T1][T4] = +0.00; K[T1][T5] = +0.00;
            K[T2][T1] = +0.00; K[T2][T2] = +1.00; K[T2][T3] = +0.00; K[T2][T4] = +0.00; K[T2][T5] = +0.00;
            K[T3][T1] = +0.00; K[T3][T2] = +0.00; K[T3][T3] = +1.00; K[T3][T4] = +0.00; K[T3][T5] = +0.00;
            K[T4][T1] = +0.00; K[T4][T2] = +0.00; K[T4][T3] = +0.00; K[T4][T4] = +0.00; K[T4][T5] = +0.00;
            K[T5][T1] = +0.00; K[T5][T2] = +0.00; K[T5][T3] = +0.00; K[T5][T4] = +0.00; K[T5][T5] = +0.00;
        }

    }
}
//-------------------------------------------------------------------------------------------------------------
/* --- Вычисление фазового УЭС для зонда КАРТОГРАФ (RO_ARG) --- */
int32_t RO_ARG(struct METROLOGY_CARTOGRAPH *metro, struct ALLDATA *data, uint16_t n, uint16_t freq_idx) {
    float Ro0 = 7200.0f;
    const float epsilon_ARG = 1e-5f;            /* Точность по фазе (~0.00057 град) */
    const float Ro_0 = 0.01f, Ro_max = 7000.0f;  /* Границы поиска УЭС */

    float target_phase = data->phase_smt[n];

    /* Оценка на границах диапазона чувствительности */
    float phase_min = cargf(SIGNAL(metro, n, freq_idx, Ro_0));
    float phase_max = cargf(SIGNAL(metro, n, freq_idx, Ro_max));

    if (target_phase <= phase_max) {
        Ro0 = 7200.0f; /* Выше предела чувствительности зонда */
    }
    else if (target_phase >= phase_min) {
        Ro0 = Ro_0;    /* Ниже нижнего предела */
    }
    else {
        float ro_left = Ro_0;
        float ro_right = Ro_max;

        for (int iter = 0; iter < MAX_ITER_RO; ++iter) {
            float ro_mid = 0.5f * (ro_left + ro_right);
            float cur_phase = cargf(SIGNAL(metro, n, freq_idx, ro_mid));
            float diff = cur_phase - target_phase;

            /* Критерий сходимости: по невязке фазы или относительной ширине интервала */
            if (fabsf(diff) < epsilon_ARG || (ro_right - ro_left) < 1e-4f * ro_mid) {
                ro_left = ro_mid;
                ro_right = ro_mid;
                break;
            }

            /* Фаза монотонно убывает с ростом сопротивления ro */
            if (diff > 0.0f) {
                ro_left = ro_mid;   /* Увеличиваем ro для уменьшения фазы */
            } else {
                ro_right = ro_mid;  /* Уменьшаем ro для увеличения фазы */
            }
        }
        Ro0 = 0.5f * (ro_left + ro_right);
    }

    data->rho_ph_smt[n] = Ro0;
    return 0;
}
//-----------------------------------------------------------------------------------------------------------
/* --- Вычисление амплитудного УЭС для зонда КАРТОГРАФ (RO_ATT) --- */
int32_t RO_ATT(struct METROLOGY_CARTOGRAPH *metro, struct ALLDATA *data, uint16_t n, uint16_t freq_idx) {
    float Ro0 = 1200.0f;
    const float epsilon_ATT = 1e-5f;            /* Точность по амплитудному коэффициенту */
    const float Ro_0 = 0.01f, Ro_max = 1000.0f;  /* Границы поиска УЭС */

    /* Перевод симметризованного затухания из дБ в линейное отношение амплитуд */
    float target_att = powf(10.0f, (data->att_smt[n]) / 20.0f);

    /* Оценка на границах диапазона чувствительности */
    float att_min = cabsf(SIGNAL(metro, n, freq_idx, Ro_0));
    float att_max = cabsf(SIGNAL(metro, n, freq_idx, Ro_max));

    if (target_att >= att_max) {
        Ro0 = 1200.0f; /* Выше предела чувствительности зонда */
    }
    else if (target_att <= att_min) {
        Ro0 = Ro_0;    /* Ниже нижнего предела */
    }
    else {
        float ro_left = Ro_0;
        float ro_right = Ro_max;

        for (int iter = 0; iter < MAX_ITER_RO; ++iter) {
            float ro_mid = 0.5f * (ro_left + ro_right);
            float cur_att = cabsf(SIGNAL(metro, n, freq_idx, ro_mid));
            float diff = cur_att - target_att;

            /* Критерий сходимости */
            if (fabsf(diff) < epsilon_ATT || (ro_right - ro_left) < 1e-4f * ro_mid) {
                ro_left = ro_mid;
                ro_right = ro_mid;
                break;
            }

            /* Линейный сигнал монотонно возрастает с ростом сопротивления ro */
            if (diff < 0.0f) {
                ro_left = ro_mid;   /* Увеличиваем ro для увеличения амплитуды */
            } else {
                ro_right = ro_mid;  /* Уменьшаем ro для уменьшения амплитуды */
            }
        }
        Ro0 = 0.5f * (ro_left + ro_right);
    }

    data->rho_att_smt[n] = Ro0;
    return 0;
}
//------------------------------------------------------------------------------------------------------------
//Калибровка пришедших от направленных приемников данных  amp_Vzz, amp_Vzx[4];//16, ph_Vzx_Vzz
// и формирование геосигнала.
void calc_geo_signal_smt(struct METROLOGY_CARTOGRAPH *metro, struct ALLDATA * data , uint16_t freq_idx){

    for (int Tx = 0; Tx < 4; Tx++){
        complex float R_pure[2];
        for (int N_rx = 0; N_rx < 2; N_rx++){
            float beta_rad = metro->betta_Z_deg[N_rx][freq_idx][Tx] / 57.2958f;
            float d_ph_cal = metro->d_ph_Vzx_Vzz_mG[N_rx][freq_idx][Tx] / 57295.8f;
            float V_zx_colar = metro->V_zx_colar_add[N_rx][freq_idx][Tx];
            float ph = data->R_zx[N_rx].ph_Vzx_Vzz[Tx];

            float amp_ratio = data->R_zx[N_rx].amp_Vzx[Tx] / data->R_zx[N_rx].amp_Vzz[Tx];
            complex float R_raw = amp_ratio * (cosf(ph) + sinf(ph) * I);
            //Поправка на аппаратурную разность фаз
            complex float R_ph_corr = R_raw * (cosf(-d_ph_cal) + sinf(-d_ph_cal) * I);
            //Вычитание паразитной корпусной добавки
            complex float R_corr = R_ph_corr - (complex float)V_zx_colar;
            //Приведение реального угла beta к идеальной антенне 45 градусов
            R_pure[N_rx] = R_corr * (0.70710678f / sinf(beta_rad));
            //Расчет геосигнала
            data->R_zx[N_rx].Geo[Tx] = (1.0f - R_pure[N_rx]) / (1.0f + R_pure[N_rx]);
        }
        //Расчет компенсированного геосигнала из двух приемников
        complex float R_boundary = 0.5f*(R_pure[0] - R_pure[1]);
        complex float geo_signal_smt = (1.0f - R_boundary) / (1.0f + R_boundary);
        data-> ATT_dB_geo_signal_smt[Tx] = 20.0f*log10f(cabsf(geo_signal_smt));
        data-> PH_deg_geo_signal_smt[Tx] = 57.296f*(cargf(geo_signal_smt));
        //Расчет сигнала от анизотропии из двух приемников
        //следующий этап
        //complex float R_aniso = 0.5f*(R_pure[0] + R_pure[1]);
        //complex float geo_signal_aniso = (1.0f - R_aniso) / (1.0f + R_aniso);
        //data-> ATT_dB_geo_signal_aniso[Tx] = 20.0f*log10f(cabsf(geo_signal_aniso));
        //data-> PH_deg_geo_signal_aniso[Tx] = 57.296f*(cargf(geo_signal_aniso));
    }
}
