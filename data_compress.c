/*
 * data_compress.c
 *
 *  Created on: 22 дек. 2025 г.
 *      Author: user
 */
#include "_globals.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define WORK_400 24
#define WORK_2000 25

struct REAP_CONST reap =
{
    { {0.1f, 0.42f, 1.2f, 2.7f},     {1.0f, 4.1f, 7.6f, 10.0f} },
    { {31, 63, 255, 511} ,           {63,  255, 511, 1024} },
    { {4.0f, 10.0f, 18.0f, 26.0f},   {18.0f, 30.0f, 42.0f, 56.0f} },
    { {63, 255, 511, 511},           {255, 255, 511, 511} }
};
//-----------------------------------------------------------------------------
/*void calc_geo_smt(struct ALLDATA* data)
{
    enum Right_Left {LEFT,RIGHT};
    //LEFT или RIGHT пока не выбрали
    for (int Tx = 0; Tx < 4; Tx++)
    {
        data->ATT_dB_geo_signal_smt[Tx] = -20.0f*log10(cabsf(data->R_zx[LEFT].Geo[Tx]));
        data->PH_deg_geo_signal_smt[Tx] = 57.290f*(cargf(data->R_zx[LEFT].Geo[Tx]));
    }
}*/
//-----------------------------------------------------------------------------
//вспомогательна€ функци€ сжати€ ”Ё— в восьмибитное слово
uint16_t ro_to_bits_code(float ro)
{
    uint16_t result = 0;
    float percent = 2.0f;
    float percent_add = 0.0000593f;
    float Ro_0 = 0.3f;

    if (ro < Ro_0)
        return 0;

    for (int n = 0; n < 255; n++)
    {
        float ro_min = Ro_0 * pow(((1.0f + percent / 100.0f) + percent_add * n), n);
        float ro_max = Ro_0 * pow(((1.0f + percent / 100.0f) + percent_add * (n + 1)), n + 1);

        if (ro >= ro_min && ro <= ro_max)
        {
            if (ro <= (ro_min + ro_max) / 2.0f)
                result = n;
            else if (ro > (ro_min + ro_max) / 2.0f)
                result = n + 1;
        }
        if (n == 254 && ro >= ro_max)
        {
            result = n + 1;
        }
    }
    return result;
}
//-----------------------------------------------------------------------------
/*void set_mean_bits(struct REAP_CONST *reap, struct TO_PACK *pack)
{
    //заполн€ем количество значащих бит
    for (int n = 0; n < 4; n++)
        pack->G[n].bits = log2(reap->ATT_grad[_400_kGz][n] + 1);
    for (int n = 4; n < 8; n++)
        pack->G[n].bits = log2(reap->PH_grad[_400_kGz][n - 4] + 1);
    for (int n = 8; n < 12; n++)
        pack->G[n].bits = 8;

    for (int n = 12; n < 16; n++)
        pack->G[n].bits = log2(reap->ATT_grad[_2000_kGz][n-12] + 1);
    for (int n = 16; n < 20; n++)
        pack->G[n].bits = log2(reap->PH_grad[_2000_kGz][n -16] + 1);
    for (int n = 20; n < 24; n++)
        pack->G[n].bits = 8;
}*/
//-----------------------------------------------------------------------------
//перевод из структуры данных картографа в сжатую форму
void compress(struct REAP_CONST *reap, struct ALLDATA *data, struct COMPRESSED_1freq *c_data)
{
    enum Right_Left { LEFT, RIGHT };
    c_data->frame = (uint32_t)(data->frame); //float???
    uint16_t freq;
    if(data->dds_freq >= 1000)
        freq = 1;
    else
        freq = 0;

    c_data->dds_freq = freq;

    for (int Tx = 0; Tx < 4; Tx++)
    {
        float ATT = data->ATT_dB_geo_signal_smt[Tx];
        float PH = data->PH_deg_geo_signal_smt[Tx];
        // reap
        c_data->GA[Tx].data = round((ATT + reap->ATT_max[freq][Tx])*(reap->ATT_grad[freq][Tx] / (2.0f*reap->ATT_max[freq][Tx])));
        c_data->GP[Tx].data = round((PH + reap->PH_max[freq][Tx])*(reap->PH_grad[freq][Tx] / (2.0f*reap->PH_max[freq][Tx])));
        c_data->Ro[Tx].data = ro_to_bits_code((float)data->rho_ph_smt[Tx]);
        //заполн€ем количество значащих бит
        c_data->GA[Tx].bits = log2(reap->ATT_grad[freq][Tx] + 1);
        c_data->GP[Tx].bits = log2(reap->PH_grad[freq][Tx] + 1);
        c_data->Ro[Tx].bits = 8;
    }
}
//-----------------------------------------------------------------------------
void OutCompressedData(struct ALLDATA *data, struct METROLOGY_CARTOGRAPH *metro)
{
    memset((uint16_t*)&data[0].out_arr[0], 0x00, 16); //32
    memset((uint16_t*)&data[1].out_arr[0], 0x00, 16); //32
    struct TO_PACK to_pack;
    struct COMPRESSED_1freq c_data[2];
    struct PACKED packed;
    uint16_t bit_cntr = 0;
    uint16_t byte_cntr = 0;
    uint16_t bit = 0;

    bool work_400 = false;
    bool work_2000 = false;
    if((metro->select_key >> WORK_400) & 1)
    {
        compress(&metro->reap, &data[_400_kGz], &c_data[_400_kGz]);
        memcpy(&to_pack, &c_data[_400_kGz].GA[0], sizeof(struct COMPRESSED_1freq) - 4); //8
        work_400 = true;
    }
    if((metro->select_key >> WORK_2000) & 1)
    {
        compress(&metro->reap, &data[_2000_kGz], &c_data[_2000_kGz]);
        memcpy(&to_pack.G[12], &c_data[_2000_kGz].GA[0], sizeof(struct COMPRESSED_1freq) - 4); //8
        work_2000 = true;
    }

    uint16_t n_0 = 0;
    uint16_t n_end = 0;
    uint16_t write_arr_freq_idx = 0;
    uint32_t key = metro->select_key;

    if(work_400 == true && work_2000 == true)//работают обе частоты
    {
        n_0 = 0;
        n_end = 24;
        write_arr_freq_idx = _2000_kGz;
    }
    else if(work_400 == true && work_2000 == false)//работает только 400
    {
        n_0 = 0;
        n_end = 12;
        write_arr_freq_idx = _400_kGz;
        //key = key & 0x03000FFF; //отключаем кривые дл€ 2000
    }
    else if(work_400 == false && work_2000 == true)//работает только 2000
    {
        n_0 = 12;
        n_end = 24;
        write_arr_freq_idx = _2000_kGz;
        //key = key & 0x03FFF000; //отключаем кривые дл€ 400
    }
    else
    {
        n_0 = 0;
        n_end = 0;
        write_arr_freq_idx = 0;
    }

    data->all_bit_cntr = 0;
    for(int n = n_0; n < n_end; n++)
    {
        if((key >> n) & 1)
        {
            //struct PACKED packed = to_pack.G[n];
            memcpy(&packed, &to_pack.G[n], sizeof(struct PACKED));

            for(uint16_t bit_idx = 0; bit_idx < packed.bits; bit_idx++)
            {
                bit = (packed.data >> bit_idx) & 1;
                if(bit == 1)
                {
                    data[write_arr_freq_idx].out_arr[byte_cntr] |= (1 << bit_cntr);
                }
                data[write_arr_freq_idx].all_bit_cntr++;
                bit_cntr++;
                if(bit_cntr == 16)
                {
                    bit_cntr = 0;
                    byte_cntr++;
                }
            }
        }
    }
}
//-----------------------------------------------------------------------------
void transform_data(struct ALLDATA* data, struct GP_DATA* gp_data)
{
    gp_data->signature = data[0].signature;
    //в data[i].R_zz.condition дл€ каждой частоты:
    //00000000 00000000 00000000 00004321
    //в gp_data->condition:
    //                  400  kGz  2000 kGz
    //00000000 00000000 00054321 00054321
    // »звлекаем младшие байты (8 бит) из каждого condition
    uint32_t cond_2000 = data[1].R_zz.condition & 0xFF; // ћладший байт
    uint32_t cond_400 = data[0].R_zz.condition & 0xFF; // ¬торой байт
    // —обираем  в один uint32_t
    gp_data->condition = cond_2000 | (cond_400 << 8);
    gp_data->frame = data[0].frame;
    gp_data->temperature = data[0].R_zz.temperature;
    int N_Tx = 4;
    for (int freq = 0; freq < 2; freq++)
    {
        gp_data->ZERO_AM_RX_1[freq] = cabsf(data[freq].R_zz.Tx_0[0]);
        gp_data->ZERO_AM_RX_2[freq] = cabsf(data[freq].R_zz.Tx_0[1]);
        for (int Tx = 0; Tx < N_Tx; Tx++)
        {
            //изменено название с rho_smt на rho_ph_smt
            gp_data->phase_smt[freq][Tx] = data[freq].phase_smt[Tx];
            gp_data->rho_ph_smt[freq][Tx] = data[freq].rho_ph_smt[Tx];
            //добавлено дл€ амплитудных измерений ”Ё—////////////////////////
            gp_data->att_smt[freq][Tx] = data[freq].att_smt[Tx];
            gp_data->rho_att_smt[freq][Tx] = data[freq].rho_att_smt[Tx];
            /////////////////////////////////////////////////////////////////
            gp_data->AM_RX_1[freq][Tx] = cabsf(data[freq].R_zz.Rzz1[Tx]);
            gp_data->AM_RX_2[freq][Tx] = cabsf(data[freq].R_zz.Rzz2[Tx]);

            float signal = cargf(data[freq].R_zz.Rzz2[Tx]) - cargf(data[freq].R_zz.Rzz1[Tx]);
                    if(fabs(signal) < 40)
                    {
                        while (signal >= PI) signal -= 2 * PI;
                        while (signal <= -PI) signal += 2 * PI;
                    }
                    else { signal = 0.0; }
                    gp_data->DELTA_PH[freq][Tx] = signal;
          }
     }
}

