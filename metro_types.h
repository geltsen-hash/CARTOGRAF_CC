/*
 * metro_types.h
 *
 *  Created for: kg_controller
 *  Description: Metrology structures, telemetry frames and geophysical data packets
 */

#ifndef METRO_TYPES_H_
#define METRO_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <complex.h>
#include "sensor_types.h"
#include "cyclogram_types.h"

enum
{
    TYPE_CARTOGRAPH = 0,
    TYPE_LWD_4TX = 1
};

enum
{
    PROFILE_LWD_4TX = 349,
    PROFILE_CARTOGRAPH = 352,
    PROFILE_CARTOGRAPH_RAW = 351
};

enum T_
{
    T1, T2, T3, T4, T5
};

enum FREQ
{
    _400_kGz,
    _2000_kGz
};

enum Air_type
{
    PH,
    ATT
};

struct direct_RX
{
    float complex Tx_0;
    float complex Geo[4];
    float amp_Vzz[4];
    float amp_Vzx[4];
    float ph_Vzx_Vzz[4];
    float dv[4];
    float border_angle[4];
    float temperature;
    uint32_t condition;
};

struct undirect_RX
{
    float complex Tx_0[2];
    float complex Rzz1[4];
    float complex Rzz2[4];
    float temperature;
    uint32_t condition;
};

struct PACKED
{
    uint16_t data;
    uint16_t bits;
};

struct COMPRESSED_1freq
{
    uint32_t frame;
    uint32_t dds_freq;
    struct PACKED GA[4];
    struct PACKED GP[4];
    struct PACKED Ro[4];
};

struct TO_PACK
{
    struct PACKED G[24];
};

struct GP_DATA
{
    uint32_t signature;
    uint32_t condition;
    uint32_t frame;
    float temperature;
    float rho_ph_smt[2][5];
    float phase_smt[2][5];
    float AM_RX_1[2][5];
    float ZERO_AM_RX_1[2];
    float AM_RX_2[2][5];
    float ZERO_AM_RX_2[2];
    float DELTA_PH[2][5];
    float ZERO_dPH[2];
    float rho_att_smt[2][5];
    float att_smt[2][5];
};

struct ALLDATA
{
    uint32_t signature;
    uint32_t frame;
    uint32_t dds_freq;
    float ATT_dB_geo_signal_smt[4];
    float PH_deg_geo_signal_smt[4];
    float rho_ph_smt[4];
    float phase_smt[4];
    uint16_t out_arr[16];
    uint32_t all_bit_cntr;
    uint32_t depth;
    struct direct_RX R_zx[2];
    struct undirect_RX R_zz;
    struct Inclinometer INC;
    uint16_t start_sector;
    uint16_t DataValid;
    float Wg;
    float rho_att_smt[4];
    float att_smt[4];
};

struct _16_sector_data
{
    float Re_sector[16];
    float Im_sector[16];
};

struct raw_sector_data_all
{
    struct _16_sector_data Rx_L[4];
    struct _16_sector_data Rx_R[4];
};

struct REAP_CONST
{
    float ATT_max[2][4];
    uint32_t ATT_grad[2][4];
    float PH_max[2][4];
    uint32_t PH_grad[2][4];
};

struct GEO_CAL
{
    float Beta_Z[2][4];
    float V_zx_colar_add[2][4];
    float Vxz_Vzz_air_d_ph[2][4];
};

struct METROLOGY_GP
{
    uint32_t signature;
    uint32_t serial;
    uint16_t L1[5];
    uint16_t L2[5];
    uint16_t F[2];
    int16_t air_ph[2][5];
    int16_t min_amp[2][5];
    uint32_t D_sonde_mm;
    uint32_t work_type;
    uint32_t Rx_Position;
    float air_att_dB[2][5];
    uint16_t service[58];
};

struct METROLOGY_CARTOGRAPH
{
    uint32_t signature;
    uint32_t serial;
    uint16_t L1[5];
    uint16_t L2[5];
    uint16_t F[2];
    int16_t air_ph[2][5];
    int16_t min_amp[2][5];
    uint32_t D_sonde_mm;
    uint32_t work_type;
    uint32_t Rx_Position;
    float air_att_dB[2][5];
    uint16_t service[58];
    uint32_t L_geo[5];
    struct INC_SET inc_set;
    struct TURN_PRESETS turn_presets;
    uint64_t unix_epoh_data;
    struct REAP_CONST reap;
    uint32_t select_key;
    float betta_Z_deg[2][2][4];
    float V_zx_colar_add[2][2][4];
    float d_ph_Vzx_Vzz_mG[2][2][4];
    uint16_t service1[242];
};

struct ID
{
    uint32_t struct_size;
    uint32_t type_;
    uint32_t N_Tx;
    uint32_t mod;
    uint32_t number;
    uint32_t type;
};

#endif /* METRO_TYPES_H_ */
