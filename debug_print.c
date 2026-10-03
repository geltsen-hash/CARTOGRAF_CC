/*
 * debug_print.c
 *
 *  Created on: 19 ����. 2024 �.
 *      Author: user
 */

#include "driverlib.h"
#include "_globals.h"
#include "device.h"
#include "board.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>


extern char str[128];
extern uartPrintMode uPm;
extern uint16_t aps_idx;
extern apsMeasState apsMMode;
extern uint16_t aps_idx;

extern struct TUartCMD UartCmd;
extern float slo_modul_g, slo_modul_m, slo_modul_w, rot;
extern int N, K;
extern float aps_point, G_M_angle, MA_delay;

extern boolean bAPS2MTF_C; //������ � CAN MTF_C ������ AZM
extern boolean continue_aps;

extern float aps_arr[17];
extern float aps_m_arr[17];

extern struct Vec GBuff, MBuff, WBuff;
//-----------------------------------------------------------------------------
void ParseDebugCmd(void)
{
    if(UartCmd.ready == true)
    {
        //SendString(UartCmd.buf);
        if(UartCmd.len == 3) // 1-��������� ������ ������� �������: [char][\r][\n]
        {
              if(UartCmd.buf[0] == 'D')
              {
                  uPm = PRINT_GMW;
              }
              else if(UartCmd.buf[0] == 'A')
              {
                  uPm = PRINT_APS;
                  aps_idx = 0;
                  apsMMode = APS_IDLE;/////
              }
              else if(UartCmd.buf[0] == 'R')
              {
                  delay_ms(100);
                  uPm = PRINT_RAW;
              }
              else if(UartCmd.buf[0] == 'U')
              {
                  uPm = PRINT_ANG;
              }
              else if(UartCmd.buf[0] == 'S')
              {
                 continue_aps = false;
                 uPm = STOP;
              }
        }
        else if(UartCmd.len == 5)// RST - �����
        {
            if(UartCmd.buf[0] == 'R' && UartCmd.buf[1] == 'S' && UartCmd.buf[2] == 'T')
            {
                bAPS2MTF_C = false;
                aps_idx = 0;
                apsMMode = APS_COMPLETE;
            }
        }
        else if(UartCmd.len > 3)   //AZMMTF DBGPE CALG CALM CALW SET
        {
            if(UartCmd.buf[0] == 'A' && UartCmd.buf[1] == 'Z' && UartCmd.buf[2] == 'M' && UartCmd.buf[3] == 'M' && UartCmd.buf[4] == 'T' && UartCmd.buf[4] == 'F')
            {
                bAPS2MTF_C = true;
            }
            else if(UartCmd.buf[0] == 'R' && UartCmd.buf[1] == 'O' && UartCmd.buf[2] == 'T' && UartCmd.buf[3] == 'A' && UartCmd.buf[4] == 'T'&& UartCmd.buf[5] == 'E') //
            {
                if(1 == sscanf(UartCmd.buf, "ROTATE %d", &gEmulRotateSpeed))
                {
                   if(gEmulRotateSpeed > 5000)
                       gEmulRotateSpeed = 5000;
                }
                sprintf(str, "ROTATE:%d\r\n", gEmulRotateSpeed);
                SendString(str);
            }
            else if(UartCmd.buf[0] == 'C' && UartCmd.buf[1] == 'A' && UartCmd.buf[2] == 'L' && UartCmd.buf[3] == 'G') //CALG
            {
               if(12 == sscanf(UartCmd.buf, "CALG%f %f %f %f %f %f %f %f %f %f %f %f", &(G_offset_sens.offset.X), &(G_offset_sens.offset.Y),&(G_offset_sens.offset.Z),
                               &(G_offset_sens.sens.XX),&(G_offset_sens.sens.YX),&(G_offset_sens.sens.ZX),
                               &(G_offset_sens.sens.XY),&(G_offset_sens.sens.YY),&(G_offset_sens.sens.ZY),
                               &(G_offset_sens.sens.XZ),&(G_offset_sens.sens.YZ),&(G_offset_sens.sens.ZZ)))
                {

                   //
                   FLASH_WriteCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
                   sprintf(str, "GOK\r\n"); SendString(str); delay_us(50000);
                }
            }

            else if(UartCmd.buf[0] == 'C' && UartCmd.buf[1] == 'A' && UartCmd.buf[2] == 'L' && UartCmd.buf[3] == 'M') //CALM
            {
               if(12 == sscanf(UartCmd.buf, "CALM%f %f %f %f %f %f %f %f %f %f %f %f", &(M_offset_sens.offset.X), &(M_offset_sens.offset.Y),&(M_offset_sens.offset.Z),
                               &(M_offset_sens.sens.XX),&(M_offset_sens.sens.YX),&(M_offset_sens.sens.ZX),
                               &(M_offset_sens.sens.XY),&(M_offset_sens.sens.YY),&(M_offset_sens.sens.ZY),
                               &(M_offset_sens.sens.XZ),&(M_offset_sens.sens.YZ),&(M_offset_sens.sens.ZZ)))
               {
                   FLASH_WriteCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
                   sprintf(str, "MOK\r\n"); SendString(str); delay_us(50000);

               }
            }
            else if(UartCmd.buf[0] == 'C' && UartCmd.buf[1] == 'A' && UartCmd.buf[2] == 'L' && UartCmd.buf[3] == 'W') //CALW
            {
               if(12 == sscanf(UartCmd.buf, "CALW%f %f %f %f %f %f %f %f %f %f %f %f", &(W_offset_sens.offset.X), &(W_offset_sens.offset.Y),&(W_offset_sens.offset.Z),
                               &(W_offset_sens.sens.XX),&(W_offset_sens.sens.YX),&(W_offset_sens.sens.ZX),
                               &(W_offset_sens.sens.XY),&(W_offset_sens.sens.YY),&(W_offset_sens.sens.ZY),
                               &(W_offset_sens.sens.XZ),&(W_offset_sens.sens.YZ),&(W_offset_sens.sens.ZZ)))
               {
                   FLASH_WriteCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
                   sprintf(str, "WOK\r\n"); SendString(str); delay_us(50000);
               }
            }
            else if(UartCmd.buf[0] == 'S' && UartCmd.buf[1] == 'E' && UartCmd.buf[2] == 'T') //SET
            {
               if(12 == sscanf(UartCmd.buf, "SET%ld %ld %ld %ld %ld %ld %f %f %f %f %f %f",
                               &(Settings->MA_WINDOW_SIZE), &(Settings->MLD_WINDOW_SIZE),&(Settings->N),
                               &(Settings->K),&(Settings->auto_delta),&(Settings->INTERRUPT_BY_ANGLE_PATH),
                               &(Settings->W_MIN),&(Settings->W_MAX),&(Settings->K_predict),
                               &(Settings->APS_DELTA),&(Settings->history_angle),&(Settings->K_delta)))
               {
                   FLASH_WriteMetrology(&Metro);
                   sprintf(str, "SOK\r\n"); SendString(str); delay_us(50000);
               }
            }
            else if(UartCmd.buf[0] == 'G' && UartCmd.buf[1] == 'E' && UartCmd.buf[2] == 'T' && UartCmd.buf[3] == 'G') //SET
            {
                sprintf(str, "G:%.0f %.0f %.0f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\r\n",
                              G_offset_sens.offset.X, G_offset_sens.offset.Y, G_offset_sens.offset.Z,
                              G_offset_sens.sens.XX, G_offset_sens.sens.YX, G_offset_sens.sens.ZX,
                              G_offset_sens.sens.XY, G_offset_sens.sens.YY, G_offset_sens.sens.ZY,
                              G_offset_sens.sens.XZ, G_offset_sens.sens.YZ, G_offset_sens.sens.ZZ
                       );
                SendString(str);  delay_us(50000);
            }
            else if(UartCmd.buf[0] == 'G' && UartCmd.buf[1] == 'E' && UartCmd.buf[2] == 'T' && UartCmd.buf[3] == 'M') //SET
            {
                sprintf(str, "M:%.0f %.0f %.0f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\r\n",
                              M_offset_sens.offset.X, M_offset_sens.offset.Y, M_offset_sens.offset.Z,
                              M_offset_sens.sens.XX, M_offset_sens.sens.YX, M_offset_sens.sens.ZX,
                              M_offset_sens.sens.XY, M_offset_sens.sens.YY, M_offset_sens.sens.ZY,
                              M_offset_sens.sens.XZ, M_offset_sens.sens.YZ, M_offset_sens.sens.ZZ
                        );
                SendString(str);  delay_us(50000);
            }

            else if(UartCmd.buf[0] == 'G' && UartCmd.buf[1] == 'E' && UartCmd.buf[2] == 'T' && UartCmd.buf[3] == 'W') //SET
            {
                sprintf(str, "W:%.0f %.0f %.0f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\r\n",
                              W_offset_sens.offset.X, W_offset_sens.offset.Y, W_offset_sens.offset.Z,
                              W_offset_sens.sens.XX, W_offset_sens.sens.YX, W_offset_sens.sens.ZX,
                              W_offset_sens.sens.XY, W_offset_sens.sens.YY, W_offset_sens.sens.ZY,
                              W_offset_sens.sens.XZ, W_offset_sens.sens.YZ, W_offset_sens.sens.ZZ
                       );
                SendString(str);  delay_us(50000);
            }
            else if(UartCmd.buf[0] == 'G' && UartCmd.buf[1] == 'E' && UartCmd.buf[2] == 'T' && UartCmd.buf[3] == 'S') //SET
            {
                sprintf(str, "SET:%ld %ld %ld %ld %ld %ld %.4f %.4f %.4f %.4f %.4f %.4f\r\n",
                        Settings->MA_WINDOW_SIZE, Settings->MLD_WINDOW_SIZE, Settings->N,
                        Settings->K, Settings->auto_delta, Settings->INTERRUPT_BY_ANGLE_PATH,
                        Settings->W_MIN, Settings->W_MAX, Settings->K_predict,
                        Settings->APS_DELTA, Settings->history_angle, Settings->K_delta
                       );
                SendString(str);  delay_us(50000);
           }

       }//else if(UartCmd.len > 3)

        //GPIO_WritePin(LED_2, 0);
        //GPIO_WritePin(LED_1, 0);
        memset(UartCmd.buf, 0x00, sizeof(UartCmd.buf));
        UartCmd.len = 0;
        UartCmd.ready = false;
    }//if(UartCmd.ready == true)
}
//-----------------------------------------------------------------------------
bool bAPSPrinted = false;
void PrintDebugInfo(void)
{
//������ � UART

    if(uPm == PRINT_GMW)
    {
        sprintf(str, "D:%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\r\n", GBuff.X, GBuff.Y, GBuff.Z, MBuff.X, MBuff.Y, MBuff.Z, WBuff.X, WBuff.Y, WBuff.Z);//rad/c
        SendString(str);
    }
    else if(uPm == PRINT_APS)
    {
        sprintf(str, "A:%.5f %.5f %.7f %.5f\r\n", MTF, Wg, MA_delay, aps_point);
        SendString(str);
       // DELAY_US(20000);

        if(apsMMode == APS_COMPLETE && !bAPSPrinted)
        {
            bAPSPrinted = true;
            for (int i = 0; i < 16; i++)
            {
                sprintf(str,"N:%d %.6f %.6f\r\n", i+1, aps_arr[i], aps_m_arr[i]);
                SendString(str);
            }
            sprintf(str,"N:%d %.6f %.6f\r\n", 17, aps_arr[16], aps_m_arr[16]);
            SendString(str);
           // double nnn = Now;
            //sprintf(str,"ERR: %d %d %X %X %.3f \r\n",  0, 0, error_msg, error_sector_msg, nnn/1000 );
           //SendString(str);
            /////////////////////////////////
            for (int i = 0; i < 17; i++)
            {
                 aps_arr[i] = 0;
                 aps_m_arr[i] = 0;
            }
            /////////////////////////////////
        }
    }
    else if(uPm == PRINT_RAW)
    {
        sprintf(str, "R:%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\r\n", G_raw.X, G_raw.Y, G_raw.Z, M_raw.X, M_raw.Y, M_raw.Z, W_raw.X, W_raw.Y, W_raw.Z);
        SendString(str);
    }
    else if(uPm == PRINT_ANG)
    {
         sprintf(str, "U:%.1f %.1f %.1f %.1f %.4f %d %.4f %.4f %.1f\r\n", angle_aps_deg, MTF*57.296, angle_zen_deg, angle_azm_deg,
                                                                          slo_modul_w*57.296, K, Wg, omega*1000.0, (MTF - prediction)*57.296);//rad/c
         SendString(str);
    }
    else if(uPm == STOP)
    {
        return;
    }
    else
    {
        sprintf(str, "ERROR: uPm unknown! \r\n");
        SendString(str);
        uPm = STOP;
    }
}

