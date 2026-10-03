/*
 * inc_flash.c
 *
 *  Created on: 20 июн. 2018 г.
 *      Author: 1
 */

//-------------------------------------------------------------------------------------------------------
// ќ“ Ћё„»“№ —“»–јЌ»≈ —≈ “ќ–ќ¬ W/X/Y ¬ свойствах проекта Properties->Debug->Flash settings->Erase settings
//-------------------------------------------------------------------------------------------------------

#include "_globals.h"
#include <string.h>
#include "flash_programming_c28.h"      // Flash API example header file
#include "F021_F2837xS_C28x.h"

#define CPUCLK_FREQUENCY 100
#define  WORDS_IN_FLASH_BUFFER    1024  // Programming data buffer, words

#define PUMPREQUEST *(unsigned long*)(0x00050024)

#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #define ramFuncSection ".TI.ramfunc"
    #else
        #define ramFuncSection "ramfuncs"
    #endif
#endif

//#pragma DATA_SECTION(Buffer,"BufferDataSection");
uint16   Buffer[WORDS_IN_FLASH_BUFFER + 1];
uint32   *Buffer32 = (uint32 *)Buffer;
bool bFlashError = false;

//------------------------------------------------------------------------------------

#pragma CODE_SECTION(FLASH_ReadSet, ramFuncSection);
void FLASH_ReadSet(struct INC_SET *pSet) //чтение
{
    volatile int size = sizeof(struct INC_SET);
    memcpy(Settings, (uint16_t*)BOne_SectorX_start, size);
}
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(FLASH_ReadCal, ramFuncSection);
void FLASH_ReadCal(struct Cal *pG, struct Cal *pM, struct Cal *pW) //чтение
{
    volatile int size = sizeof(struct Cal);
    memcpy(pG, (uint16_t*)BOne_SectorW_start, size);
    memcpy(pM, (uint16_t*)BOne_SectorW_start + size, size);
    memcpy(pW, (uint16_t*)BOne_SectorW_start + 2*size, size);
}
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(FLASH_ReadMetrology, ramFuncSection);
void FLASH_ReadMetrology(struct METROLOGY_CARTOGRAPH *pMetro)
{
    volatile int size = sizeof(struct METROLOGY_CARTOGRAPH);
    memcpy(pMetro, (uint16_t*)BOne_SectorX_start, size);
    gWorkType = GetWorkType(Metro.signature);
}
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(FLASH_WriteCal, ramFuncSection);
void FLASH_WriteCal(struct Cal *pG, struct Cal *pM, struct Cal *pW) //стирает сектор W и записывает данные калибровки
{
  volatile uint32 u32Index = 0;
  volatile Fapi_StatusType oReturnCheck;
  volatile Fapi_FlashStatusType oFlashStatus;
  Fapi_FlashStatusWordType oFlashStatusWord;

  CPUTimer_disableInterrupt(TIMER0_1MS_BASE);
  DINT;
  controller_state = STATE_IDLE;

  volatile unsigned int size = sizeof(struct Cal);
  for(int i=0;i<(WORDS_IN_FLASH_BUFFER+1);i++)  // 0xFFFF
  {Buffer[i] = 0xFFFF;}
     //3 структуры последовательно одна за другой
     memcpy(Buffer, pG, size);
     memcpy(Buffer + size, pM, size);
     memcpy(Buffer + 2*size, pW, size);

     EALLOW;

     PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1
     oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, CPUCLK_FREQUENCY); //100 ћ√ц
     if(oReturnCheck != Fapi_Status_Success)
     {
         Example_Error(oReturnCheck);
         //return;
     }
     oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1
         //стираем сектор W                                                             //0xE8000
         oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorW_start); //банк1 сектор W
         while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation

         oReturnCheck = Fapi_doBlankCheck((uint32 *)BOne_SectorW_start,// Verify that SectorW is erased.  The Erase step itself does a
                                          Bzero_16KSector_u32length,    // verify as it goes.  This verify is a 2nd verification that can be done.
                                          &oFlashStatusWord);
         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
             //return;
         }

     u32Index = BOne_SectorW_start; //pizdec
     for(uint16_t i=0;(u32Index < (BOne_SectorW_start + WORDS_IN_FLASH_BUFFER)) && (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index += 8)
     {
         oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,8,0,0,Fapi_AutoEccGeneration);
         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
             //return;
         }
         while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
         oFlashStatus = Fapi_getFsmStatus(); //for any debug
          oReturnCheck = Fapi_doVerify((uint32 *)u32Index,4,Buffer32+(i/2), &oFlashStatusWord);// Verify the values programmed.
          if(oReturnCheck != Fapi_Status_Success)
          {
              Example_Error(oReturnCheck);
              //return;
          }

     }
      //Flash1EccRegs.ECC_ENABLE.bit.ENABLE = 0xA; // Enable ECC
      PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0
      EDIS;

      controller_state = STATE_NORMAL;
      EINT;
  CPUTimer_enableInterrupt(TIMER0_1MS_BASE);
}
//------------------------------------------------------------------------------------
/*#pragma CODE_SECTION(FLASH_WriteSet, ramFuncSection);
void FLASH_WriteSet(struct Set *pSet) //стирает сектор X и записывает данные калибровки
{
  volatile uint32 u32Index = 0;
  volatile Fapi_StatusType oReturnCheck;
  volatile Fapi_FlashStatusType oFlashStatus;
  Fapi_FlashStatusWordType oFlashStatusWord;

  volatile unsigned int size = sizeof(struct Cal);
  for(int i=0;i<(WORDS_IN_FLASH_BUFFER+1);i++)  // 0xFFFF
  {Buffer[i] = 0xFFFF;}

  memcpy(Buffer, pSet, size);

  EALLOW;

     PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1
     oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, CPUCLK_FREQUENCY); //100 ћ√ц
     if(oReturnCheck != Fapi_Status_Success)
     {
         Example_Error(oReturnCheck);
         return;
     }
     oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1
         //стираем сектор X                                                             //0xF0000
         oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorX_start); //банк1 сектор X
         while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation

         oReturnCheck = Fapi_doBlankCheck((uint32 *)BOne_SectorX_start,// Verify that SectorX is erased.  The Erase step itself does a
                                          Bzero_16KSector_u32length,    // verify as it goes.  This verify is a 2nd verification that can be done.
                                          &oFlashStatusWord);
         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
             return;
         }

     u32Index = BOne_SectorX_start; //pizdec
     for(uint16_t i=0;(u32Index < (BOne_SectorX_start + WORDS_IN_FLASH_BUFFER)) && (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index += 8)
     {
         oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,8,0,0,Fapi_AutoEccGeneration);
         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
             return;
         }
         while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
         oFlashStatus = Fapi_getFsmStatus(); //for any debug
          oReturnCheck = Fapi_doVerify((uint32 *)u32Index,4,Buffer32+(i/2), &oFlashStatusWord);// Verify the values programmed.
          if(oReturnCheck != Fapi_Status_Success)
          {
              Example_Error(oReturnCheck);
              return;
          }

     }
      //Flash1EccRegs.ECC_ENABLE.bit.ENABLE = 0xA; // Enable ECC
      PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0
   EDIS;
}*/
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(FLASH_ReadSetOfPresets, ramFuncSection);
void FLASH_ReadSetOfPresets(struct SET_OF_PRESETS *pSetOfPresets)
{
    uint16_t *ptr  = (uint16_t*)BOne_SectorY_start;
    volatile int size = sizeof(struct SET_OF_PRESETS);
    volatile uint32_t maxindex = (BOne_SectorY_End - BOne_SectorY_start) / size;
    for(uint32_t i=0; i<maxindex*size; i+=size)
    {
        if(ptr[i]!=0x5AA5 && i>=size)
        {
            memcpy(pSetOfPresets, &(ptr[i-size]), size);
            break;
        }
    }
    gEmulRotateSpeed = (SetOfPresets.DebugSession >> 8)&0xFF;
}
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(FLASH_WriteSetOfPresets, ramFuncSection);
void FLASH_WriteSetOfPresets(struct SET_OF_PRESETS *pSetOfPresets) //в сектор Y
{
  volatile uint32 u32Index = 0;
  volatile Fapi_StatusType oReturnCheck;
  volatile Fapi_FlashStatusType oFlashStatus;
  //Fapi_FlashStatusWordType oFlashStatusWord;
  uint16_t *ptr = (uint16_t*)BOne_SectorY_start;

  volatile unsigned int size = sizeof(struct SET_OF_PRESETS);

  CPUTimer_disableInterrupt(TIMER0_1MS_BASE);
  DINT;
  controller_state = STATE_IDLE;

  for(int i=0;i<(WORDS_IN_FLASH_BUFFER+1);i++)  // 0xFFFF
  {Buffer[i] = 0xFFFF;}

  memcpy(Buffer, pSetOfPresets, size);

  EALLOW;

     PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1
     oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, CPUCLK_FREQUENCY); //100 ћ√ц
     if(oReturnCheck != Fapi_Status_Success)
     {
         Example_Error(oReturnCheck);
         //return;
     }
     oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1

     //uint16_t maxindex = Bzero_16KSector_u32length/size;
     volatile uint32_t maxindex = (BOne_SectorY_End - BOne_SectorY_start) / size; //сколько структур влезет в страницу

     for(uint32_t i=0; i<maxindex*size;i+=size) //ищем куда будем писать
     {
         if(i>=((maxindex-1)*size)) //дошли до конца страницы, стираем все и пишем сначала
         {
            //стираем сектор Y
            ptr = (uint16_t*)BOne_SectorY_start; //пишем с начала сектора стерев его
            oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorY_start); //банк1 сектор Y
            while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation
            break;
         }

         if(ptr[i] != 0x5AA5)//зан€т ли данный индекс
         {
             if(ptr[i]==0xFFFF && ptr[i+1]==0xFFFF && ptr[i+2]==0xFFFF && ptr[i+3]==0xFFFF &&
                     ptr[i+4]==0xFFFF && ptr[i+5]==0xFFFF && ptr[i+6]==0xFFFF && ptr[i+7]==0xFFFF) //убеждаемс€ что место под 8 слов не зан€то
             {
                 ptr = ptr+i;
             }
             else
             {
                 //стираем сектор Y
                 ptr = (uint16_t*)BOne_SectorY_start; //пишем с начала сектора стерев его
                 oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorY_start); //банк1 сектор Y
                 while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation
             }
             break;
         }
     }

     u32Index = (uint32_t)ptr;
     //u32Index = BOne_SectorY_start;
     for(uint16_t i=0;(u32Index < ((uint32_t)ptr + size)) && (oReturnCheck == Fapi_Status_Success); i+=8, u32Index+=8)
     {
         oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,8,0,0,Fapi_AutoEccGeneration);

         while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
         oFlashStatus = Fapi_getFsmStatus(); //for any debug

     }
      //Flash1EccRegs.ECC_ENABLE.bit.ENABLE = 0xA; // Enable ECC
      PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0
   EDIS;

   controller_state = STATE_NORMAL;
   EINT;
  CPUTimer_enableInterrupt(TIMER0_1MS_BASE);
}
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(FLASH_WriteMetrology, ramFuncSection);
void FLASH_WriteMetrology(struct METROLOGY_CARTOGRAPH *pMetro) //в сектор X
{
  volatile uint32 u32Index = 0;
  volatile Fapi_StatusType oReturnCheck;
  volatile Fapi_FlashStatusType oFlashStatus;
  Fapi_FlashStatusWordType oFlashStatusWord;

  CPUTimer_disableInterrupt(TIMER0_1MS_BASE);
  DINT;
  controller_state = STATE_IDLE;

  volatile unsigned int size = sizeof(struct METROLOGY_CARTOGRAPH);
  for(int i=0;i<(WORDS_IN_FLASH_BUFFER+1);i++)  // 0xFFFF
  {Buffer[i] = 0xFFFF;}

  memcpy(Buffer, pMetro, size);

  EALLOW;

     PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1
     oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, CPUCLK_FREQUENCY); //100 ћ√ц
     if(oReturnCheck != Fapi_Status_Success)
     {
         Example_Error(oReturnCheck);
         //return;
     }
     oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1
         //стираем сектор X                                                             //0xF0000
         oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorX_start); //банк1 сектор X
         while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation

         oReturnCheck = Fapi_doBlankCheck((uint32 *)BOne_SectorX_start,// Verify that SectorX is erased.  The Erase step itself does a
                                          Bzero_16KSector_u32length,    // verify as it goes.  This verify is a 2nd verification that can be done.
                                          &oFlashStatusWord);
         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
             //return;
         }

     u32Index = BOne_SectorX_start; //pizdec
     for(uint16_t i=0;(u32Index < (BOne_SectorX_start + WORDS_IN_FLASH_BUFFER)) && (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index += 8)
     {
         oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,8,0,0,Fapi_AutoEccGeneration);
         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
             //return;
         }
         while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
         oFlashStatus = Fapi_getFsmStatus(); //for any debug
          oReturnCheck = Fapi_doVerify((uint32 *)u32Index,4,Buffer32+(i/2), &oFlashStatusWord);// Verify the values programmed.
          if(oReturnCheck != Fapi_Status_Success)
          {
              Example_Error(oReturnCheck);
             // return;
          }

     }
      //Flash1EccRegs.ECC_ENABLE.bit.ENABLE = 0xA; // Enable ECC
      PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0
   EDIS;

   controller_state = STATE_NORMAL;
   EINT;
  CPUTimer_enableInterrupt(TIMER0_1MS_BASE);
}
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(FLASH_Erase,ramFuncSection);
void FLASH_Erase(uint16_t page) //стирает сектор W/X/Y
{

     volatile Fapi_StatusType oReturnCheck;
     volatile Fapi_FlashStatusType oFlashStatus;
     Fapi_FlashStatusWordType oFlashStatusWord;

    DINT;
    switch (page)
    {
    case PAGE_W:
         EALLOW;

         PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1

         oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, 100); //100 ћ√ц

         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
         }

         oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1

         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
         }
             //стираем сектор W                                                             //0xE8000
             oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorW_start); //банк1 сектор W
             while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation

             oReturnCheck = Fapi_doBlankCheck((uint32 *)BOne_SectorW_start,// Verify that SectorW is erased.  The Erase step itself does a
                                              Bzero_16KSector_u32length,    // verify as it goes.  This verify is a 2nd verification that can be done.
                                              &oFlashStatusWord);

             if(oReturnCheck != Fapi_Status_Success)
             {
                  Example_Error(oReturnCheck);
             }

             PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0

             EDIS;
             break;
    case PAGE_X:
        EALLOW;

        PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1

        oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, 100); //100 ћ√ц

        if(oReturnCheck != Fapi_Status_Success)
        {
            Example_Error(oReturnCheck);
        }

        oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1

        if(oReturnCheck != Fapi_Status_Success)
        {
            Example_Error(oReturnCheck);
        }
            //стираем сектор X                                                             //0xF0000
            oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorX_start); //банк1 сектор X
            while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation

            oReturnCheck = Fapi_doBlankCheck((uint32 *)BOne_SectorX_start,// Verify that SectorW is erased.  The Erase step itself does a
                                             Bzero_16KSector_u32length,    // verify as it goes.  This verify is a 2nd verification that can be done.
                                             &oFlashStatusWord);

            if(oReturnCheck != Fapi_Status_Success)
            {
                 Example_Error(oReturnCheck);
            }

            PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0

            EDIS;
            break;

     case PAGE_Y:
         EALLOW;

         PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1

         oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, 100); //100 ћ√ц

         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
         }

         oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1

         if(oReturnCheck != Fapi_Status_Success)
         {
             Example_Error(oReturnCheck);
         }
             //стираем сектор Y                                                             //0xF8000
             oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorY_start); //банк1 сектор Y
             while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation

             oReturnCheck = Fapi_doBlankCheck((uint32 *)BOne_SectorY_start,// Verify that SectorY is erased.  The Erase step itself does a
                                              BOne_16KSector_u32length,    // verify as it goes.  This verify is a 2nd verification that can be done.
                                             &oFlashStatusWord);

             if(oReturnCheck != Fapi_Status_Success)
             {
                  Example_Error(oReturnCheck);
             }

             PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0

             EDIS;
            break;
     case PAGE_AB:
              EALLOW;

              PUMPREQUEST = 0x5A5A0001; // Give pump ownership to FMC1

              oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, 100); //100 ћ√ц

              if(oReturnCheck != Fapi_Status_Success)
              {
                  Example_Error(oReturnCheck);
              }

              oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1); //банк 1

              if(oReturnCheck != Fapi_Status_Success)
              {
                  Example_Error(oReturnCheck);
              }
                  //стираем сектор AB
                  oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32 *)BOne_SectorAB_start); //банк1 сектор AB
                  while(Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}  // Wait until FSM is done with erase sector operation

                  oReturnCheck = Fapi_doBlankCheck((uint32 *)BOne_SectorAB_start,// Verify that SectorAB is erased.  The Erase step itself does a
                                                   BOne_16KSector_u32length,    // verify as it goes.  This verify is a 2nd verification that can be done.
                                                  &oFlashStatusWord);

                  if(oReturnCheck != Fapi_Status_Success)
                  {
                       Example_Error(oReturnCheck);
                  }

                  PUMPREQUEST = 0x5A5A0000; // Give pump ownership back to FMC0

                  EDIS;
                 break;
     default:
            break;

    }//endswitch
    EINT;

}
//------------------------------------------------------------------------------------
#pragma CODE_SECTION(Example_Error,ramFuncSection);
void Example_Error(Fapi_StatusType status)
{
    bFlashError = true; //ошибка записи
    //__asm("    ESTOP0"); //  Error code will be in the status parameter
}
//------------------------------------------------------------------------------------




