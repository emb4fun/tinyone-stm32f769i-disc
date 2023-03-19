/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2019-2022 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may 
*     be used to endorse or promote products derived from this software 
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
**************************************************************************/
#if defined(USE_BOARD_MB1225)
#define __TALBOARD_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include <time.h>
#include "tal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Values are based on:
 * STM32Cube_FW_F7_V1.15.0\Drivers\BSP\STM32756G_EVAL\stm32756g_eval_sdram.h
 */
#define REFRESH_COUNT_100MHZ  ((uint32_t)0x0603)   /* SDRAM refresh counter (100Mhz SD clock) */

/*
 * For a 96MHz SDCLK the correct value is 0x05C8 which produce a
 * refresh cycle of exact 64ms. But for security 0x05C0 will be used
 * which produce a refresh cycle of 63.6ms.
 */
#define REFRESH_COUNT_96MHZ   ((uint32_t)0x05C0)   /* SDRAM refresh counter */ 

/*******************************************************************/

#define SDRAM_MEMORY_WIDTH    FMC_SDRAM_MEM_BUS_WIDTH_32
#define SDCLOCK_PERIOD        FMC_SDRAM_CLOCK_PERIOD_2

#define SDRAM_TIMEOUT         ((uint32_t)0xFFFF)

#define SDRAM_MODEREG_BURST_LENGTH_1               ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2               ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4               ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8               ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL        ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED       ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2                ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3                ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD      ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED   ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE       ((uint16_t)0x0200)

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

#if defined(MEMORY_INIT)
static SDRAM_HandleTypeDef       sdramHandle;
static FMC_SDRAM_TimingTypeDef   Timing;
static FMC_SDRAM_CommandTypeDef  Command;
#endif

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

#if defined(MEMORY_INIT)
/*************************************************************************/
/*  _HAL_Delay                                                           */
/*                                                                       */
/*  A small delay without systick functionality.                         */
/*                                                                       */
/*  In    : Timeout                                                      */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void _HAL_Delay (uint32_t Timeout)
{
   /* 8 is the number of required instructions cycles for the below loop statement.
   The SDMMC_CMDTIMEOUT is expressed in ms */
   volatile uint32_t count = Timeout * ((SystemCoreClock / 8) / 1000);
  
   do
   {
      if (count-- == 0)
      {
         break;
      }
   } while (1);   /*lint !e506*/
   
} /* _HAL_Delay */

/*************************************************************************/
/*  BSP_SDRAM_Initialization_Sequence                                    */
/*                                                                       */
/*  Perform the SDRAM external memory initialization sequence.           */
/*                                                                       */
/*  In    : hsdram, Command                                              */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void BSP_SDRAM_Initialization_sequence (uint32_t RefreshCount)
{
   /*
    * Code is based on:
    * STM32Cube_FW_F7_V1.15.0\Drivers\BSP\STM32756G_EVAL\stm32756g_eval_sdram.c
    */

   __IO uint32_t tmpmrd = 0;
  
   /* Step 1: Configure a clock configuration enable command */
   Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
   Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
   Command.AutoRefreshNumber      = 1;
   Command.ModeRegisterDefinition = 0;

   /* Send the command */
   HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);

   /* Step 2: Insert 100 us minimum delay */ 
   /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
   _HAL_Delay(1);
    
   /* Step 3: Configure a PALL (precharge all) command */ 
   Command.CommandMode            = FMC_SDRAM_CMD_PALL;
   Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
   Command.AutoRefreshNumber      = 1;
   Command.ModeRegisterDefinition = 0;

   /* Send the command */
   HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);  
  
   /* Step 4: Configure an Auto Refresh command */ 
   Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
   Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
   Command.AutoRefreshNumber      = 2;
   Command.ModeRegisterDefinition = 0;

   /* Send the command */
   HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);
  
   /* Step 5: Program the external memory mode register */
   tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_8          |\
                      SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                      SDRAM_MODEREG_CAS_LATENCY_3           |\
                      SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                      SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  
   Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
   Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
   Command.AutoRefreshNumber      = 1;
   Command.ModeRegisterDefinition = tmpmrd;

   /* Send the command */
   HAL_SDRAM_SendCommand(&sdramHandle, &Command, SDRAM_TIMEOUT);
  
   /* Step 6: Set the refresh rate counter */
   /* Set the device refresh rate */
   HAL_SDRAM_ProgramRefreshRate(&sdramHandle, RefreshCount); 

} /* BSP_SDRAM_Initialization_sequence */

/*************************************************************************/
/*  BoardConfigSDRAM                                                     */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void BoardConfigSDRAM (void)
{
   /*
    * Code is based on:
    * STM32Cube_FW_F7_V1_15_0\Drivers\BSP\STM32F769I-Discovery\stm32f769i_discovery_sdram.c
    */

   /* SDRAM device configuration */
   sdramHandle.Instance = FMC_SDRAM_DEVICE;
    
   /* Timing configuration for 96Mhz as SDRAM clock frequency (System clock is up to 192Mhz) */
   Timing.LoadToActiveDelay    = 2;
   Timing.ExitSelfRefreshDelay = 7;
   Timing.SelfRefreshTime      = 5;
   Timing.RowCycleDelay        = 6;
   Timing.WriteRecoveryTime    = 3;
   Timing.RPDelay              = 2;
   Timing.RCDDelay             = 2;
  
   sdramHandle.Init.SDBank             = FMC_SDRAM_BANK1;
   sdramHandle.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
   sdramHandle.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;
   sdramHandle.Init.MemoryDataWidth    = SDRAM_MEMORY_WIDTH;
   sdramHandle.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
   sdramHandle.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
   sdramHandle.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
   sdramHandle.Init.SDClockPeriod      = SDCLOCK_PERIOD;
   sdramHandle.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
   sdramHandle.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;

   /* Initialize the SDRAM controller */
   HAL_SDRAM_Init(&sdramHandle, &Timing);
  
   /* SDRAM initialization sequence */
   BSP_SDRAM_Initialization_sequence(REFRESH_COUNT_96MHZ);

} /* BoardConfigSDRAM */
#endif /* defined(MEMORY_INIT) */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

#if defined(MEMORY_INIT)
/*************************************************************************/
/*  MemoryInit                                                           */
/*                                                                       */
/*  If MEMORY_INIT is defined, the MemoryInit() function will be called. */
/*  By default  MemoryInit() is called after SystemInit() to enable an   */
/*  external memory controller.                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void MemoryInit (void)
{
   /*
    * SDRAM is still configured in system_stm32f7xx.c
    */
//   BoardConfigSDRAM();
   
} /* MemoryInit */
#endif

/*************************************************************************/
/*  tal_BoardEnableCOMx                                                  */
/*                                                                       */
/*  Board and hardware depending functionality to enable the COM port.   */
/*  If this port is not supported by the board, return an error.         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_BoardEnableCOM1 (void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;
   
   /* 
    * Use USART1, PA9 (TX) and PA10(RX)  
    */
   
   /* Turn on GPIOA and GPIOB clock */
   __HAL_RCC_GPIOA_CLK_ENABLE();    /*lint !e717*/

   /* Turn on USART1 clock */
   __HAL_RCC_USART1_CLK_ENABLE();   /*lint !e717*/

   /* Configure USART Tx as alternate function */
   GPIO_InitStructure.Pin       = GPIO_PIN_9;
   GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
   GPIO_InitStructure.Speed     = GPIO_SPEED_FAST;
   GPIO_InitStructure.Pull      = GPIO_PULLUP;
   GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Configure USART Rx as alternate function */
   GPIO_InitStructure.Pin       = GPIO_PIN_10;
   GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
   GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   return(TAL_OK);
} /* tal_BoardEnableCOM1 */

TAL_RESULT tal_BoardEnableCOM2 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM2 */

TAL_RESULT tal_BoardEnableCOM3 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM3 */

TAL_RESULT tal_BoardEnableCOM4 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM4 */

TAL_RESULT tal_BoardEnableCOM5 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM5 */

TAL_RESULT tal_BoardEnableCOM6 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM6 */

TAL_RESULT tal_BoardEnableCOM7 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM7 */

TAL_RESULT tal_BoardEnableCOM8 (void)
{
   return(TAL_ERR_COM_PORT_NOHW);
} /* tal_BoardEnableCOM8 */

/*************************************************************************/
/*  tal_BoardEnableCANx                                                  */
/*                                                                       */
/*  Board and hardware depending functionality to enable the CAN port.   */
/*  If this port is not supported by the board, return an error.         */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT tal_BoardEnableCAN1 (void)
{
   return(TAL_ERR_CAN_PORT_NOHW);
} /* tal_BoardEnableCAN1 */

TAL_RESULT tal_BoardEnableCAN2 (void)
{
   return(TAL_ERR_CAN_PORT_NOHW);
} /* tal_BoardEnableCAN2 */

/*************************************************************************/
/*  tal_BoardGetMACAddress                                               */
/*                                                                       */
/*  Retrieve the MAC address of the board.                               */
/*  In case of an error, a default address will be used.                 */
/*                                                                       */
/*  In    : pAddress                                                     */
/*  Out   : pAddress                                                     */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT tal_BoardGetMACAddress (uint8_t *pAddress)
{
   static uint8_t bMACRetrieved  = TAL_FALSE;
   static uint8_t  MACAddress[6] = {0x02,0x80,0xE1,0x00,0x00,0x00};
   uint32_t       dValue;
   
   if (TAL_FALSE == bMACRetrieved)
   {
      bMACRetrieved = TAL_TRUE;
      
      dValue = HAL_GetUIDw0();   
      MACAddress[5] = dValue & 0xFF; dValue = dValue >> 8;
      MACAddress[4] = dValue & 0xFF; dValue = dValue >> 8;
      MACAddress[3] = dValue & 0xFF;
   }
   
   /* Return MAC address */
   memcpy(pAddress, MACAddress, 6);
   
   return(TAL_OK);
} /* tal_BoardGetMACAddress */

/*************************************************************************/
/*  tal_BoardRTCSetTM                                                    */
/*                                                                       */
/*  Set the onboard RTC time by TM                                       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTCSetTM (struct tm *pTM)
{
   (void)pTM;
} /* tal_BoardRTCSetTM */

/*************************************************************************/
/*  tal_BoardRTCSetUnixtime                                              */
/*                                                                       */
/*  Set the onboard RTC time by Unixtime                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTCSetUnixtime (uint32_t Unixtime)
{
   (void)Unixtime;
} /* tal_BoardRTCSetUnixtime */

/*************************************************************************/
/*  tal_BoardRTC2System                                                  */
/*                                                                       */
/*  Set the system time from the RTC.                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_BoardRTC2System (void)
{
} /* tal_BoardRTC2System */

#endif /* USE_BOARD_MB1225 */

/*** EOF ***/
