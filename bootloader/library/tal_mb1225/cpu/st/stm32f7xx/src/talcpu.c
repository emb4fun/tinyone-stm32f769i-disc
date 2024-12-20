/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2018-2022 by Michael Fischer (www.emb4fun.de).
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
#define __TALCPU_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdlib.h>
#include <string.h>
#include "tal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static uint32_t dHiResPeriod   = 0;
static uint32_t dHiResPeriod16 = 0;

static RNG_HandleTypeDef RNGHandle;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 192000000
  *            HCLK(Hz)                       = 192000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = HSE_VALUE
  *            PLL_M                          = (HSE_VALUE/1000000)
  *            PLL_N                          = 384
  *            PLL_P                          = 2
  *            PLL_Q                          = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 6
  * @param  None
  * @retval None
  */
void SystemClock_Config (void)
{
   RCC_ClkInitTypeDef RCC_ClkInitStruct;
   RCC_OscInitTypeDef RCC_OscInitStruct;
   HAL_StatusTypeDef ret;


#if defined(STM32F769xx) || defined(STM32F779xx)
   /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();  /*lint !e717*/

   /* The voltage scaling allows optimizing the power consumption when the device is
      clocked below the maximum system frequency, to update the voltage scaling value
      regarding system frequency refer to product datasheet.  */
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); /*lint !e717*/
#endif


   /* Enable HSE Oscillator and activate PLL with HSE as source */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = (HSE_VALUE/1000000);
   RCC_OscInitStruct.PLL.PLLN = 384;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 8;

   ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
   if(ret != HAL_OK)
   {
      while(1) { ; }
   }

   /* Activate the OverDrive to reach the 216 MHz Frequency */
   ret = HAL_PWREx_EnableOverDrive();
   if(ret != HAL_OK)
   {
      while(1) { ; }
   }

   /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
   RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

   ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
   if(ret != HAL_OK)
   {
      while(1) { ; }
   }

} /* SystemClock_Config */

/*************************************************************************/
/*  NewSysTick_Config                                                    */
/*                                                                       */
/*  Based on the "core_cm4.h" version, but an AHB clock divided by 8 is  */
/*  used for the SysTick clock source.                                   */
/*                                                                       */
/*  The function initializes the System Timer and its interrupt, and     */
/*  starts the System Tick Timer. Counter is in free running mode to     */
/*  generate periodic interrupts.                                        */
/*                                                                       */
/*  In    : ticks                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static uint32_t NewSysTick_Config (uint32_t ticks)
{
   /*
    * AHB clock divided by 8 is used for the SysTick clock source.
    */
   ticks = ticks / 8;

   /* Reload value impossible */
   if ((ticks - 1) > SysTick_LOAD_RELOAD_Msk)  return (1);

   /* Set reload register */
   SysTick->LOAD = ticks - 1;

   /* Set Priority for Systick Interrupt */
   tal_CPUIrqSetPriority(SysTick_IRQn, SYSTICK_PRIO);

   /* Load the SysTick Counter Value */
   SysTick->VAL = 0;

   /*
    * SysTick IRQ and SysTick Timer must be
    * enabled with tal_CPUSysTickStart later.
    */

   return(ticks);
} /* NewSysTick_Config */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  tal_CPUInit                                                          */
/*                                                                       */
/*  "Initialize" the CPU.                                                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUInit (void)
{
   static uint8_t bInitDone = FALSE;

   if (FALSE == bInitDone)
   {
      bInitDone = TRUE;

      /*
       * If the program is started via the bootloader, further
       * initialization MUST NOT be carried out again since the
       * bootloader has already do this.
       */
#if defined(__BOOT__)
      /* Do nothing here, already done */
#else
      /* Enable I-Cache */
      SCB_EnableICache();

      /* Enable D-Cache */
      SCB_EnableDCache();

      /* Configure Instruction cache through ART accelerator */
#if (ART_ACCLERATOR_ENABLE != 0)
      __HAL_FLASH_ART_ENABLE();
#endif /* ART_ACCLERATOR_ENABLE */

      /* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0U)
      __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

      /*
       * Enable HSE oscillator and configure the PLL when using HSE
       * oscillator as PLL clock source.
       */
      SystemClock_Config();
#endif /* defined(__BOOT__) */

      /**********************************************************/

      /* Update clock info */
      SystemCoreClockUpdate();

      /* Init SysTick */
      dHiResPeriod = NewSysTick_Config(SystemCoreClock / OS_TICKS_PER_SECOND);

      /*
       * dHiResPeriod value must be a 16bit count, but here it is
       * bigger. Therefore dHiResPeriod must be divided by 16.
       */
      dHiResPeriod16 = dHiResPeriod / 16;

      /* Configure Priority Grouping (core_cm4.h) */
      NVIC_SetPriorityGrouping(0);

      /* System configuration controller clock enable */
      RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
   }

} /* tal_CPUInit */

/*************************************************************************/
/*  tal_CPUSysTickStart                                                  */
/*                                                                       */
/*  Start the SysTick.                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUSysTickStart (void)
{
   /* Enable SysTick IRQ and SysTick Timer */
   SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

} /* tal_CPUSysTickStart */

/*************************************************************************/
/*  tal_CPUIrqEnable                                                     */
/*                                                                       */
/*  Enable the given IRQ.                                                */
/*                                                                       */
/*  In    : IRQ                                                          */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqEnable (int IRQ)
{
   NVIC_EnableIRQ((IRQn_Type)IRQ);

} /* tal_CPUIrqEnable */

/*************************************************************************/
/*  tal_CPUIrqDisable                                                    */
/*                                                                       */
/*  Disable the given IRQ.                                               */
/*                                                                       */
/*  In    : IRQ                                                          */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqDisable (int IRQ)
{
   NVIC_DisableIRQ((IRQn_Type)IRQ);

} /* tal_CPUIrqDisable */

/*************************************************************************/
/*  tal_CPUIrqDisableAll                                                 */
/*                                                                       */
/*  Disable all interrupts.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqDisableAll (void)
{
   /*lint +rw(_to_semi) */
   /*lint -d__disable_irq=_to_semi */

   __disable_irq();

} /* tal_CPUIrqDisableAll */

/*************************************************************************/
/*  tal_CPUIrqSetPriority                                                */
/*                                                                       */
/*  Set priority of the given IRQ.                                       */
/*                                                                       */
/*  In    : IRQ, Priority                                                */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUIrqSetPriority (int IRQ, int Priority)
{
   NVIC_SetPriority((IRQn_Type)IRQ, (uint32_t)Priority);

} /* tal_CPUIrqSetPriority */

/*************************************************************************/
/*  tal_CPUStatGetHiResPeriod                                            */
/*                                                                       */
/*  Return the HiResPeriod value.                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: HiResPeriod                                                  */
/*************************************************************************/
uint32_t tal_CPUStatGetHiResPeriod (void)
{
   return(dHiResPeriod16);
} /* tal_CPUStatGetHiResPeriod */

/*************************************************************************/
/*  tal_CPUStatGetHiResCnt                                               */
/*                                                                       */
/*  Return the HiRes counter.                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: HiRes counter                                                */
/*************************************************************************/
uint32_t tal_CPUStatGetHiResCnt (void)
{
   uint32_t dValue;

   /* Get milliseconds */
   dValue  = (OS_TimeGet() << 16);

   /* The SysTick counts down from HiResPeriod, therefore HiResPeriod - X */
   /* HiResPeriod is used, therefore divide the time by 16 too */
   dValue |= (uint16_t)((dHiResPeriod - SysTick->VAL) / 16);

   return(dValue);
} /* tal_CPUStatGetHiResCnt */

/*************************************************************************/
/*  tal_CPUGetFrequencyCPU                                               */
/*                                                                       */
/*  Return the clock frequency of the CPU in MHz.                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Frequency                                                    */
/*************************************************************************/
uint32_t tal_CPUGetFrequencyCPU (void)
{
   uint32_t dFrequency;

   SystemCoreClockUpdate();
   dFrequency = SystemCoreClock;

   return(dFrequency);
} /* tal_CPUGetFrequencyCPU */

/*************************************************************************/
/*  tal_CPUGetFrequencyAHB                                               */
/*                                                                       */
/*  Return the clock frequency of the AHB bus in MHz.                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Frequency                                                    */
/*************************************************************************/
uint32_t tal_CPUGetFrequencyAHB (void)
{
   uint32_t dFrequency;
   uint32_t dDiv;

   SystemCoreClockUpdate();

   dDiv = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
   dFrequency = SystemCoreClock >> dDiv;

   return(dFrequency);
} /* tal_CPUGetFrequencyAHB */

/*************************************************************************/
/*  tal_CPUGetFrequencyAPB1                                              */
/*                                                                       */
/*  Return the clock frequency of the APB1 bus in MHz.                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Frequency                                                    */
/*************************************************************************/
uint32_t tal_CPUGetFrequencyAPB1 (void)
{
   uint32_t dFrequency;

   SystemCoreClockUpdate();
   dFrequency = HAL_RCC_GetPCLK1Freq();

   return(dFrequency);
} /* tal_CPUGetFrequencyAPB1 */

/*************************************************************************/
/*  tal_CPUGetFrequencyAPB2                                              */
/*                                                                       */
/*  Return the clock frequency of the APB2 bus in MHz.                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Frequency                                                    */
/*************************************************************************/
uint32_t tal_CPUGetFrequencyAPB2 (void)
{
   uint32_t dFrequency;

   SystemCoreClockUpdate();
   dFrequency = HAL_RCC_GetPCLK2Freq();

   return(dFrequency);
} /* tal_CPUGetFrequencyAPB2 */

/*************************************************************************/
/*  tal_CPURngInit                                                       */
/*                                                                       */
/*  Initialize the random number generator.                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPURngInit (void)
{
   memset(&RNGHandle, 0x00, sizeof(RNGHandle));

   /* RNG Peripheral clock enable */
   __HAL_RCC_RNG_CLK_ENABLE();      /*lint !e717*/

   /* Enable RNG reset state */
   __HAL_RCC_RNG_FORCE_RESET();     /*lint !e717*/

   /* Release RNG from reset state */
   __HAL_RCC_RNG_RELEASE_RESET();

   RNGHandle.Instance = RNG;
   HAL_RNG_Init(&RNGHandle);

} /* tal_CPURngInit */

/*************************************************************************/
/*  tal_CPURngDeInit                                                     */
/*                                                                       */
/*  DeInitialize the random number generator.                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPURngDeInit (void)
{
   HAL_RNG_DeInit(&RNGHandle);

   /* Enable RNG reset state */
   __HAL_RCC_RNG_FORCE_RESET();     /*lint !e717*/

   /* Release RNG from reset state */
   __HAL_RCC_RNG_RELEASE_RESET();

   /* RNG Peripheral clock disable */
   __HAL_RCC_RNG_CLK_DISABLE();

} /* tal_CPURngDeInit */

/*************************************************************************/
/*  tal_CPURngHardwarePoll                                               */
/*                                                                       */
/*  Generates a 32-bit random number.                                    */
/*                                                                       */
/*  In    : pData, dSize                                                 */
/*  Out   : pData                                                        */
/*  Return: TALOK / TAL_ERROR                                            */
/*************************************************************************/
TAL_RESULT tal_CPURngHardwarePoll (uint8_t *pData, uint32_t dSize)
{
   TAL_RESULT        Error = TAL_ERROR;
   HAL_StatusTypeDef Status;
   uint32_t          Data;

   for (uint32_t i = 0; i < dSize; ++i)
   {
      Status = HAL_RNG_GenerateRandomNumber(&RNGHandle, &Data);
      if (Status != HAL_OK) goto RngHardwarePollExit; /*lint !e801 */

      pData[i] = (uint8_t)(Data & 0xFF);
   }

   Error = TAL_OK;

RngHardwarePollExit:
   return(Error);
} /* tal_CPURngHardwarePoll */

/*************************************************************************/
/*  tal_CPUReboot                                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void tal_CPUReboot (void)
{
#if defined(__DEBUG__)
   term_printf("*** Reboot ***\r\n");
   OS_TimeDly(500);
#endif

#if defined(__FLASH__)
{
   IWDG_HandleTypeDef IwdgHandle;

   IwdgHandle.Instance       = IWDG;
   IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;  /* Set a clock 1 kKHz */
   IwdgHandle.Init.Reload    = 100;
   IwdgHandle.Init.Window    = IWDG_WINDOW_DISABLE;

   HAL_IWDG_Init(&IwdgHandle);
}
#endif

   /*
    * Wait for watchdog reset
    */
   TAL_CPU_DISABLE_ALL_INTS();
   while (1)
   {
      __asm__ ("nop");
   }
   TAL_CPU_ENABLE_ALL_INTS(); /*lint !e527*/

} /* tal_CPUReboot */

/*************************************************************************/
/*  SysTick_Handler                                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void SysTick_Handler (void)
{
   TAL_CPU_IRQ_ENTER();

   OS_TimerCallback();

   HAL_IncTick();

   TAL_CPU_IRQ_EXIT();
} /* SysTick_Handler */


/*** EOF ***/
