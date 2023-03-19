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
#define __TALCPU_COM_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include "tal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static TAL_COM_DCB *DCBArray[TAL_COM_PORT_MAX];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  IRQHandler                                                           */
/*                                                                       */
/*  This is the generic IRQ handler.                                     */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void IRQHandler (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error;
   USART_TypeDef  *pUSARTx;
   uint32_t        dStatus;
   uint8_t         bData;

   /* Get USART pointer */
   pUSARTx = (USART_TypeDef *)pDCB->HW.dBaseAddress;

   /* Get IRQ status */   
   dStatus = pUSARTx->ISR;
   
   /* 
    * Check for RX interrupt
    * "Read data register not empty" and "Overrun error"  
    */
   if (dStatus & (USART_ISR_RXNE | USART_ISR_ORE))
   {
      /* Read data */
      bData = (uint8_t)pUSARTx->RDR;
      
      /* If we have no overflow... */
      if (TAL_FALSE == pDCB->bRxOverflow)
      {
         /* ... put it into the ring buffer */
         Error = tal_MISCRingAdd(&pDCB->RxRing, &bData);
         if (TAL_OK == Error)
         {
            /* Signal counting semaphore */
            OS_SemaSignalFromInt(&pDCB->RxRdySema);
         }
         else
         {
            /* Ups, overflow */
            pDCB->bRxOverflow = TAL_OK;
         }
      }  
   } /* end "RX interrupt" */
   
   
   /* 
    * Check for TX interrupt, but only if it is anabled 
    */
   if ( (pUSARTx->CR1 & USART_CR1_TXEIE) && (dStatus & USART_ISR_TXE) )
   {
      /* Read Data from the ring buffer */
      Error = tal_MISCRingGet(&pDCB->TxRing, &bData);
      if (Error != TAL_OK)
      {
         /* Ups, no data available, disable interrupt */
         pUSARTx->CR1 &= ~USART_CR1_TXEIE;
      }
      else
      {
         /* Send data */
         pUSARTx->TDR = bData;
      }   
   } /* end "TX interrupt */
   
} /* IRQHandler */

/*************************************************************************/
/*  CalcBRR                                                              */
/*                                                                       */
/*  Calculate the baud rate register value.                              */
/*  Based on the functionality of USART_Init from ST.                    */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: BRR value                                                    */
/*************************************************************************/
static uint16_t CalcBRR (TAL_COM_DCB *pDCB)
{
   uint16_t wBRR;
   uint32_t dAPBClock;
   uint32_t dDivider;
   uint32_t dTempReg;
   
   if ( (TAL_COM_PORT_1 == pDCB->ePort) || (TAL_COM_PORT_6 == pDCB->ePort) )
   {
      dAPBClock = tal_CPUGetFrequencyAPB2();
   }
   else
   {
      dAPBClock = tal_CPUGetFrequencyAPB1();
   }

   /* Determine the integer part */
   if (TAL_TRUE == pDCB->HW.bOver8)
   {
      dDivider  = ((((dAPBClock)*2)+((pDCB->Settings.dBaudrate)/2))/((pDCB->Settings.dBaudrate)));
      dTempReg  = dDivider & 0xFFF0U;
      dTempReg |= (uint16_t)((dDivider & (uint16_t)0x000FU) >> 1U);
   }
   else
   {
      dTempReg = ((((dAPBClock))+((pDCB->Settings.dBaudrate)/2))/((pDCB->Settings.dBaudrate)));
   }

   /* Write to USART BRR register */
   wBRR = (uint16_t)dTempReg;

   return(wBRR);   
} /* CalcBRR */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  USARTx_IRQHandler                                                    */
/*                                                                       */
/*  This is the Cortex USARTx IRQ handler.                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void USART1_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_1]);
   TAL_CPU_IRQ_EXIT();
} /* USART1_IRQHandler */

void USART2_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_2]);
   TAL_CPU_IRQ_EXIT();   
} /* USART2_IRQHandler */

void USART3_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_3]);
   TAL_CPU_IRQ_EXIT();   
} /* USART3_IRQHandler */

void UART4_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_4]);
   TAL_CPU_IRQ_EXIT();   
} /* UART4_IRQHandler */

void UART5_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_5]);
   TAL_CPU_IRQ_EXIT();   
} /* UART5_IRQHandler */

void USART6_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_6]);
   TAL_CPU_IRQ_EXIT();   
} /* USART6_IRQHandler */

void UART7_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_7]);
   TAL_CPU_IRQ_EXIT();   
} /* UART7_IRQHandler */

void UART8_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_8]);
   TAL_CPU_IRQ_EXIT();   
} /* UART8_IRQHandler */

/*************************************************************************/
/*  cpu_COMInit                                                          */
/*                                                                       */
/*  Prepare the hardware for use by the Open function later. Set the HW  */
/*  information depending of ePort and "enable" the COM port.            */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMInit (TAL_COM_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_ERR_COM_PORT_RANGE;
   TAL_COM_HW  *pHW    = &pDCB->HW;
   
   switch (pDCB->ePort)
   {
      case TAL_COM_PORT_1:
      {
         Error = tal_BoardEnableCOM1();
         if (TAL_OK == Error)
         {
            DCBArray[TAL_COM_PORT_1] = pDCB;
         
            pHW->dBaseAddress = USART1_BASE;
            pHW->IrqNumber    = USART1_IRQn;
            pHW->bIrqPriority = USART1_PRIO;
            pHW->bOver8       = USART1_OVER_8; 
            
            /* Reset USART */
            RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
            RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
         }
         break;
      } /* TAL_COM_PORT_1 */
      
      default:
      {
         /* Do nothing */
         break;
      }
   } /* end switch (pDCB->ePort) */
   
   return(Error);
} /* cpu_COMInit */

/*************************************************************************/
/*  cpu_COMOpen                                                          */
/*                                                                       */
/*  Open the COM port.                                                   */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMOpen (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_ERROR;
   USART_TypeDef  *pUSARTx;
   uint16_t        wBRR = 0;
   uint16_t        wCR1 = 0;
   uint16_t        wCR2 = 0;
   uint8_t         bDummy;
   
   /* Get USART pointer */
   pUSARTx = (USART_TypeDef *)pDCB->HW.dBaseAddress;
         
   /* Check word length */
   switch (pDCB->Settings.eLength)
   {
      case TAL_COM_LENGTH_8:
      {
         /* Do nothing */
         break;
      }
      
      case TAL_COM_LENGTH_9:
      {
         wCR1 |= USART_CR1_M; /*lint !e734*/
         break;
      }
   
      default:
      {
         Error = TAL_ERR_COM_LENGTH;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eLength) */
   
   /* Check parity settings */
   switch (pDCB->Settings.eParity)
   {
      case TAL_COM_PARITY_NONE:
      {
         /* Do nothing */
         break;
      } 
      
      case TAL_COM_PARITY_EVEN:
      {
         wCR1 |= USART_CR1_PCE;
         break;
      }
      
      case TAL_COM_PARITY_ODD:
      {
         wCR1 |= (USART_CR1_PCE | USART_CR1_PS);
         break;
      }
      
      default:
      {
         Error = TAL_ERR_COM_PARITY;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eParity) */
    
   /* Check stop bit settings */
   switch (pDCB->Settings.eStop)
   {
      case TAL_COM_STOP_0_5:
      {
         wCR2 |= USART_CR2_STOP_0;
         break;
      }
      
      case TAL_COM_STOP_1_0:
      {
         /* Do nothing */
         break;
      }
      
      case TAL_COM_STOP_1_5:
      {
         wCR2 |= (USART_CR2_STOP_1 | USART_CR2_STOP_0);
         break;
      }
      
      case TAL_COM_STOP_2_0:
      {
         wCR2 |= USART_CR2_STOP_1;
         break;
      }
      
      default:
      {
         Error = TAL_ERR_COM_STOP;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eStop) */

   /* Check baud rate */
   if (pDCB->Settings.dBaudrate != 0)
   {
      if (TAL_TRUE == pDCB->HW.bOver8)
      {
         wCR1 |= USART_CR1_OVER8;
      }   
      
      wBRR = CalcBRR(pDCB);
   }
   else
   {
      Error = TAL_ERR_COM_BAUDRATE;
      goto COMOpenEnd;  /*lint !e801*/
   }

   /* Enable transmitter and receiver */
   wCR1 |= (USART_CR1_TE | USART_CR1_RE);
   
   /* Enable USART itself */
   wCR1 |= USART_CR1_UE;
   
   /* Configure USART */   
   pUSARTx->CR1 = wCR1;
   pUSARTx->CR2 = wCR2;
   pUSARTx->BRR = wBRR;
   
   /* Dummy read to clear the DR register */
   bDummy = (uint8_t)pUSARTx->RDR;
   (void)bDummy;
   
   /* Enable RX interrrupt */
   pUSARTx->CR1 |= USART_CR1_RXNEIE;
   
   /* Set priority and enable the IRQ */
   tal_CPUIrqSetPriority(pDCB->HW.IrqNumber, pDCB->HW.bIrqPriority);
   tal_CPUIrqEnable(pDCB->HW.IrqNumber);

   Error = TAL_OK;
   
COMOpenEnd:   
   return(Error);
} /* cpu_COMOpen */

/*************************************************************************/
/*  cpu_COMClose                                                         */
/*                                                                       */
/*  Close the COM port.                                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMClose (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_OK;
   USART_TypeDef  *pUSARTx;
   
   /* Get USART pointer */
   pUSARTx = (USART_TypeDef *)pDCB->HW.dBaseAddress;

   /* Disable the IRQ */   
   tal_CPUIrqDisable(pDCB->HW.IrqNumber);
   
   /* Disable the  USART itself */
   pUSARTx->CR1 &= ~USART_CR1_UE;
   
   /* Disable TX and RX interrupt  */
   pUSARTx->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_RXNEIE);
   
   return(Error);
} /* cpu_COMClose */

/*************************************************************************/
/*  cpu_COMStartTx                                                       */
/*                                                                       */
/*  Send the data from the ring buffer if the TX interrupt is disabled.  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMStartTx (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_OK;
   USART_TypeDef  *pUSARTx;
   uint8_t         bData;
   
   /* Get USART pointer */
   pUSARTx = (USART_TypeDef *)pDCB->HW.dBaseAddress;

   TAL_CPU_DISABLE_ALL_INTS();
   if (pUSARTx->CR1 & USART_CR1_TXEIE)
   {
      /* TX interrupt is enabled, do nothing */
   }
   else
   {
      /* Get data from the ring buffer */
      Error = tal_MISCRingGet(&pDCB->TxRing, &bData);
      if (TAL_OK == Error)
      {
         /* Send data */
         pUSARTx->TDR = bData;
      
         /* Enable TX interrupt */
         pUSARTx->CR1 |= USART_CR1_TXEIE;
      }
   }
   TAL_CPU_ENABLE_ALL_INTS();
   
   return(Error);
} /* cpu_COMStartTx */

/*************************************************************************/
/*  cpu_COMTxIsRunning                                                   */
/*                                                                       */
/*  Check if TX is still running.                                        */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT cpu_COMTxIsRunning (TAL_COM_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_OK;
   USART_TypeDef  *pUSARTx;
   
   /* Get USART pointer */
   pUSARTx = (USART_TypeDef *)pDCB->HW.dBaseAddress;

   TAL_CPU_DISABLE_ALL_INTS();
   if (pUSARTx->CR1 & USART_CR1_TXEIE)
   {
      /* TX is still running */
      Error = TAL_OK;
   }
   else
   {
      /* TX is not running */
      Error = TAL_ERROR;
   }
   TAL_CPU_ENABLE_ALL_INTS();
   
   return(Error);
} /* cpu_COMTxIsRunning */

/*** EOF ***/
