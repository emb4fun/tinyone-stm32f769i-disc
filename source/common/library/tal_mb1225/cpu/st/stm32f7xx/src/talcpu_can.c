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
#define __TALCPU_CAN_C__

/*
 * The STM32F4xx supports 28 filter banks shared between CAN1 and CAN2.
 * Here the banks will be divided between CAN1 and CAN2.
 *
 * CAN1:  0...13, FIFO 0
 * CAN2: 14...27, FIFO 1
 */

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>
#include "tal.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Filter defines
 */

/* Must be CAN_MAX_FILTER_BANKS together */
#define CAN1_FILTER_BANK_COUNT   14
#define CAN2_FILTER_BANK_COUNT   14

#if ((CAN1_FILTER_BANK_COUNT + CAN2_FILTER_BANK_COUNT) != CAN_MAX_FILTER_BANKS)
   Error: Filter bank count != CAN_MAX_FILTER_BANKS;
#endif   

#define CAN1_FILTER_BANK_START   0
#define CAN1_FILTER_BANK_END     (CAN1_FILTER_BANK_START + CAN1_FILTER_BANK_COUNT - 1)

#define CAN2_FILTER_BANK_START   (CAN1_FILTER_BANK_END + 1)
#define CAN2_FILTER_BANK_END     (CAN2_FILTER_BANK_START + CAN2_FILTER_BANK_COUNT - 1)

#define FILTER_0_MASK            ((1L << CAN1_FILTER_BANK_COUNT) - 1)
#define FILTER_1_MASK            (((1L << CAN2_FILTER_BANK_COUNT) - 1) << CAN1_FILTER_BANK_COUNT)


#define FILTER_MS_MASK_16_BIT    0
#define FILTER_MS_MASK_32_BIT    1
#define FILTER_MS_LIST_16_BIT    2
#define FILTER_MS_LIST_32_BIT    3


/*
 * These defines was not set correct by the header files.
 * Therefore we must define it here new.
 */
#undef  CAN_SJW_2TQ
#define CAN_SJW_2TQ              (2-1)
#undef  CAN_BS2_2TQ
#define CAN_BS2_2TQ              (2-1)
#undef  CAN_BS1_13TQ
#define CAN_BS1_13TQ             (13-1)


/*
 * Time out for INAK bit 
 */
#undef INAK_TIMEOUT 
#define INAK_TIMEOUT    0x0000FFFF

/*
 * Used to convert the 8 byte of a TAL_CAN_OBJECT
 * to the Mailbox data field TDLR and TDHR.
 */
typedef struct _mdata_
{
   uint32_t MData[2];
} MDATA;

/*
 * Some CAN timing calculators:
 * http://www.bittiming.can-wiki.info
 * http://www.port.de/de/bit-timing.html
 */ 

/*
 * BTR defines for 48MHz clock, Sample Point 87.5% (16tq)
 */
#define BTR_1000K_48MHZ ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | (  3-1))
#define BTR_500K_48MHZ  ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | (  6-1))
#define BTR_250K_48MHZ  ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | ( 12-1))
#define BTR_125K_48MHZ  ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | ( 24-1))
#define BTR_100K_48MHZ  ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | ( 30-1))
#define BTR_50K_48MHZ   ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | ( 60-1))
#define BTR_20K_48MHZ   ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | (150-1))
#define BTR_10K_48MHZ   ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | (300-1))
#define BTR_5K_48MHZ    ((CAN_SJW_2TQ<<24) | (CAN_BS2_2TQ<<20) | (CAN_BS1_13TQ<<16) | (600-1))

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static TAL_CAN_DCB *DCBArray[TAL_CAN_PORT_MAX];
static uint8_t     bFilterSetupDone = TAL_FALSE;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  SendObject                                                           */
/*                                                                       */
/*  Try to send the given Object.                                        */
/*                                                                       */
/*  In    : pCANx, pCANObject                                            */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
static TAL_RESULT SendObject (CAN_TypeDef *pCANx, TAL_CAN_OBJECT *pCANObject)
{
   TAL_RESULT  Error = TAL_ERROR;
   uint8_t    bMboxIndex = 0;
   MDATA     *pMsgData; 
   uint32_t   dTIR;
   uint32_t   dTDTR;
   uint32_t   dTDLR;
   uint32_t   dTDHR;
   
   /* Find a free mailbox */
   if      (pCANx->TSR & CAN_TSR_TME0)
   {
      bMboxIndex = 0;
   }
   else if (pCANx->TSR & CAN_TSR_TME1)
   {
      bMboxIndex = 1;
   }
   else if (pCANx->TSR & CAN_TSR_TME2)
   {
      bMboxIndex = 2;
   }
   else
   {
      Error = TAL_ERROR;
      goto SendObjectEnd;  /*lint !e801*/
   }
   
   /* Setup Identifier */
   if (pCANObject->bFlags & TAL_CAN_OBJECT_FLAGS_STD_ID)
   {
      dTIR = pCANObject->dIdentifier << 21;
   }
   else
   {
      dTIR = (pCANObject->dIdentifier << 3) | CAN_TI0R_IDE;
   }   
   
   /* Setup DLC */
   dTDTR = pCANObject->bDLC & CAN_TDT0R_DLC;

   /* Setup Data */         
   pMsgData = (MDATA*)&pCANObject->Data[0];
   dTDLR    = pMsgData->MData[0];
   dTDHR    = pMsgData->MData[1];

   /* Setup mailbox */
   pCANx->sTxMailBox[bMboxIndex].TIR  = dTIR;
   pCANx->sTxMailBox[bMboxIndex].TDTR = dTDTR;
   pCANx->sTxMailBox[bMboxIndex].TDLR = dTDLR;
   pCANx->sTxMailBox[bMboxIndex].TDHR = dTDHR;

   /* Request transmission */
   pCANx->sTxMailBox[bMboxIndex].TIR |= CAN_TI0R_TXRQ;

   Error = TAL_OK;

SendObjectEnd:
   return(Error);   
} /* SendObject */

/*************************************************************************/
/*  TX_IRQHandler                                                        */
/*                                                                       */
/*  This is the generic TX IRQhandler.                                   */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void TX_IRQHandler (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT     Error;
   TAL_CAN_OBJECT CANObject;
   CAN_TypeDef  *pCANx;

   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;

   /* 
    * Check for TX interrupt, but only if it is anabled 
    */
   if (pCANx->IER & CAN_IER_TMEIE)
   {
      /* Read Object from the ring buffer */
      Error = tal_MISCRingGet(&pDCB->TxRing, (uint8_t*)&CANObject);
      if (Error != TAL_OK)
      {
         /* Ups, no data available, disable interrupt */
         pCANx->IER &= ~CAN_IER_TMEIE;
      }
      else
      {
         /* Send Object */
         SendObject(pCANx, &CANObject); 
      }   
   } /* end "TX interrupt */

   (void)pDCB;
} /* TX_IRQHandler */

/*************************************************************************/
/*  RX_IRQHandler                                                        */
/*                                                                       */
/*  This is the generic RX IRQhandler.                                   */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void RX_IRQHandler (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT     Error;
   TAL_CAN_OBJECT CANObject;
   CAN_TypeDef  *pCANx;
   MDATA        *pMsgData; 
   uint8_t       bFIFONumber;
   uint8_t       bFilterMatchIndex;
   uint32_t      dFifoStatus;
   uint32_t      dTIR;
   uint32_t      dTDTR;
   uint32_t      dTDLR;
   uint32_t      dTDHR;
   
   /* Set timestamp, if supported */
   if (NULL == pDCB->GetRxTimestamp)
   { 
      CANObject.dTimestamp = 0;
   }
   else
   {
      CANObject.dTimestamp = pDCB->GetRxTimestamp();
   }   
   
   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;
   
   /* Get associated FIFO */  
   bFIFONumber = pDCB->HW.bFIFONumber;
   
   /* Get FIFO status (CAN_RFxR) */
   dFifoStatus = *pDCB->HW.pRFxR;
   
   /* Clear overrun */
   dFifoStatus |= CAN_RF0R_FOVR0;
   
   /* Clear full */
   dFifoStatus |= CAN_RF0R_FULL0;
      
   /* Get CAN message */
   dTIR  = pCANx->sFIFOMailBox[bFIFONumber].RIR;
   dTDTR = pCANx->sFIFOMailBox[bFIFONumber].RDTR;
   dTDLR = pCANx->sFIFOMailBox[bFIFONumber].RDLR;
   dTDHR = pCANx->sFIFOMailBox[bFIFONumber].RDHR;

   /* Release FIFO output mailbox */   
   dFifoStatus |= CAN_RF0R_RFOM0;

   /* Write new CAN_RFxR value */
   *pDCB->HW.pRFxR = dFifoStatus;
      
   /* Get Identifier and IDE info */
   if (dTIR & CAN_RI0R_IDE)
   {
      CANObject.dIdentifier = (dTIR >> 3) & 0x1FFFFFFF; 
      CANObject.bFlags      = TAL_CAN_OBJECT_FLAGS_EXT_ID;
   }
   else
   {
      CANObject.dIdentifier = (dTIR >> 21) & 0x7FF;
      CANObject.bFlags      = TAL_CAN_OBJECT_FLAGS_STD_ID;
   }   
   
   /* Get DLC */
   CANObject.bDLC = (dTDTR & CAN_RDT0R_DLC);
   
   /* Get filter match index */
   bFilterMatchIndex = (uint8_t)((dTDTR & CAN_RDT0R_FMI) >> 8);
   (void)bFilterMatchIndex;
   
   /* Get data */
   pMsgData = (MDATA*)&CANObject.Data[0];
   pMsgData->MData[0] = dTDLR;
   pMsgData->MData[1] = dTDHR;
   
   /* If we have no overflow... */
   if (TAL_FALSE == pDCB->bRxOverflow)
   {
      /* ... put it into the ring buffer */
      Error = tal_MISCRingAdd(&pDCB->RxRing, (uint8_t*)&CANObject);
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
   
} /* RX_IRQHandler */

/*************************************************************************/
/*  SCE_IRQHandler                                                       */
/*                                                                       */
/*  This is the generic SCE IRQhandler.                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SCE_IRQHandler (TAL_CAN_DCB *pDCB)
{
   (void)pDCB;
} /* SCE_IRQHandler */

/*************************************************************************/
/*  AddIDtoBank                                                          */
/*                                                                       */
/*  Add the identifier to the filter bank.                               */
/*                                                                       */
/*  Note: The entry which is not used, must be filled with the ID too.   */
/*        But here, do not set the InUseMask.                            */
/*                                                                       */
/*  In    : bModeScale, dFilterID, pFilter                               */
/*  Out   : none                                                         */
/*  Return: TAL_FALSE / TAL_TRUE                                         */
/*************************************************************************/
static void AddIDtoBank  (uint8_t       bModeScale,
                          uint32_t      dFilterID,
                          FITLER_ENTRY *pFilter)
{
   /* Set Mode and Scale information */
   pFilter->bModeScale = bModeScale; 

   if (FILTER_MS_LIST_16_BIT == bModeScale)
   {
      /*
       * Find a free entry and add the identifier here
       */
      if      (0 == (pFilter->bInUseMask & 0x01))
      {
         pFilter->bInUseMask        |= 0x01;
         pFilter->Bank.List16.wID[0] = (uint16_t)dFilterID;
      }
      else if (0 == (pFilter->bInUseMask & 0x02))
      {
         pFilter->bInUseMask        |= 0x02;
         pFilter->Bank.List16.wID[1] = (uint16_t)dFilterID;
      }
      else if (0 == (pFilter->bInUseMask & 0x04))
      {
         pFilter->bInUseMask        |= 0x04;
         pFilter->Bank.List16.wID[2] = (uint16_t)dFilterID;
      }
      else if (0 == (pFilter->bInUseMask & 0x08))
      {
         pFilter->bInUseMask        |= 0x08;
         pFilter->Bank.List16.wID[3] = (uint16_t)dFilterID;
      }
      
      /* Fill the rest free entries with the identifier too */
      if (0 == (pFilter->bInUseMask & 0x01)) pFilter->Bank.List16.wID[0] = (uint16_t)dFilterID;
      if (0 == (pFilter->bInUseMask & 0x02)) pFilter->Bank.List16.wID[1] = (uint16_t)dFilterID;
      if (0 == (pFilter->bInUseMask & 0x04)) pFilter->Bank.List16.wID[2] = (uint16_t)dFilterID;
      if (0 == (pFilter->bInUseMask & 0x08)) pFilter->Bank.List16.wID[3] = (uint16_t)dFilterID;
      
   } /* end if (FILTER_MS_LIST_16_BIT == bModeScale) */


   if (FILTER_MS_LIST_32_BIT == bModeScale)
   {
      /*
       * Find a free entry and add the identifier
       */
      if      (0 == (pFilter->bInUseMask & 0x01))
      {
         pFilter->bInUseMask        |= 0x01;
         pFilter->Bank.List32.dID[0] = dFilterID;
      }
      else if (0 == (pFilter->bInUseMask & 0x02))
      {
         pFilter->bInUseMask        |= 0x02;
         pFilter->Bank.List32.dID[1] = dFilterID;
      }
      
      /* Fill the rest free entries with the identifier too */
      if (0 == (pFilter->bInUseMask & 0x01)) pFilter->Bank.List32.dID[0] = dFilterID;
      if (0 == (pFilter->bInUseMask & 0x02)) pFilter->Bank.List32.dID[1] = dFilterID;
      
   } /* end if (FILTER_MS_LIST_32_BIT == bModeScale) */

} /* AddIDtoBank */                          

/*************************************************************************/
/*  FindUseFilterAndRemove                                               */
/*                                                                       */
/*  Find the filter bank which use this identifier, and remove it.       */
/*                                                                       */
/*  Note: The entry which is not used, must be filled with an ID too.    */
/*        But here, do not set the InUseMask.                            */
/*                                                                       */
/*  In    : bModeScale, dFilterID, pFilter                               */
/*  Out   : none                                                         */
/*  Return: NULL / pFilter                                               */
/*************************************************************************/
static FITLER_ENTRY *FindUseFilterAndRemove (uint8_t       bModeScale, 
                                             uint32_t      dFilterID,
                                             FITLER_ENTRY *pFilter)
{
   FITLER_ENTRY *pFoundFilter = NULL;   
   
   if (pFilter->bInUseMask != 0)
   {
      if (FILTER_MS_LIST_16_BIT == bModeScale)
      {
         /*
          * Find the entry with the identifier
          */
         if      (pFilter->bInUseMask & 0x01)
         {
            if (pFilter->Bank.List16.wID[0] == (uint16_t)dFilterID) 
            {
               pFoundFilter         = pFilter; 
               pFilter->bInUseMask &= ~0x01;
            }   
         }
         else if (pFilter->bInUseMask & 0x02)
         {
            if (pFilter->Bank.List16.wID[1] == (uint16_t)dFilterID) 
            {
               pFoundFilter         = pFilter; 
               pFilter->bInUseMask &= ~0x02;
            }   
         }
         else if (pFilter->bInUseMask & 0x04)
         {
            if (pFilter->Bank.List16.wID[2] == (uint16_t)dFilterID) 
            {
               pFoundFilter         = pFilter; 
               pFilter->bInUseMask &= ~0x04;
            }   
         }
         else if (pFilter->bInUseMask & 0x08)
         {
            if (pFilter->Bank.List16.wID[3] == (uint16_t)dFilterID) 
            {
               pFoundFilter         = pFilter; 
               pFilter->bInUseMask &= ~0x08;
            }   
         }
         
         /* Check if the empty entries must be filled with an identifier */
         if (pFoundFilter)
         {  
            if (0 == pFilter->bInUseMask)
            {
               /* No used entries available */
               pFilter->bModeScale         = 0;
               pFilter->Bank.List16.wID[0] = 0;
               pFilter->Bank.List16.wID[1] = 0;
               pFilter->Bank.List16.wID[2] = 0;
               pFilter->Bank.List16.wID[3] = 0;
            }
            else
            {
               /* Used entries available, find a used identifier */
               if (pFilter->bInUseMask & 0x01) dFilterID = pFilter->Bank.List16.wID[0];
               if (pFilter->bInUseMask & 0x02) dFilterID = pFilter->Bank.List16.wID[1];
               if (pFilter->bInUseMask & 0x04) dFilterID = pFilter->Bank.List16.wID[2];
               if (pFilter->bInUseMask & 0x08) dFilterID = pFilter->Bank.List16.wID[3];
               
               /* Fill the rest free entries with the identifier too */
               if (0 == (pFilter->bInUseMask & 0x01)) pFilter->Bank.List16.wID[0] = (uint16_t)dFilterID;
               if (0 == (pFilter->bInUseMask & 0x02)) pFilter->Bank.List16.wID[1] = (uint16_t)dFilterID;
               if (0 == (pFilter->bInUseMask & 0x04)) pFilter->Bank.List16.wID[2] = (uint16_t)dFilterID;
               if (0 == (pFilter->bInUseMask & 0x08)) pFilter->Bank.List16.wID[3] = (uint16_t)dFilterID;
            }
         } /* end if (pFoundFilter) */
      } /* end if (FILTER_MS_LIST_16_BIT == bModeScale) */


      if (FILTER_MS_LIST_32_BIT == bModeScale)
      {
         /*
          * Find the entry with the identifier
          */
         if      (pFilter->bInUseMask & 0x01)
         {
            if (pFilter->Bank.List32.dID[0] == dFilterID) 
            {
               pFoundFilter         = pFilter; 
               pFilter->bInUseMask &= ~0x01;
            }   
         }
         else if (pFilter->bInUseMask & 0x02)
         {
            if (pFilter->Bank.List32.dID[1] == dFilterID) 
            {
               pFoundFilter         = pFilter; 
               pFilter->bInUseMask &= ~0x02;
            }   
         }
         
         /* Check if the empty entries must be filled with an identifier */
         if (pFoundFilter)
         {  
            if (0 == pFilter->bInUseMask)
            {
               /* No used entries available */
               pFilter->bModeScale         = 0;
               pFilter->Bank.List32.dID[0] = 0;
               pFilter->Bank.List32.dID[1] = 0;
            }
            else
            {
               /* Used entries available, find a used identifier */
               if (pFilter->bInUseMask & 0x01) dFilterID = pFilter->Bank.List32.dID[0];
               if (pFilter->bInUseMask & 0x02) dFilterID = pFilter->Bank.List32.dID[1];
               
               /* Fill the rest free entries with the identifier too */
               if (0 == (pFilter->bInUseMask & 0x01)) pFilter->Bank.List32.dID[0] = dFilterID;
               if (0 == (pFilter->bInUseMask & 0x02)) pFilter->Bank.List32.dID[1] = dFilterID;
            }
         } /* end if (pFoundFilter) */
      } /* end if (FILTER_MS_LIST_32_BIT == bModeScale) */
      
   } /* end if (pFilter->bInUseMask != 0) */
   
   return(pFoundFilter);
} /* FindUseFilter */                                     

/*************************************************************************/
/*  FindFreeFilter                                                       */
/*                                                                       */
/*  Check if the filter bank has a free entry for this identifier.       */
/*                                                                       */
/*  In    : bModeScale, pFilter                                          */
/*  Out   : none                                                         */
/*  Return: NULL / pFilter                                               */
/*************************************************************************/
static FITLER_ENTRY *FindFreeFilter (uint8_t bModeScale, FITLER_ENTRY *pFilter)
{
   FITLER_ENTRY *pFoundFilter = NULL;   
   
   if (pFilter->bInUseMask == 0)
   {
      pFoundFilter = pFilter;   
   }
   else
   {
      switch (bModeScale)
      {
         case FILTER_MS_LIST_16_BIT:         
         {
            /* 0x0F = all 4 entries used */
            if (pFilter->bInUseMask != 0x0F)
            {
               pFoundFilter = pFilter;   
            }
            break;
         }
         
         case FILTER_MS_LIST_32_BIT:         
         {
            /* 0x03 = all 2 entries used */
            if (pFilter->bInUseMask != 0x03)
            {
               pFoundFilter = pFilter;   
            }
            break;
         }
      
         default:
         {
            /* Do nothing */
            break;
         }
      } /* end switch (bModeScale) */
   } /* end if (pFilter->bInUseMask == 0) */
   
   return(pFoundFilter);
} /* FindFreeFilter */

/*************************************************************************/
/*  CheckIDinBank                                                        */
/*                                                                       */
/*  Check if the identifier is in the filter bank.                       */
/*                                                                       */
/*  In    : bModeScale, dFilterID, pFilter                               */
/*  Out   : none                                                         */
/*  Return: TAL_FALSE / TAL_TRUE                                         */
/*************************************************************************/
static uint8_t CheckIDinBank (uint8_t       bModeScale, 
                              uint32_t      dFilterID,
                              FITLER_ENTRY *pFilter)
{
   uint8_t bFoundID = TAL_FALSE;
   
   if (pFilter->bInUseMask != 0)
   {
      switch (bModeScale)
      {
         case FILTER_MS_LIST_16_BIT:         
         {
            if ( (pFilter->Bank.List16.wID[0] == (uint16_t)dFilterID) ||
                 (pFilter->Bank.List16.wID[1] == (uint16_t)dFilterID) ||
                 (pFilter->Bank.List16.wID[2] == (uint16_t)dFilterID) ||
                 (pFilter->Bank.List16.wID[3] == (uint16_t)dFilterID) )
            {
               bFoundID = TAL_TRUE;
            }     
            break;
         }
         
         case FILTER_MS_LIST_32_BIT:         
         {
            if ( (pFilter->Bank.List32.dID[0] == dFilterID) ||
                 (pFilter->Bank.List32.dID[1] == dFilterID) )
            {
               bFoundID = TAL_TRUE;
            }     
            break;
         }
      
         default:
         {
            /* Do nothing */
            break;
         }
      } /* end switch (bModeScale) */
   } /* end if (pFilter->bInUseMask != 0) */

   return(bFoundID);
} /* CheckIDinBank */

/*************************************************************************/
/*  FilterBankUpdate                                                     */
/*                                                                       */
/*  Update the filter bank.                                              */
/*                                                                       */
/*  In    : pEntry                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void FilterBankUpdate (FITLER_ENTRY *pEntry)
{
   uint32_t bBankMask;
   uint8_t  bBank = pEntry->bBankIndex;
   
   /* Check valid bank range */
   if (bBank < CAN_MAX_FILTER_BANKS)
   {
      /* Request Initialization mode */
      CAN1->FMR = CAN_FMR_FINIT;
   
      /* Create the mask for the bank */
      bBankMask = (1u << bBank);
      
      /* Disable filter bank */   
      CAN1->FA1R &= ~bBankMask;
      
      switch (pEntry->bModeScale)
      {
         case FILTER_MS_MASK_16_BIT:
         {
            CAN1->FM1R &= ~bBankMask;
            CAN1->FS1R &= ~bBankMask;
            break;
         }
         case FILTER_MS_MASK_32_BIT:         
         {
            CAN1->FM1R &= ~bBankMask;
            CAN1->FS1R |=  bBankMask;
            break;
         }
         case FILTER_MS_LIST_16_BIT:         
         {
            CAN1->FM1R |=  bBankMask;
            CAN1->FS1R &= ~bBankMask;
            break;
         }
         case FILTER_MS_LIST_32_BIT:         
         {
            CAN1->FM1R |= bBankMask;
            CAN1->FS1R |= bBankMask;
            break;
         }
         default:
         {
            /* Do nothing */
            break;
         }
      } /* end switch (pEntry->bModeScale */
      
      /* Set filter Data */
      CAN1->sFilterRegister[bBank].FR1 = pEntry->Bank.Raw.wData1;
      CAN1->sFilterRegister[bBank].FR2 = pEntry->Bank.Raw.wData2;

      /* Check if the filter must be enabled */
      if (pEntry->bInUseMask != 0)
      {
         CAN1->FA1R |= bBankMask;
      }
      
      /* Leave Initialization mode */
      CAN1->FMR &= ~CAN_FMR_FINIT;
   } /* if (bBank < CAN_MAX_FILTER_BANKS) */
} /* FilterBankUpdate */ 

/*************************************************************************/
/*  FilterClear                                                          */
/*                                                                       */
/*  Clear the filter setup for this port.                                */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void FilterClear (TAL_CAN_DCB *pDCB)
{
   uint8_t bIndex;

   /* Disable filters */
   CAN1->FA1R &= ~(pDCB->HW.dFilterMask);

   /* Clear filter information */   
   memset(&pDCB->HW.FilterArray[0], 0x00, sizeof(pDCB->HW.FilterArray));
   
   /* Setup bank and ID information */
   for (bIndex = 0; bIndex < CAN_MAX_FILTER_BANKS; bIndex++)
   {
      pDCB->HW.FilterArray[bIndex].bBankIndex = bIndex + pDCB->HW.bFilterBankStart;
      
      FilterBankUpdate(&pDCB->HW.FilterArray[bIndex]);
   }
   
} /* FilterClear */

/*************************************************************************/
/*  FilterSetup                                                          */
/*                                                                       */
/*  Init the Acceptance filters.                                         */
/*                                                                       */
/*  The STM32F4xx supports 28 filter banks shared between CAN1 and CAN2. */
/*  Here the banks will be divided between CAN1 and CAN2.                */
/*                                                                       */
/*  CAN1:  0...13, FIFO 0                                                */
/*  CAN2: 14...27, FIFO 1                                                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void FilterSetup (void)
{
   uint8_t bIndex;
   
   /* Request Initialization mode */
   CAN1->FMR = CAN_FMR_FINIT;
   
   /* Set CAN2 start bank */ 
   CAN1->FMR |= (CAN2_FILTER_BANK_START << 8);
   
   /* Make the filter FIFO assignment */
   CAN1->FFA1R = FILTER_1_MASK;
   
   /* Disable all filters */
   CAN1->FA1R = 0;
   
   /* Clear all bank registers */
   for (bIndex = 0; bIndex < CAN_MAX_FILTER_BANKS; bIndex++)
   {
      CAN1->sFilterRegister[bIndex].FR1 = 0;
      CAN1->sFilterRegister[bIndex].FR2 = 0;
   }
   
   /* Leave Initialization mode */
   CAN1->FMR &= ~CAN_FMR_FINIT;
} /* FilterSetup */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  CANx_IRQHandler                                                      */
/*                                                                       */
/*  This is the Cortex CANx IRQ handler.                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void CAN1_TX_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   TX_IRQHandler(DCBArray[TAL_CAN_PORT_1]);
   TAL_CPU_IRQ_EXIT();
} /* CAN1_TX_IRQHandler */

void CAN1_RX0_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   RX_IRQHandler(DCBArray[TAL_CAN_PORT_1]);
   TAL_CPU_IRQ_EXIT();
} /* CAN1_RX0_IRQHandler */

void CAN1_SCE_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   SCE_IRQHandler(DCBArray[TAL_CAN_PORT_1]);
   TAL_CPU_IRQ_EXIT();
} /* CAN1_SCE_IRQHandler */

/**********************************************/

void CAN2_TX_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   TX_IRQHandler(DCBArray[TAL_CAN_PORT_2]);
   TAL_CPU_IRQ_EXIT();
} /* CAN2_TX_IRQHandler */

void CAN2_RX1_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   RX_IRQHandler(DCBArray[TAL_CAN_PORT_2]);
   TAL_CPU_IRQ_EXIT();
} /* CAN2_RX1_IRQHandler */

void CAN2_SCE_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   SCE_IRQHandler(DCBArray[TAL_CAN_PORT_2]);
   TAL_CPU_IRQ_EXIT();
} /* CAN2_SCE_IRQHandler */

/*************************************************************************/
/*  cpu_CANInit                                                          */
/*                                                                       */
/*  Prepare the hardware for use by the Open function later. Set the HW  */
/*  information depending of ePort and "enable" the CAN port.            */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANInit (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_ERR_CAN_PORT_RANGE;
   TAL_CAN_HW  *pHW    = &pDCB->HW;
      
   switch (pDCB->ePort)
   {
      case TAL_CAN_PORT_1:
      {
         Error = tal_BoardEnableCAN1();
         if (TAL_OK == Error)
         {
            DCBArray[TAL_CAN_PORT_1] = pDCB;

            pHW->dBaseAddress     = CAN1_BASE;
            pHW->dFilterMask      = FILTER_0_MASK;
            pHW->bFilterBankStart = CAN1_FILTER_BANK_START;
            pHW->bFilterBankEnd   = CAN1_FILTER_BANK_END;
            pHW->bFilterBankCount = CAN1_FILTER_BANK_COUNT;
            pHW->bFIFONumber      = 0; 
            pHW->pRFxR            = &CAN1->RF0R;
            pHW->bRxIntEnFifoFlag = CAN_IER_FMPIE0;
            pHW->TXIrqNumber      = CAN1_TX_IRQn; 
            pHW->RXIrqNumber      = CAN1_RX0_IRQn;
            pHW->SCEIrqNumber     = CAN1_SCE_IRQn;
            pHW->bTXIrqPriority   = 15; /* FIXME */
            pHW->bRXIrqPriority   = 15; /* FIXME */
            pHW->bSCEIrqPriority  = 15; /* FIXME */

            /* CAN1 Periph clock enable */
            __HAL_RCC_CAN1_CLK_ENABLE();
         }
         break;
      } /* TAL_CAN_PORT_1 */
   
      case TAL_CAN_PORT_2:
      {
         Error = tal_BoardEnableCAN2();
         if (TAL_OK == Error)
         {
            DCBArray[TAL_CAN_PORT_2] = pDCB;

            pHW->dBaseAddress     = CAN2_BASE;
            pHW->dFilterMask      = FILTER_0_MASK;            
            pHW->bFilterBankStart = CAN2_FILTER_BANK_START;
            pHW->bFilterBankEnd   = CAN2_FILTER_BANK_END;
            pHW->bFilterBankCount = CAN2_FILTER_BANK_COUNT;
            pHW->bFIFONumber      = 1; 
            pHW->pRFxR            = &CAN2->RF1R;
            pHW->bRxIntEnFifoFlag = CAN_IER_FMPIE1;
            pHW->TXIrqNumber      = CAN2_TX_IRQn; 
            pHW->RXIrqNumber      = CAN2_RX1_IRQn;
            pHW->SCEIrqNumber     = CAN2_SCE_IRQn;
            pHW->bTXIrqPriority   = 15; /* FIXME */
            pHW->bRXIrqPriority   = 15; /* FIXME */
            pHW->bSCEIrqPriority  = 15; /* FIXME */

            /* CAN2 Periph clock enable */
            __HAL_RCC_CAN2_CLK_ENABLE();
         }
         break;
      } /* TAL_CAN_PORT_2 */
      
      default:
      {
         /* Do nothing */
         Error = TAL_ERR_CAN_PORT_RANGE;
         break;
      }
   } /* end switch (pDCB->ePort) */
   
   return(Error);
} /* cpu_CANInit */

/*************************************************************************/
/*  cpu_CANOpen                                                          */
/*                                                                       */
/*  Open the CAN port.                                                   */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANOpen (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_ERROR;
   CAN_TypeDef *pCANx;
   uint32_t     dWaitAck;
   uint32_t     dBTR = 0;
   
   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;
         
   /* 
    * Check baud rate 
    */
   if (48000000L == tal_CPUGetFrequencyAPB1())
   {
      
      switch (pDCB->Settings.dBitRate)
      {
         /*lint -save -e648 -e778 -e831 -e845 */
         case TAL_CAN_BR_1000K: dBTR = BTR_1000K_48MHZ; break;
         case TAL_CAN_BR_500K:  dBTR = BTR_500K_48MHZ;  break;
         case TAL_CAN_BR_250K:  dBTR = BTR_250K_48MHZ;  break;
         case TAL_CAN_BR_125K:  dBTR = BTR_125K_48MHZ;  break;
         case TAL_CAN_BR_100K:  dBTR = BTR_100K_48MHZ;  break;
         case TAL_CAN_BR_50K:   dBTR = BTR_50K_48MHZ;   break;
         case TAL_CAN_BR_20K:   dBTR = BTR_20K_48MHZ;   break;
         case TAL_CAN_BR_10K:   dBTR = BTR_10K_48MHZ;   break;
         case TAL_CAN_BR_5K:    dBTR = BTR_5K_48MHZ;    break;
         /*lint -restore */
         
         default:
         {
            Error = TAL_ERR_CAN_BAUDRATE;
            goto CANOpenEnd;  /*lint !e801*/
            break;   /*lint !e527*/
         }
      } /* end switch (pDCB->Settings.dBaudrate) */
   }
   else
   {
      Error = TAL_ERR_CAN_BAUDRATE;
      goto CANOpenEnd;  /*lint !e801*/
   }
   
   /* Check if we must init the filter in general */
   if (TAL_FALSE == bFilterSetupDone)
   {
      FilterSetup();
      bFilterSetupDone = TAL_TRUE;
   }            

   FilterClear(pDCB);

   /* Exit Sleep mode. */
   pCANx->MCR &= ~CAN_MCR_SLEEP;
   
   /* Initialization request */
   pCANx->MCR |= CAN_MCR_INRQ;
   
   /* Wait the acknowledge */
   dWaitAck = 0;
   while (((pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (dWaitAck != INAK_TIMEOUT))
   {
      dWaitAck++;
   }
   
   /* Check acknowledge */
   if ((pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
   {
      Error = TAL_ERR_CAN_INTERN_ACK;
      goto CANOpenEnd;  /*lint !e801*/
   }

   /* 
    * TTCM: Time triggered communication mode 
    *
    * 0: Time Triggered Communication mode disabled.
    */
   pCANx->MCR &= ~CAN_MCR_TTCM;
   
   /* 
    * ABOM: Automatic bus-off management 
    *
    * 1: The Bus-Off state is left automatically by hardware once 128 
    *    occurrences of 11 recessive bits have been monitored.
    */
   pCANx->MCR |= CAN_MCR_ABOM;

   /*
    * AWUM: Automatic wakeup mode 
    *
    * 1: The Sleep mode is left automatically by hardware on CAN message
    *    detection. The SLEEP bit of the CAN_MCR register and the SLAK bit
    *    of the CAN_MSR register are cleared by hardware.
    */
   pCANx->MCR |= CAN_MCR_AWUM;
   
   /*
    * NART: No automatic retransmission
    *
    * 0: The CAN hardware will automatically retransmit the message until
    *    it has been successfully transmitted according to the CAN standard.
    */ 
   pCANx->MCR &= ~CAN_MCR_NART;
   
   /*
    * RFLM: Receive FIFO locked mode
    *
    * 0: Receive FIFO not locked on overrun. Once a receive FIFO is full
    *    the next incoming message will overwrite the previous one.
    */
   pCANx->MCR &= ~CAN_MCR_RFLM;
   
   /*
    * TXFP: Transmit FIFO priority
    *
    * 1: Priority driven by the request order (chronologically)
    */
   pCANx->MCR |= CAN_MCR_TXFP;
   
   /* Set CAN bit timing register */
   pCANx->BTR = dBTR;
   
   /* Request leave initialisation */
   pCANx->MCR &= ~CAN_MCR_INRQ;

   /* Wait the acknowledge */
   dWaitAck = 0;
   while (((pCANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (dWaitAck != INAK_TIMEOUT))
   {
     dWaitAck++;
   }
   
   /* Check acknowledged */
   if ((pCANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
   {
      Error = TAL_ERR_CAN_INTERN_ACK;
      goto CANOpenEnd;  /*lint !e801*/
   }
   
   /* Enable RX interrrupt */
   pCANx->IER |= pDCB->HW.bRxIntEnFifoFlag;

   /* Set priority and enable the RX IRQ */
   tal_CPUIrqSetPriority(pDCB->HW.RXIrqNumber, pDCB->HW.bRXIrqPriority);
   tal_CPUIrqEnable(pDCB->HW.RXIrqNumber);
   
   /* Set priority and enable the TX IRQ */
   tal_CPUIrqSetPriority(pDCB->HW.TXIrqNumber, pDCB->HW.bTXIrqPriority);
   tal_CPUIrqEnable(pDCB->HW.TXIrqNumber);
   
   /* TODO, SCE interrupt ? */
   
   Error = TAL_OK;
   
CANOpenEnd:   
   return(Error);
} /* cpu_CANOpen */

/*************************************************************************/
/*  cpu_CANClose                                                         */
/*                                                                       */
/*  Close the CAN port.                                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANClose (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_OK;
   CAN_TypeDef *pCANx;
   
   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;

   /* Disable the IRQ */   
   tal_CPUIrqDisable(pDCB->HW.TXIrqNumber); 
   tal_CPUIrqDisable(pDCB->HW.RXIrqNumber);
   tal_CPUIrqDisable(pDCB->HW.SCEIrqNumber);

   /* Disable RX interrrupt */
   pCANx->IER &= ~pDCB->HW.bRxIntEnFifoFlag;

   /* Disable TX interrupt */
   pCANx->IER &= ~CAN_IER_TMEIE;
   
   /* Disable filters */
   FilterClear(pDCB);
      
   return(Error);
} /* cpu_CANClose */

/*************************************************************************/
/*  cpu_CANStartTx                                                       */
/*                                                                       */
/*  Send the data from the ring buffer if the TX interrupt is disabled.  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANStartTx (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT       Error = TAL_ERROR;
   CAN_TypeDef    *pCANx;
   TAL_CAN_OBJECT   CANObject;
   
   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;

   TAL_CPU_DISABLE_ALL_INTS();
   if (pCANx->IER & CAN_IER_TMEIE)
   {
      /* TX interrupt is enabled, do nothing */
   }
   else
   {
      Error = tal_MISCRingGet(&pDCB->TxRing, (uint8_t*)&CANObject);
      if (TAL_OK == Error)
      {
         Error = SendObject(pCANx, &CANObject);
         if (TAL_OK == Error)
         {
            /* Enable TX interrupt */
            pCANx->IER |= CAN_IER_TMEIE;
         }
      }
   }
   TAL_CPU_ENABLE_ALL_INTS();
   
   return(Error);
} /* cpu_CANStartTx */

/*************************************************************************/
/*  cpu_CANTxIsRunning                                                   */
/*                                                                       */
/*  Check if TX is still running.                                        */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT cpu_CANTxIsRunning (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_OK;
   CAN_TypeDef *pCANx;
   
   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;

   TAL_CPU_DISABLE_ALL_INTS();
   if (pCANx->IER & CAN_IER_TMEIE)
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
} /* cpu_CANTxIsRunning */

/*************************************************************************/
/*  cpu_CANEnableSilentMode                                              */
/*                                                                       */
/*  Enable "Silent" mode.                                                */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANEnableSilentMode (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_ERROR;
   uint32_t     dWaitAck;
   CAN_TypeDef *pCANx;
   
   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;

   /* Initialization request */
   pCANx->MCR |= CAN_MCR_INRQ;

   /* Wait the acknowledge */
   dWaitAck = 0;
   while (((pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (dWaitAck != INAK_TIMEOUT))
   {
      dWaitAck++;
   }
   
   /* Check acknowledge */
   if ((pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
   {
      Error = TAL_ERR_CAN_INTERN_ACK;
      goto EnableSilentModeEnd;  /*lint !e801*/
   }

   /* 
    * Set Silent mode. 
    */
   pCANx->BTR |= CAN_BTR_SILM;

   /* Request leave initialisation */
   pCANx->MCR &= ~CAN_MCR_INRQ;

   /* Wait the acknowledge */
   dWaitAck = 0;
   while (((pCANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (dWaitAck != INAK_TIMEOUT))
   {
     dWaitAck++;
   }
   
   /* Check acknowledged */
   if ((pCANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
   {
      Error = TAL_ERR_CAN_INTERN_ACK;
      goto EnableSilentModeEnd;  /*lint !e801*/
   }
   
   Error = TAL_OK;
      
EnableSilentModeEnd:      
   return(Error);
} /* cpu_CANEnableSilentMode */

/*************************************************************************/
/*  cpu_CANDisableSilentMode                                             */
/*                                                                       */
/*  Disable "Silent" mode.                                               */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANDisableSilentMode (TAL_CAN_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_ERROR;
   uint32_t     dWaitAck;
   CAN_TypeDef *pCANx;
   
   /* Get CAN pointer */
   pCANx = (CAN_TypeDef *)pDCB->HW.dBaseAddress;

   /* Initialization request */
   pCANx->MCR |= CAN_MCR_INRQ;

   /* Wait the acknowledge */
   dWaitAck = 0;
   while (((pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (dWaitAck != INAK_TIMEOUT))
   {
      dWaitAck++;
   }
   
   /* Check acknowledge */
   if ((pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
   {
      Error = TAL_ERR_CAN_INTERN_ACK;
      goto DisableSilentModeEnd; /*lint !e801*/
   }

   /* 
    * Remove Silent mode. 
    */
   pCANx->BTR &= ~CAN_BTR_SILM;

   /* Request leave initialisation */
   pCANx->MCR &= ~CAN_MCR_INRQ;

   /* Wait the acknowledge */
   dWaitAck = 0;
   while (((pCANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (dWaitAck != INAK_TIMEOUT))
   {
     dWaitAck++;
   }
   
   /* Check acknowledged */
   if ((pCANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
   {
      Error = TAL_ERR_CAN_INTERN_ACK;
      goto DisableSilentModeEnd; /*lint !e801*/
   }
   
   Error = TAL_OK;
      
DisableSilentModeEnd:      
   return(Error);
} /* cpu_CANDisableSilentMode */

/*************************************************************************/
/*  cpu_CANIdentRegister                                                 */
/*                                                                       */
/*  Register a CAN identifier.                                           */
/*                                                                       */
/*  In    : pDCB, dIdentifier, Type                                      */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANIdentRegister (TAL_CAN_DCB   *pDCB,
                                uint32_t       dIdentifier,
                                TAL_CAN_TYPE_ID Type)
{
   TAL_RESULT       Error = TAL_ERROR;     
   FITLER_ENTRY   *pFilterArray;   
   FITLER_ENTRY   *pFilter = NULL;
   uint8_t         bIndex;
   uint8_t         bBankCount;
   uint8_t         bModeScale;
   uint8_t         bFoundID;
   uint32_t        dFilterID;

   pFilterArray = pDCB->HW.FilterArray;
   bBankCount   = pDCB->HW.bFilterBankCount;
   
   /*
    * Create Mode and Scale depending of Identifier and Type.
    */
  
   /* 
    * Handle special case TAL_CAN_ID_RX_ALL, all identifier
    * from type STD or EXT should be received. A 32 Bit Mask
    * Mode is needed.
    */
   if (TAL_CAN_ID_RX_ALL == dIdentifier)
   {
      /* Test if STD or EXT ID */
      if (TAL_CAN_TYPE_STD_ID == Type)
      {
         pFilterArray[0].bInUseMask        = 3;
         pFilterArray[0].bModeScale        = FILTER_MS_MASK_32_BIT;
         pFilterArray[0].Bank.Mask32.dID   = 0;
         pFilterArray[0].Bank.Mask32.dMask = CAN_RI0R_IDE;
         
         FilterBankUpdate(&pFilterArray[0]);
      }
      else
      {
         pFilterArray[1].bInUseMask        = 3;
         pFilterArray[1].bModeScale        = FILTER_MS_MASK_32_BIT;
         pFilterArray[1].Bank.Mask32.dID   = CAN_RI0R_IDE;
         pFilterArray[1].Bank.Mask32.dMask = CAN_RI0R_IDE;
         
         FilterBankUpdate(&pFilterArray[1]);
      }   
      
      Error = TAL_OK;
      goto IdentRegisterEnd;  /*lint !e801*/
   } /* end if (TAL_CAN_ID_RX_ALL == dIdentifier) */
      
   /*
    * At this point the TAL_CAN_ID_RX_ALL identifier
    * was handled. Now List Mode, 16 or 32 Bit is needed.
    */
    
   if (TAL_CAN_TYPE_STD_ID == Type)
   {
      /* Check valid range */
      if (dIdentifier > 0x7FF)
      {
         Error = TAL_ERR_CAN_ID_RANGE;
         goto IdentRegisterEnd;  /*lint !e801*/
      }
      
      bModeScale = FILTER_MS_LIST_16_BIT;
      dFilterID  = (dIdentifier << 5);
   }
   else
   {
      /* Check valid range */
      if (dIdentifier > 0X1FFFFFFF)
      {
         Error = TAL_ERR_CAN_ID_RANGE;
         goto IdentRegisterEnd;  /*lint !e801*/
      }
   
      bModeScale = FILTER_MS_LIST_32_BIT;
      dFilterID  = (dIdentifier << 3) | CAN_RI0R_IDE; 
   }
   
   /*
    * Check if this identifier is registered
    */
   for (bIndex = 0; bIndex < bBankCount; bIndex++)
   {
      bFoundID = CheckIDinBank(bModeScale, dFilterID, &pFilterArray[bIndex]); 
      if (TAL_TRUE == bFoundID)
      {
         Error = TAL_ERR_CAN_ID_USED;
         goto IdentRegisterEnd;  /*lint !e801*/
      }
   }
   
   /*
    * At this point the identifier is not registered.
    * Find a free bank which can be used.
    */    
   for (bIndex = 0; bIndex < bBankCount; bIndex++)
   {
      pFilter = FindFreeFilter(bModeScale, &pFilterArray[bIndex]); 
      if (pFilter)
      {
         /* Found a filter which can be used */
         break;
      }
   }
   if (NULL == pFilter)
   {
      Error = TAL_ERR_CAN_ID_NO_ENTRY;
      goto IdentRegisterEnd;  /*lint !e801*/
   }
   
   /*
    * At this point pFilter can be used for the identifier
    */
   AddIDtoBank(bModeScale, dFilterID, pFilter);    

   /*
    * Update the filter hardware now
    */    
   FilterBankUpdate(pFilter);
   
   Error = TAL_OK;

IdentRegisterEnd:   
   return(Error);
} /* cpu_CANIdentRegister */   

/*************************************************************************/
/*  cpu_CANIdentDeRegister                                               */
/*                                                                       */
/*  DeRegister a CAN identifier.                                         */
/*                                                                       */
/*  In    : pDCB, dIdentifier, Type                                      */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_CANIdentDeRegister (TAL_CAN_DCB   *pDCB,
                                  uint32_t       dIdentifier,
                                  TAL_CAN_TYPE_ID Type)
{
   TAL_RESULT       Error = TAL_ERROR;     
   FITLER_ENTRY   *pFilterArray;   
   FITLER_ENTRY   *pFilter = NULL;
   uint8_t         bIndex;
   uint8_t         bBankCount;
   uint8_t         bModeScale;
   uint8_t         bFoundID = TAL_FALSE;
   uint32_t        dFilterID;

   pFilterArray = pDCB->HW.FilterArray;
   bBankCount   = pDCB->HW.bFilterBankCount;
   
   /*
    * Create Mode and Scale depending of Identifier and Type.
    */
  
   /* 
    * Handle special case TAL_CAN_ID_RX_ALL, all identifier
    * from type STD or EXT should be received. A 32 Bit Mask
    * Mode is needed.
    */
   if (TAL_CAN_ID_RX_ALL == dIdentifier)
   {
      /* Test if STD or EXT ID */
      if (TAL_CAN_TYPE_STD_ID == Type)
      {
         pFilterArray[0].bInUseMask        = 0;
         pFilterArray[0].bModeScale        = 0;
         pFilterArray[0].Bank.Mask32.dID   = 0;
         pFilterArray[0].Bank.Mask32.dMask = 0;
         
         FilterBankUpdate(&pFilterArray[0]);
      }
      else
      {
         pFilterArray[1].bInUseMask        = 0;
         pFilterArray[1].bModeScale        = 0;
         pFilterArray[1].Bank.Mask32.dID   = 0;
         pFilterArray[1].Bank.Mask32.dMask = 0;
         
         FilterBankUpdate(&pFilterArray[1]);
      }   
      
      Error = TAL_OK;
      goto IdentDeRegisterEnd;   /*lint !e801*/
   } /* end if (TAL_CAN_ID_RX_ALL == dIdentifier) */
   

   /*
    * At this point the TAL_CAN_ID_RX_ALL identifier
    * is handled. Now List Mode, 16 or 32 Bit is needed.
    */
    
   if (TAL_CAN_TYPE_STD_ID == Type)
   {
      /* Check valid range */
      if (dIdentifier > 0x7FF)
      {
         Error = TAL_ERR_CAN_ID_RANGE;
         goto IdentDeRegisterEnd;   /*lint !e801*/
      }
      
      bModeScale = FILTER_MS_LIST_16_BIT;
      dFilterID  = (dIdentifier << 5);
   }
   else
   {
      /* Check valid range */
      if (dIdentifier > 0X1FFFFFFF)
      {
         Error = TAL_ERR_CAN_ID_RANGE;
         goto IdentDeRegisterEnd;   /*lint !e801*/
      }
   
      bModeScale = FILTER_MS_LIST_32_BIT;
      dFilterID  = (dIdentifier << 3) | CAN_RI0R_IDE; 
   }
   
   /*
    * Check if this identifier is registered
    */
   for (bIndex = 0; bIndex < bBankCount; bIndex++)
   {
      bFoundID = CheckIDinBank(bModeScale, dFilterID, &pFilterArray[bIndex]); 
      if (TAL_TRUE == bFoundID)
      {
         break;
      }
   }
   if (TAL_FALSE == bFoundID)
   {
      Error = TAL_ERR_CAN_ID_NOT_USED;
      goto IdentDeRegisterEnd;   /*lint !e801*/
   }
   
   /*
    * At this point the identifier is registered.
    * Find the bank which is used and remove the identifier.
    */    
   for (bIndex = 0; bIndex < bBankCount; bIndex++)
   {
      pFilter = FindUseFilterAndRemove(bModeScale, dFilterID, &pFilterArray[bIndex]); 
      if (pFilter)
      {
         break;
      }
   }
   if (NULL == pFilter)
   {
      Error = TAL_ERR_CAN_ID_NO_ENTRY;
      goto IdentDeRegisterEnd;   /*lint !e801*/
   }

   /*
    * Update the filter hardware now
    */    
   FilterBankUpdate(pFilter);
   
   Error = TAL_OK;

IdentDeRegisterEnd:   
   return(Error);
} /* cpu_CANIdentDeRegister */   

/*** EOF ***/
