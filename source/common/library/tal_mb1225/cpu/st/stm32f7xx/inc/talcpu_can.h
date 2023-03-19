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
#if !defined(__TALCPU_CAN_H__)
#define __TALCPU_CAN_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>

/**************************************************************************
*  Global Definitions
**************************************************************************/

/*
 * The STM32F407 CPU supports up to 2 CAN ports
 * and the STM32F103ZE only 1 port.
 */
typedef enum _tal_can_port_
{
   TAL_CAN_PORT_1 = 0,  /* Must be start at 0 */
   TAL_CAN_PORT_2,

   TAL_CAN_PORT_MAX
} TAL_CAN_PORT; 

/*
 * And 28 filter banks
 */
#define CAN_MAX_FILTER_BANKS  28

typedef struct _fb_mask_32_
{
   uint32_t dID;
   uint32_t dMask;
} FB_MASK_32;

typedef struct _fb_list_32_
{
   uint32_t dID[2];
} FB_LIST_32;

typedef struct _fb_mask_16_
{
   uint16_t wID1;
   uint16_t wMask1;
   uint16_t wID2;
   uint16_t wMask2;
} FB_MASK_16;

typedef struct _fb_list_16_
{
   uint16_t wID[4];
} FB_LIST_16;

typedef struct _fb_raw_
{
   uint32_t wData1;
   uint32_t wData2;
} FB_RAW;

typedef union _filter_bank_
{
   FB_RAW     Raw;
   FB_MASK_32 Mask32;
   FB_LIST_32 List32;
   FB_MASK_16 Mask16;
   FB_LIST_16 List16;   
} FILTER_BANK;

typedef struct _filter_entry_
{
   uint8_t    bInUseMask; /* Bit coded info if a entry is used */
   uint8_t    bBankIndex;
   uint8_t    bModeScale;
   FILTER_BANK Bank;
} FITLER_ENTRY;


/*
 * Hardware information
 */
typedef struct _tal_can_hw_
{
   uint32_t           dBaseAddress;
   uint32_t           dFilterMask;
   uint8_t            bFilterBankStart;
   uint8_t            bFilterBankEnd;
   uint8_t            bFilterBankCount;
   uint8_t            bFIFONumber;
   volatile uint32_t *pRFxR;
   uint8_t            bRxIntEnFifoFlag;
   IRQn_Type           TXIrqNumber;
   IRQn_Type           RXIrqNumber;
   IRQn_Type           SCEIrqNumber;   
   uint8_t            bTXIrqPriority;
   uint8_t            bRXIrqPriority;
   uint8_t            bSCEIrqPriority;
   FITLER_ENTRY FilterArray[CAN_MAX_FILTER_BANKS];
} TAL_CAN_HW;

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

/*
 * Functions still defined in talcan.h
 */

#endif /* !__TALCPU_CAN_H__ */

/*** EOF ***/
