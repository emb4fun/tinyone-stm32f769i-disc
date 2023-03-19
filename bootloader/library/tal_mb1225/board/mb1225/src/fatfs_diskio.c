/**************************************************************************
*  Copyright (c) 2018-2022 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Based on an example from ST. Therefore partial copyright:
*  Copyright (c) 2017 STMicroelectronics
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

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tal.h"
#include "ff.h"
#include "diskio.h"

#include "stm32f769i_discovery_sd.h"

/*=======================================================================*/
/*  Extern                                                               */
/*=======================================================================*/

/* SD handler declared in "stm32746g_discovery_sd.c" file */
extern SD_HandleTypeDef uSdHandle;

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define BLOCK_SIZE            512

#define SD_TIMEOUT_MS         5000

#define READ_EVENT_READY      0x0001
#define READ_EVENT_WAIT_ALL   (READ_EVENT_READY)

#define WRITE_EVENT_READY     0x0001
#define WRITE_EVENT_WAIT_ALL  (WRITE_EVENT_READY)

#define SD_LED_ON()           tal_LEDSet(TAL_LED_CHANNEL_1)
#define SD_LED_OFF()          tal_LEDClear(TAL_LED_CHANNEL_1)

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static DSTATUS DiskStatus = STA_NOINIT;
static OS_SEMA FSSema;

static OS_EVENT ReadEvent;
static OS_EVENT WriteEvent;

static int BSP_SDInitDone = 0;

uint8_t CopyBuffer[16*1024] __attribute__((section(".dtcm")));

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  CheckStatusWithTimeout                                               */
/*                                                                       */
/*  Wait until SDIO is ready or timeout.                                 */
/*                                                                       */
/*  In    : timeout                                                      */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static int CheckStatusWithTimeout (uint32_t timeout)
{
   uint32_t timer = OS_TimeGet();
  
   while( OS_TimeGet() - timer < timeout)
   {
      if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
      {
         return(0);
      }
   }

   return(-1);
} /* CheckStatusWithTimeout */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  ff_memalloc                                                          */
/*                                                                       */
/*  Allocate a memory block                                              */
/*                                                                       */
/*  In    : msize                                                        */
/*  Out   : none                                                         */
/*  Return: Returns pointer to the allocated memory block                */
/*************************************************************************/
void *ff_memalloc (UINT msize)
{
   return(xmalloc(XM_ID_FS, msize));
} /* ff_memalloc */

/*************************************************************************/
/*  ff_memfree                                                           */
/*                                                                       */
/*  Free a memory block                                                  */
/*                                                                       */
/*  In    : mblock                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void ff_memfree (void *mblock)
{
   xfree(mblock);
} /* ff_memfree */

/*************************************************************************/
/*  ff_cre_syncobj                                                       */
/*                                                                       */
/*  Create a Synchronization Object                                      */
/*                                                                       */
/*  In    : vol,  corresponding logical drive being processed.           */
/*          sobj, pointer to return the created sync object.             */
/*  Out   : none                                                         */
/*  Return: 0 = Error / 1 = OK                                           */
/*************************************************************************/
int ff_cre_syncobj (BYTE vol, FF_SYNC_t *sobj)
{
   (void)vol;
   
   /* Init the semaphore */
   OS_SemaCreate(&FSSema, 1, 1);
   *sobj = &FSSema;

   return(1);
} /* ff_cre_syncobj */

/*************************************************************************/
/*  ff_del_syncobj                                                       */
/*                                                                       */
/*  Delete a Synchronization Object                                      */
/*                                                                       */
/*  In    : sobj, sync object tied to the logical drive to be deleted.   */
/*  Out   : none                                                         */
/*  Return: 0 = Error / 1 = OK                                           */
/*************************************************************************/
int ff_del_syncobj (FF_SYNC_t sobj)
{
   /* Reset the semaphore */
   OS_SemaDelete(sobj);
   
   return(1);
} /* ff_del_syncobj */

/*************************************************************************/
/*  ff_req_grant                                                         */
/*                                                                       */
/*  Request Grant to Access the Volume                                   */
/*                                                                       */
/*  In    : sobj, sync object to wait.                                   */
/*  Out   : none                                                         */
/*  Return: 1 = Got a grant / 0 =  Could not get a grant                 */
/*************************************************************************/
int ff_req_grant (FF_SYNC_t sobj)
{
   OS_SemaWait(sobj, OS_WAIT_INFINITE);
   
   return(1);
} /* ff_req_grant */

/*************************************************************************/
/*  ff_rel_grant                                                         */
/*                                                                       */
/*  Request Grant to Access the Volume                                   */
/*                                                                       */
/*  In    : sobj, Sync object to be signaled.                            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void ff_rel_grant (FF_SYNC_t sobj)
{
   OS_SemaSignal(sobj);
} /* ff_rel_grant */

/*************************************************************************/
/*  disk_initialize                                                      */
/*                                                                       */
/*  Initialize a Drive                                                   */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: Status of Disk Functions                                     */
/*************************************************************************/
DSTATUS disk_initialize (uint8_t pdrv)
{
   static int InitDone = FALSE;
   
   if (DiskStatus & STA_NODISK) 
   {
      /* No card in the socket */
      return(DiskStatus);
   } 
   
   SD_LED_ON();

   if (0 == pdrv)
   {
      if (FALSE == InitDone)
      { 
         InitDone = TRUE;
         
         OS_EventCreate(&ReadEvent);
         OS_EventCreate(&WriteEvent);
      }   
   
      DiskStatus |= STA_NOINIT;
      
      /* Init BSP_SD only if it not done before */
      if (0 == BSP_SDInitDone)
      {
         if(BSP_SD_Init() == MSD_OK)
         {
            BSP_SDInitDone = 1;
            DiskStatus  &= ~STA_NOINIT;
         }
      }
      else
      {
         DiskStatus  &= ~STA_NOINIT;
      }
   }   

   SD_LED_OFF();

   return(DiskStatus);
} /* disk_initialize */

/*************************************************************************/
/*  disk_removed                                                         */
/*                                                                       */
/*  Medium was removed                                                   */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void disk_removed (BYTE pdrv)
{
   if (0 == pdrv)
   {
      BSP_SD_DeInit();
      
      BSP_SDInitDone = 0;
      DiskStatus    |= STA_NODISK;
   }
         
} /* disk_removed */

/*************************************************************************/
/*  disk_status                                                          */
/*                                                                       */
/*  Get Drive Status                                                     */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: Status of Disk Functions                                     */
/*************************************************************************/
DSTATUS disk_status (uint8_t pdrv)
{
   if (0 == pdrv)
   {
      DiskStatus |= STA_NODISK;
      if (SD_PRESENT == BSP_SD_IsDetected())
      {
         DiskStatus &= ~STA_NODISK;
      }
   }
   else
   {
      return(STA_NOINIT);
   } 

   return(DiskStatus);
} /* disk_status */

/*************************************************************************/
/*  disk_read                                                            */
/*                                                                       */
/*  Read Sector(s)                                                       */
/*                                                                       */
/*  In    : pdrv,   physical drive nmuber to identify the drive          */
/*          buff,   data buffer to store read data                       */
/*          sector, sector address in LBA                                */
/*          count,  number of sectors to read                            */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
   DRESULT  Result = RES_ERROR;
   uint32_t AlignedAddr;
   uint32_t Event;
   int      rc;
 
   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }  
      
      SD_LED_ON();
      
      /* Check if SDIO is ready */
      if (CheckStatusWithTimeout(SD_TIMEOUT_MS) < 0)
      {
         SD_LED_OFF();
         return(RES_ERROR);
      }

#if !defined(__SDRAM__)
      if (BSP_SD_ReadBlocks_DMA((uint32_t*)buff, sector, count) == MSD_OK)   /*lint !e826*/
      {
         /* Wait for the event or timeout */
         rc = OS_EventWait(&ReadEvent, READ_EVENT_WAIT_ALL, OS_EVENT_MODE_OR, &Event, SD_TIMEOUT_MS); 
         if ((OS_RC_OK == rc) && (Event & READ_EVENT_READY))
         {
            if (SD_TRANSFER_OK == BSP_SD_GetCardState())
            {
               Result = RES_OK;  
                      
               /*
                * The SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
                * adjust the address and the D-Cache size to invalidate accordingly.
                */
               AlignedAddr = (uint32_t)buff & ~0x1F;
               SCB_InvalidateDCache_by_Addr((uint32_t*)AlignedAddr, (int32_t)((count * BLOCK_SIZE) + ((uint32_t)buff - AlignedAddr)));
            }
         }
      }
#else
      /*
       * Workaround in case of running from external SDRAM.
       * It looks that in this case the CPU (STM32F769) has problems with the DMA.
       */ 
      if (BSP_SD_ReadBlocks_DMA((uint32_t*)CopyBuffer, sector, count) == MSD_OK)   /*lint !e826*/
      {
         /* Wait for the event or timeout */
         rc = OS_EventWait(&ReadEvent, READ_EVENT_WAIT_ALL, OS_EVENT_MODE_OR, &Event, SD_TIMEOUT_MS); 
         if ((OS_RC_OK == rc) && (Event & READ_EVENT_READY))
         {
            if (SD_TRANSFER_OK == BSP_SD_GetCardState())
            {
               Result = RES_OK;  
                      
               /*
                * The SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
                * adjust the address and the D-Cache size to invalidate accordingly.
                */
               AlignedAddr = (uint32_t)CopyBuffer & ~0x1F;
               SCB_InvalidateDCache_by_Addr((uint32_t*)AlignedAddr, (int32_t)((count * BLOCK_SIZE) + ((uint32_t)CopyBuffer - AlignedAddr)));
               memcpy(buff, CopyBuffer, (count*BLOCK_SIZE)); 
            }
         }
      }
#endif      
         
      SD_LED_OFF();
   }

   return(Result);
} /* disk_read */

/*************************************************************************/
/*  disk_write                                                           */
/*                                                                       */
/*  Write Sector(s)                                                      */
/*                                                                       */
/*  In    : pdrv,   physical drive nmuber to identify the drive          */
/*          buff,   data to be written                                   */
/*          sector, sector address in LBA                                */
/*          count,  number of sectors to write                           */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
   DRESULT  Result = RES_ERROR;
   uint32_t AlignedAddr;
   uint32_t Event;
   int      rc;
   uint32_t timer;
 
   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }
      
      SD_LED_ON();


#if defined(__SDRAM__)
      /*
       * Workaround in case of running from external SDRAM.
       * It looks that in this case the CPU (STM32F769) has problems with the DMA.
       */ 
      if ((count*BLOCK_SIZE) < sizeof(CopyBuffer)) 
      {
         memcpy(CopyBuffer, buff, (count*512));
         buff = CopyBuffer;
      }         
#endif      


      /* Check if SDIO is ready */
      if (CheckStatusWithTimeout(SD_TIMEOUT_MS) < 0)
      {
         SD_LED_OFF();
         return(RES_ERROR);
      }

      /*
       * The SCB_CleanDCache_by_Addr() requires a 32-Byte aligned address
       * adjust the address and the D-Cache size to clean accordingly.
       */
      AlignedAddr = (uint32_t)buff & ~0x1F;
      SCB_InvalidateDCache_by_Addr((uint32_t*)AlignedAddr, (int32_t)((count * BLOCK_SIZE) + ((uint32_t)buff - AlignedAddr)));

      if (BSP_SD_WriteBlocks_DMA((uint32_t*)buff, sector, count) == MSD_OK)   /*lint !e826*/
      {                               
         /* Wait for the event or timeout */
         rc = OS_EventWait(&WriteEvent, WRITE_EVENT_WAIT_ALL, OS_EVENT_MODE_OR, &Event, SD_TIMEOUT_MS); 
         if ((OS_RC_OK == rc) && (Event & WRITE_EVENT_READY))
         {
            /* Wait for the SDIO or timeout */
            timer = OS_TimeGet() + SD_TIMEOUT_MS;
            while (timer > OS_TimeGet())
            {
               if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
               {
                  Result = RES_OK;
                  break;
               }
            }
         }
      }
      
      SD_LED_OFF();
   }

   return(Result);
} /* disk_write */

/*************************************************************************/
/*  disk_ioctl                                                           */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*          cmd,  control code                                           */
/*          buff, buffer to send/receive control data                    */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_ioctl (uint8_t pdrv, uint8_t cmd, void *buff)
{
   DRESULT           Result = RES_PARERR;
   BSP_SD_CardInfo   CardInfo;
  
   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }
      
      SD_LED_ON();
      
      switch(cmd)
      {
         case CTRL_SYNC:         /* Make sure that no pending write process */
            Result = RES_OK;
            break;
            
         case GET_SECTOR_COUNT:  /* Get number of sectors on the disk (DWORD) */
            BSP_SD_GetCardInfo(&CardInfo);
            *(DWORD*)buff = CardInfo.LogBlockNbr;
            Result = RES_OK;
            break;

         case GET_SECTOR_SIZE:   /* Get R/W sector size (WORD) */
            *(WORD*)buff = BLOCK_SIZE;
            Result = RES_OK;
            break;
            
         case GET_BLOCK_SIZE:    /* Get erase block size in unit of sector (DWORD) */
            *(DWORD*)buff = BLOCK_SIZE;
            Result = RES_OK;
            break;

         default:
            Result = RES_PARERR;
            break;
      }
      
      SD_LED_OFF();
   }
   
   return(Result);
} /* disk_ioctl */

/*************************************************************************/
/*  get_fattime                                                          */
/*                                                                       */
/*  Gets Time from RTC                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Time                                                         */
/*************************************************************************/
DWORD get_fattime (void)
{
   return(0);
}

/******************************************************************************/
/*                 STM32F7xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f7xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles SDMMC1 global interrupt request.
  * @param  None
  * @retval None
  */
void BSP_SDMMC_IRQHandler (void)
{
   HAL_SD_IRQHandler(&uSdHandle);
}

/**
* @brief  This function handles DMA2 Stream 6 interrupt request.
* @param  None
* @retval None
*/
void BSP_SDMMC_DMA_Tx_IRQHandler(void)
{
   HAL_DMA_IRQHandler (uSdHandle.hdmatx);
}

/**
* @brief  This function handles DMA2 Stream 3 interrupt request.
* @param  None
* @retval None
*/
void BSP_SDMMC_DMA_Rx_IRQHandler(void)
{
   HAL_DMA_IRQHandler(uSdHandle.hdmarx);
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{
   OS_EventSetFromInt(&WriteEvent, WRITE_EVENT_READY);
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
   OS_EventSetFromInt(&ReadEvent, READ_EVENT_READY);
}

/*** EOF ***/
