/**************************************************************************
*  Copyright (c) 2018 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Based on an example from ST. Therefore partial copyright:
*  Copyright (c) 2016 STMicroelectronics

*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
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
*
***************************************************************************
*  History:
*
*  26.08.2018  mifi  First Version.
**************************************************************************/

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#define __ETHERNETIF_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <string.h>

#include "tcts.h"
#include "tal.h"

#include "ethernetif.h"
#include "project.h"

#include "lwip\tcpip.h"
#include "netif\etharp.h"
#include "lwip\stats.h"
#include "lwip\igmp.h"

#include "arch/sys_arch.h"


/* Forward declarations. */
static void ethernetif_input (void *arg);

/*=======================================================================*/
/*  Global                                                               */
/*=======================================================================*/

/* Global Ethernet handle*/
ETH_HandleTypeDef EthHandle;

/*=======================================================================*/
/*  Extern                                                               */
/*=======================================================================*/

void ETH_MACAddressConfig(ETH_HandleTypeDef *heth, uint32_t MacAddr, uint8_t *Addr);

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define MULTICAST_FILTER_SIZE 32

#define IFNAME0 'e'
#define IFNAME1 'n'

#define LINK_TMR_INTERVAL     1000   

/*=======================================================================*/
/*  Definition of all global and local Data                              */
/*=======================================================================*/

ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __attribute__((section(".RxDecripSection")));  /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __attribute__((section(".TxDescripSection"))); /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((section(".RxarraySection")));   /* Ethernet Receive Buffers */
uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((section(".TxarraySection")));   /* Ethernet Transmit Buffers */

static ETH_PHY_DRIVER *pPHY = NULL;
static uint8_t          OldLinkStatus = PHY_LINK_STATUS_NO_LINK; 
static sys_sem_t        TxSema;
static OS_SEMA          RxSema;

static uint32_t MulticastFilterList[MULTICAST_FILTER_SIZE];

static int              EthFirstRx = 0;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*
 * ether_crc32_be based on FreeBSD:
 * https://lists.freebsd.org/pipermail/freebsd-current/2004-June/029754.html
 */
static uint32_t ether_crc32_be (const uint8_t *buf, size_t len)
{
   static const uint8_t rev[] = {
      0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
      0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf
   };
   static const uint32_t crctab[] = {
      0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
      0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
      0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
      0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd
   };
   size_t i;
   uint32_t crc;
   uint8_t data;

   crc = 0xffffffff; /* initial value */

   for (i = 0; i < len; i++) {
      data = buf[i];
      crc = (crc << 4) ^ crctab[(crc >> 28) ^ rev[data & 0xf]];
      crc = (crc << 4) ^ crctab[(crc >> 28) ^ rev[data >> 4]];
   }

   return (crc);
} /* ether_crc32_be */


/*************************************************************************/
/*  UpdateHashFilter                                                     */
/*                                                                       */
/*  Update the hash filter by the given address.                         */
/*                                                                       */
/*  In    : pAddress                                                     */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void UpdateHashFilter (uint8_t *pAddress)
{
   uint32_t dCRC;
   
   dCRC = ether_crc32_be(pAddress, 6);
   dCRC = ~dCRC;
   
   dCRC = (dCRC >> 26) & 0x3F;
   if (dCRC > 31)
   {
      dCRC = dCRC - 32;
      ETH->MACHTHR |= (1u << dCRC); 
   }
   else
   {
      ETH->MACHTLR |= (1u << dCRC);
   }
   
} /* UpdateHashFilter */


/*************************************************************************/
/*  MulticastAddFilter                                                   */
/*                                                                       */
/*  Add a multicast address to the filter list.                          */
/*                                                                       */
/*  In    : dAddress                                                     */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void MulticastAddFilter (uint32_t dAddress)
{
   uint8_t Buffer[6];
   uint8_t bIndex;
   
   /*
    * Check if dAddress is still in list
    */
   for (bIndex = 0; bIndex < MULTICAST_FILTER_SIZE; bIndex++)
   {
      if (MulticastFilterList[bIndex] == dAddress)
      {
         return;
      }
   }    
   
   /*
    * Check for a free entry
    */
   for (bIndex = 0; bIndex < MULTICAST_FILTER_SIZE; bIndex++)
   {
      if (0 == MulticastFilterList[bIndex])
      {
         /* Save entry */
         MulticastFilterList[bIndex] = dAddress;
         
         /* Create MAC address */
         Buffer[0] = 0x01;
         Buffer[1] = 0x00;
         Buffer[2] = 0x5E;
         Buffer[3] = ((uint8_t *) &dAddress)[1] & 0x7f;
         Buffer[4] = ((uint8_t *) &dAddress)[2];
         Buffer[5] = ((uint8_t *) &dAddress)[3];

         UpdateHashFilter(Buffer);
         break;
      }
    }  

} /* MulticastAddFilter */


/*************************************************************************/
/*  MulticastDelFilter                                                   */
/*                                                                       */
/*  Delete a multicast address from the filter and list.                 */
/*                                                                       */
/*  In    : dAddress                                                     */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void MulticastDelFilter (uint32_t dAddress)
{
   uint8_t Buffer[6];
   uint8_t bIndex;
   uint8_t bRebuildList = FALSE;
   
   /*
    * Check if dAddress is still in list
    */
   for (bIndex = 0; bIndex < MULTICAST_FILTER_SIZE; bIndex++)
   {
      if (MulticastFilterList[bIndex] == dAddress)
      {
         bRebuildList = TRUE;
         MulticastFilterList[bIndex] = 0;
         break;
      }
   }    

   if (TRUE == bRebuildList)
   {
      /* Clear hash filter */
      ETH->MACHTHR = 0;
      ETH->MACHTLR = 0;

      /* Add multicast addresses from the list */
      for (bIndex = 0; bIndex < MULTICAST_FILTER_SIZE; bIndex++)
      {
         if (MulticastFilterList[bIndex] != 0)
         {
            dAddress = MulticastFilterList[bIndex];
            
            /* Create MAC address */
            Buffer[0] = 0x01;
            Buffer[1] = 0x00;
            Buffer[2] = 0x5E;
            Buffer[3] = ((uint8_t *) &dAddress)[1] & 0x7f;
            Buffer[4] = ((uint8_t *) &dAddress)[2];
            Buffer[5] = ((uint8_t *) &dAddress)[3];

            UpdateHashFilter(Buffer);
         }
      }
   }

} /* MulticastDelFilter */


/**
 * This function handle the IGMP filter ADD/DEL functionality.
 */
static err_t igmp_mac_filter (struct netif *netif, const ip4_addr_t *group, enum netif_mac_filter_action action)
{
   err_t error = ERR_OK;
   
   (void)netif;
   (void)group;

   switch (action)
   {
      case IGMP_ADD_MAC_FILTER:
      {
         MulticastAddFilter(group->addr);
         break;
      }
      
      case IGMP_DEL_MAC_FILTER:
      {
         MulticastDelFilter(group->addr);
         break;
      }
   
      default:
      {
         error = ERR_IF;
         break;
      }
   }
   
   return(error);
} /* igmp_mac_filter */


/**
 * Timer callback function for checking the Ethernet Link.
 *
 * @param arg contain the PHYDriver
 */
static void CheckLink (void *arg)
{
   uint8_t         LinkStatus;
   uint32_t        Value;
   struct netif   *netif = (struct netif*)arg;
   
   /* Get actual Link status */
   LinkStatus = pPHY->GetLinkStatus(pPHY);

   /* Check if the status has changed */
   if (LinkStatus != OldLinkStatus)
   {
      /* Check for DOWN/UP */
      if (PHY_LINK_STATUS_NO_LINK == LinkStatus)
      {  
         tcpip_try_callback((tcpip_callback_fn)netif_set_link_down, netif);
      }
      else
      {
         /* Update MAC */
         Value  = ETH->MACCR;
         Value &= ~(ETH_MACCR_FES | ETH_MACCR_DM);
      
         if (LinkStatus & PHY_LINK_STATUS_SPEED_100M)
         {
            Value |= ETH_MACCR_FES;
         }
      
         if (LinkStatus & PHY_LINK_STATUS_MODE_FULL)
         {
            Value |= ETH_MACCR_DM;
         }
      
         ETH->MACCR = Value;
         tcpip_try_callback((tcpip_callback_fn)netif_set_link_up, netif);
      }
   } /* end if (LinkStatus != OldLinkStatus) */
   
   OldLinkStatus = LinkStatus;      
   
} /* CheckLink */


/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static err_t low_level_init (struct netif *netif)
{
   netif->flags = 0;
   
   memset(MulticastFilterList, 0x00, sizeof(MulticastFilterList));
   
   /* Configure the HW of the board, and return the PHY info */
   pPHY = BoardETHConfig();
   if (NULL == pPHY)
   {
      /* Error, no PHY available */
      return(ERR_MEM);
   }
   
   /* Set netif MAC hardware address length */
   netif->hwaddr_len = ETHARP_HWADDR_LEN;
   
   /* Set netif MAC hardware address */
   memcpy(netif->hwaddr, netif->state, ETHARP_HWADDR_LEN); 
   
   /* Set netif maximum transfer unit */
   netif->mtu = 1500;

   /* Accept broadcast address, ARP traffic and Multicast */
   netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
   
#if (LWIP_IGMP >= 1)   
   /* Add IGMP support */   
   netif->flags |= NETIF_FLAG_IGMP;
   netif->igmp_mac_filter = igmp_mac_filter;
#endif   
   
   /* Create semaphore used for low_level_output */
   sys_sem_new(&TxSema, 1);

   /* Set the MAC address in the MAC */ 
   ETH_MACAddressConfig(&EthHandle, ETH_MAC_ADDRESS0, netif->hwaddr);

   /* Initialize Tx Descriptors list: Chain Mode */
   HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
     
   /* Initialize Rx Descriptors list: Chain Mode  */
   HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

   /* 
    * Create the IPRX thread 
    *
    * PHY init, ETH start and the LinkTimer will be 
    * handled by the ethernetif_input thread.
    */   
   sys_thread_new ("IPRX", ethernetif_input, netif,
                   TASK_IP_RX_STK_SIZE, TASK_IP_RX_PRIORITY);

   return(ERR_OK);
} /* low_level_init */


/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output (struct netif *netif, struct pbuf *p)
{
   err_t errval;
   struct pbuf *q;
   uint8_t *buffer;
   __IO ETH_DMADescTypeDef *DmaTxDesc;
   uint32_t framelength = 0;
   uint32_t bufferoffset = 0;
   uint32_t byteslefttocopy = 0;
   uint32_t payloadoffset = 0;

   (void)netif;
  
   sys_arch_sem_wait(&TxSema, 0);

   buffer = (uint8_t *)(EthHandle.TxDesc->Buffer1Addr);
   DmaTxDesc = EthHandle.TxDesc;
  
   /* copy frame from pbufs to driver buffers */
   for(q = p; q != NULL; q = q->next)
   {
      /* Is this buffer available? If not, goto error */
      if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
      {
         errval = ERR_USE;
         goto error; /*lint !e801*/
      }
    
      /* Get bytes in current lwIP buffer */
      byteslefttocopy = q->len;
      payloadoffset = 0;
    
      /* Check if the length of data to copy is bigger than Tx buffer size*/
      while( (byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE ) /*lint !e845*/
      {
         /* Copy data to Tx buffer*/
         memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)q->payload + payloadoffset), (ETH_TX_BUF_SIZE - bufferoffset) );
      
         /* Point to next descriptor */
         DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);
      
         /* Check if the buffer is available */
         if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
         {
           errval = ERR_USE;
           goto error;  /*lint !e801*/
         }
      
         buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);
      
         byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
         payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
         framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
         bufferoffset = 0;
      }
    
      /* Copy the remaining bytes */
      memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)q->payload + payloadoffset), byteslefttocopy );
      bufferoffset = bufferoffset + byteslefttocopy;
      framelength = framelength + byteslefttocopy;
   }
 
   /* Prepare transmit descriptors to give to DMA */ 
   HAL_ETH_TransmitFrame(&EthHandle, framelength);
  
   errval = ERR_OK;
  
error:
  
   /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
   if ((EthHandle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
   {
      /* Clear TUS ETHERNET DMA flag */
      EthHandle.Instance->DMASR = ETH_DMASR_TUS;
       
      /* Resume DMA transmission*/
      EthHandle.Instance->DMATPDR = 0;
   }

   sys_sem_signal(&TxSema);

   return(errval);
} /* low_level_output */


/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input (struct netif *netif)
{
   struct pbuf *p = NULL, *q = NULL;
   uint16_t len;
   uint8_t *buffer;
   __IO ETH_DMADescTypeDef *dmarxdesc;
   uint32_t bufferoffset = 0;
   uint32_t payloadoffset = 0;
   uint32_t byteslefttocopy = 0;
   uint32_t i;

   (void)netif;
  
   /* get received frame */
   if(HAL_ETH_GetReceivedFrame_IT(&EthHandle) != HAL_OK)
   {
      return(NULL);
   }    
  
   /* Obtain the size of the packet and put it into the "len" variable. */
   len = (uint16_t)EthHandle.RxFrameInfos.length;
   buffer = (uint8_t *)EthHandle.RxFrameInfos.buffer;
  
   if (len > 0)
   {
      /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
      p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
   }
  
   if (p != NULL)
   {
      dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
      bufferoffset = 0;
    
      for(q = p; q != NULL; q = q->next)
      {
         byteslefttocopy = q->len;
         payloadoffset = 0;
      
         /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size */
         while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE ) /*lint !e845*/
         {
            /* Copy data to pbuf */
            memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));
        
            /* Point to next descriptor */
            dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
            buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);
        
            byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
            payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
            bufferoffset = 0;
         }
      
         /* Copy remaining data in pbuf */
         memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), byteslefttocopy);
         bufferoffset = bufferoffset + byteslefttocopy;
      }
   }
    
   /* Release descriptors to DMA */
   /* Point to first descriptor */
   dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
   /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
   for (i=0; i< EthHandle.RxFrameInfos.SegCount; i++)
   {  
      dmarxdesc->Status |= ETH_DMARXDESC_OWN;
      dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
   }
    
   /* Clear Segment_Count */
   EthHandle.RxFrameInfos.SegCount =0;
  
   /* When Rx Buffer unavailable flag is set: clear it and resume reception */
   if ((EthHandle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)  
   {
      /* Clear RBUS ETHERNET DMA flag */
      EthHandle.Instance->DMASR = ETH_DMASR_RBUS;
      
      /* Resume DMA reception */
      EthHandle.Instance->DMARPDR = 0;
   }
   
   return(p);
} /* low_level_input */


/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
static void ethernetif_input (void *arg)
{
   int             rc;
   struct pbuf    *p;
   struct netif   *netif = (struct netif*)arg;
   struct eth_hdr *ethhdr;
   uint32_t        Checktime = OS_TimeGet();
   
   /* Create the receive semaphore */
   OS_SemaCreate(&RxSema, 0, 1);

   /* Init the PHY */
   pPHY->Init(pPHY);

   /* Enable MAC and DMA transmission and reception */
   HAL_ETH_Start(&EthHandle);   

   while (1)
   {
      /* Wait for an event */
      rc = OS_SemaWait(&RxSema, 500);
      if (OS_RC_OK == rc)
      {
         if (0 == EthFirstRx)
         {
            EthFirstRx = SysTick->VAL;
         }
      
         while (1)
         {
            /* move received packet into a new pbuf */
            p = low_level_input( netif );
            
            /* no packet could be read, silently ignore this */
            if (p == NULL) break;
            
            /* points to packet payload, which starts with an Ethernet header */
            ethhdr = p->payload;
            
            switch (htons(ethhdr->type)) 
            {
               /* IP or ARP packet? */
               case ETHTYPE_IP:
               case ETHTYPE_ARP:
               
                  /* full packet send to tcpip_thread to process */
                  if (netif->input(p, netif) != ERR_OK)
                  {
                     LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                     pbuf_free(p);
                  }
                  
                  /* 
                   * In case we have receive a packet, we can
                   * restart the timeout for the CheckLink.
                   */
                  if (PHY_LINK_STATUS_NO_LINK == OldLinkStatus)
                  {
                     CheckLink(netif);
                  }
                  else
                  {
                     Checktime = OS_TimeGet();               
                  }             
                  break;
               
               default:
                  pbuf_free(p);
                  break;                  
            } /* end switch (htons(ethhdr->type)) */               
         } /* while (1), loop over low_level_input */
      } /* if (OS_RC_OK == rc) */
      
      /* Check timeout for CheckLink */
      if ((OS_TimeGet() - Checktime) > LINK_TMR_INTERVAL)
      {
         Checktime = OS_TimeGet();
         CheckLink(netif);
      }
      
   } /* end while (1), task loop */
   
} /* ethernetif_input */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/**
  * Ethernet Rx Transfer completed callback
  */
void HAL_ETH_RxCpltCallback (ETH_HandleTypeDef *heth)
{
   (void)heth;

   /* Send an "event" to wakeup the Receiver task */
   OS_SemaSignalFromInt(&RxSema);
   
} /* HAL_ETH_RxCpltCallback */


/**
 * Ethernet IRQ handler
 */
void ETH_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();

   HAL_ETH_IRQHandler(&EthHandle);
   
   TAL_CPU_IRQ_EXIT();
} /* ETH_IRQHandler */


/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init (struct netif *netif)
{
   err_t error = ERR_MEM;
   
   if (netif != NULL)
   {
#if LWIP_NETIF_HOSTNAME
      /* Initialize interface hostname */
      netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

      /* Descriptive abbreviation for this interface */
      netif->name[0] = IFNAME0;
      netif->name[1] = IFNAME1;
  
      /* 
       * We directly use etharp_output() here to save a function call.
       * You can instead declare your own function an call etharp_output()
       * from it if you have to do some checks before sending (e.g. if link
       * is available...) 
       */
      netif->output     = etharp_output;
      netif->linkoutput = low_level_output;
  
      /* Initialize the hardware */
      error = low_level_init(netif);
   }      

   return(error);
} /* ethernetif_init */

int ethernet_first_rx (void)
{
   return(EthFirstRx);
} /* ethernet_ready */

/*** EOF ***/
