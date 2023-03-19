/**************************************************************************
*  Copyright (c) 2018-2022 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Based on an example from ST. Therefore partial copyright:
*  Copyright (c) 2016 STMicroelectronics
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
#define __ETH_CONFIG_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include <string.h>

#include "tal.h"
#include "eth_phy_driver.h"

/*=======================================================================*/
/*  Extern                                                               */
/*=======================================================================*/

/* Global Ethernet handle*/
extern ETH_HandleTypeDef EthHandle;

extern void ETH_MACDMAConfig(ETH_HandleTypeDef *heth, uint32_t err);

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/* PHY registers */
#define PHYIDR1   0x02
#define PHYIDR2   0x03

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static ETH_PHY_DRIVER PHYDriver;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  PHYReadReg                                                           */
/*                                                                       */
/*  Read a register of the PHY.                                          */
/*                                                                       */
/*  In    : pPHY, Reg                                                    */
/*  Out   : none                                                         */
/*  Return: Value of the register                                        */
/*************************************************************************/
static uint16_t PHYReadReg (ETH_PHY_DRIVER *pPHY, uint16_t Reg)
{
   uint32_t RegValue;
   
   EthHandle.Init.PhyAddress = pPHY->Address;

   HAL_ETH_ReadPHYRegister(&EthHandle, Reg, &RegValue);

   return(RegValue);
} /* PHYReadReg */

/*************************************************************************/
/*  PHYWriteReg                                                          */
/*                                                                       */
/*  Write a register of the PHY.                                         */
/*                                                                       */
/*  In    : pPHY, Reg, Value                                             */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void PHYWriteReg (ETH_PHY_DRIVER *pPHY, uint16_t Reg, uint16_t Value)
{
   EthHandle.Init.PhyAddress = pPHY->Address;

   HAL_ETH_WritePHYRegister(&EthHandle, Reg, Value);
} /* PHYReadReg */

/*************************************************************************/
/*  FindPHY                                                              */
/*                                                                       */
/*  Find the PHY which is used.                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
ETH_PHY_DRIVER *FindPHY (void)
{
   uint32_t Value;
   uint8_t  Addr;

   /*
    * Setup PHYDriver
    */
   memset(&PHYDriver, 0x00, sizeof(PHYDriver));    
   PHYDriver.ReadReg  = PHYReadReg;
   PHYDriver.WriteReg = PHYWriteReg;

   /*
    * Search the PHY...
    */
   for (Addr = 0; Addr < 32; Addr++)
   {    
      PHYDriver.Address = Addr; 
    
      /* Reset PHY */
      PHYWriteReg(&PHYDriver, PHY_BCR, PHY_RESET);
      
      /* Give PHY some time to reset */
      OS_TimeWait(2);
    
      Value  = PHYReadReg(&PHYDriver, PHYIDR1);    
      Value  = Value << 16;
      Value |= PHYReadReg(&PHYDriver, PHYIDR2);
      Value &= PHY_ID_MASK;
   
      switch (Value)
      {
         case PHY_ID_LAN8742A:
         {
            PHY_LAN8742.ReadReg  = PHYReadReg;
            PHY_LAN8742.WriteReg = PHYWriteReg;
            PHY_LAN8742.Address  = Addr;
            return(&PHY_LAN8742);
            break;   /*lint !e527*/
         }
      
         default:
         {
            /* Do noting */
            break;
         }
      }
   }      
      
   return(NULL);
} /* CheckPHY */

#if defined(USE_BOARD_MB1225)
/*************************************************************************/
/*  HAL_ETH_MspInit                                                      */
/*                                                                       */
/*  Configures the GPIO ports.                                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void HAL_ETH_MspInit (ETH_HandleTypeDef *heth)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  
   (void)heth;
  
   /* Enable GPIOs clocks */
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOG_CLK_ENABLE();

   /* Ethernet pins configuration ****************************************/
   /*
        RMII_REF_CLK ----------------------> PA1
        RMII_MDIO -------------------------> PA2
        RMII_MDC --------------------------> PC1
        RMII_MII_CRS_DV -------------------> PA7
        RMII_MII_RXD0 ---------------------> PC4
        RMII_MII_RXD1 ---------------------> PC5
        RMII_MII_RXER ---------------------> PG2
        RMII_MII_TX_EN --------------------> PG11
        RMII_MII_TXD0 ---------------------> PG13
        RMII_MII_TXD1 ---------------------> PG14
   */

   /* Configure PA1, PA2 and PA7 */
   GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStructure.Pull = GPIO_NOPULL; 
   GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
   GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
   /* Configure PC1, PC4 and PC5 */
   GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Configure PG2, PG11, PG13 and PG14 */
   GPIO_InitStructure.Pin =  GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
   HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);
  
   /* Enable ETHERNET clock  */
   __HAL_RCC_ETH_CLK_ENABLE();
} /* HAL_ETH_MspInit */
#endif /* USE_BOARD_MB1225 */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  BoardETHConfig                                                       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: NULL / PHYDriver                                             */
/*************************************************************************/
ETH_PHY_DRIVER *BoardETHConfig (void)
{
   ETH_PHY_DRIVER *pPHY;
   
   uint8_t macaddress[6]= { MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5 };
  
   EthHandle.Instance             = ETH;  
   EthHandle.Init.MACAddr         = macaddress;
   EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;
   EthHandle.Init.Speed           = ETH_SPEED_100M;
   EthHandle.Init.DuplexMode      = ETH_MODE_FULLDUPLEX;
   EthHandle.Init.MediaInterface  = ETH_MEDIA_INTERFACE_RMII;
   EthHandle.Init.RxMode          = ETH_RXINTERRUPT_MODE;
   EthHandle.Init.ChecksumMode    = ETH_CHECKSUM_BY_HARDWARE;
   EthHandle.Init.PhyAddress      = LAN8742A_PHY_ADDRESS;

   if (HAL_ETH_Init(&EthHandle) != HAL_OK)
   {
      return(NULL);
   }
   
   
   /*
    * Find the PHY which is used on the board.
    */
   pPHY = FindPHY();
   if (pPHY != NULL)
   {
      /* 
       * Configure Ethernet interrupt
       */
      NVIC_SetPriority(ETH_IRQn, 3);
      NVIC_EnableIRQ(ETH_IRQn);
   }      

   return(pPHY);
} /* BoardETHConfig */

/*** EOF ***/
