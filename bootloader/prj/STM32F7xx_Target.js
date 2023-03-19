/*****************************************************************************
 *                   SEGGER Microcontroller GmbH & Co. KG                    *
 *            Solutions for real time microcontroller applications           *
 *****************************************************************************
 *                                                                           *
 *               (c) 2017 SEGGER Microcontroller GmbH & Co. KG               *
 *                                                                           *
 *           Internet: www.segger.com   Support: support@segger.com          *
 *                                                                           *
 *****************************************************************************/

function Reset() {
  TargetInterface.resetAndStop();
}

function EnableExtSDRAM() {
  // Enable GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface clock 
  TargetInterface.pokeWord(0x40023830, 0x000001F8);   // RCC->AHB1ENR
  
  // Enable FMC clock
  TargetInterface.pokeWord(0x40023838, 0x00000001);   // RCC->AHB3ENR
    
  // Connect PDx pins to FSMC Alternate function 
  TargetInterface.pokeWord(0x40020C20, 0x000000CC);   // GPIOD->AFR[0]
  TargetInterface.pokeWord(0x40020C24, 0xCC000CCC);   // GPIOD->AFR[1]  
  // Configure PDx pins in Alternate function mode   
  TargetInterface.pokeWord(0x40020C00, 0xA02A000A);   // GPIOD->MODER
  // Configure PDx pins speed to 100 MHz   
  TargetInterface.pokeWord(0x40020C08, 0xF03F000F);   // GPIOD->OSPEEDR
  // Configure PDx pins Output type to push-pull   
  TargetInterface.pokeWord(0x40020C04, 0x00000000);   // GPIOD->OTYPER
  // No pull-up, pull-down for PDx pins  
  TargetInterface.pokeWord(0x40020C0C, 0x50150005);   // GPIOD->PUPDR
  

  // Connect PEx pins to FSMC Alternate function 
  TargetInterface.pokeWord(0x40021020, 0xC00000CC);   // GPIOE->AFR[0]
  TargetInterface.pokeWord(0x40021024, 0xCCCCCCCC);   // GPIOE->AFR[1]
  // Configure PEx pins in Alternate function mode  
  TargetInterface.pokeWord(0x40021000, 0xAAAA800A);   // GPIOE->MODER
  // Configure PEx pins speed to 100 MHz  
  TargetInterface.pokeWord(0x40021008, 0xFFFFC00F);   // GPIOE->OSPEEDR
  // Configure PEx pins Output type to push-pull   
  TargetInterface.pokeWord(0x40021004, 0x00000000);   // GPIOE->OTYPER
  // No pull-up, pull-down for PEx pins  
  TargetInterface.pokeWord(0x4002100C, 0x55554005);   // GPIOE->PUPDR


  // Connect PFx pins to FSMC Alternate function 
  TargetInterface.pokeWord(0x40021420, 0x00CCCCCC);   // GPIOF->AFR[0]
  TargetInterface.pokeWord(0x40021424, 0xCCCCC000);   // GPIOF->AFR[1]
  // Configure PFx pins in Alternate function mode    
  TargetInterface.pokeWord(0x40021400, 0xAA800AAA);   // GPIOF->MODER
  // Configure PFx pins speed to 100 MHz  
  TargetInterface.pokeWord(0x40021408, 0xFFC00FFF);   // GPIOF->OSPEEDR
  // Configure PFx pins Output type to push-pull   
  TargetInterface.pokeWord(0x40021404, 0x00000000);   // GPIOF->OTYPER
  // No pull-up, pull-down for PFx pins  
  TargetInterface.pokeWord(0x4002140C, 0x55400555);   // GPIOF->PUPDR


  // Connect PGx pins to FSMC Alternate function 
  TargetInterface.pokeWord(0x40021820, 0x00CC00CC);   // GPIOG->AFR[0]
  TargetInterface.pokeWord(0x40021824, 0xC000000C);   // GPIOG->AFR[1]
  // Configure PGx pins in Alternate function mode  
  TargetInterface.pokeWord(0x40021800, 0x80020A0A);   // GPIOG->MODER
  // Configure PGx pins speed to 100 MHz  
  TargetInterface.pokeWord(0x40021808, 0xC0030F3F);   // GPIOG->OSPEEDR
  // Configure PGx pins Output type to push-pull   
  TargetInterface.pokeWord(0x40021804, 0x00000000);   // GPIOG->OTYPER
  // No pull-up, pull-down for PGx pins  
  TargetInterface.pokeWord(0x4002180C, 0x40010505);   // GPIOG->PUPDR


  // Connect PHx pins to FSMC Alternate function 
  TargetInterface.pokeWord(0x40021C20, 0x00C0CC00);   // GPIOH->AFR[0]
  TargetInterface.pokeWord(0x40021C24, 0xCCCCCCCC);   // GPIOH->AFR[1]
  // Configure PGx pins in Alternate function mode  
  TargetInterface.pokeWord(0x40021C00, 0xAAAA08A0);   // GPIOH->MODER
  // Configure PGx pins speed to 100 MHz  
  TargetInterface.pokeWord(0x40021C08, 0xFFFF0CF0);   // GPIOH->OSPEEDR
  // Configure PGx pins Output type to push-pull   
  TargetInterface.pokeWord(0x40021C04, 0x00000000);   // GPIOH->OTYPER
  // No pull-up, pull-down for PGx pins  
  TargetInterface.pokeWord(0x40021C0C, 0x55550450);   // GPIOH->PUPDR
  

  // Connect PIx pins to FSMC Alternate function 
  TargetInterface.pokeWord(0x40022020, 0xCCCCCCCC);   // GPIOI->AFR[0]
  TargetInterface.pokeWord(0x40022024, 0x00000CC0);   // GPIOI->AFR[1]
  // Configure PIx pins in Alternate function mode  
  TargetInterface.pokeWord(0x40022000, 0x0028AAAA);   // GPIOI->MODER
  // Configure PIx pins speed to 100 MHz  
  TargetInterface.pokeWord(0x40022008, 0x003CFFFF);   // GPIOI->OSPEEDR
  // Configure PIx pins Output type to push-pull   
  TargetInterface.pokeWord(0x40022004, 0x00000000);   // GPIOI->OTYPER
  // No pull-up, pull-down for PIx pins  
  TargetInterface.pokeWord(0x40021C0C, 0x00145555);   // GPIOI->PUPDR
  
    
  //-- FSMC Configuration --------------------------------
  // Enable the FSMC interface clock 
  TargetInterface.pokeWord(0x40023838, 0x00000001);   // RCC->AHB3ENR

  // Configure and enable SDRAM bank1 
  TargetInterface.pokeWord(0xA0000140, 0x000019E4);   // FMC_Bank5_6->SDCR[0] 
  TargetInterface.pokeWord(0xA0000148, 0x01125461);   // FMC_Bank5_6->SDTR[0] 
  
  // SDRAM initialization sequence
  
  //Clock enable command
  TargetInterface.pokeWord(0xA0000150, 0x00000011);   // FMC_Bank5_6->SDCMR
  
  // Delay
  TargetInterface.delay(100);
  
  // PALL command
  TargetInterface.pokeWord(0xA0000150, 0x00000012);   // FMC_Bank5_6->SDCMR 
  TargetInterface.delay(10);
  
  // Auto refresh command
  TargetInterface.pokeWord(0xA0000150, 0x000000F3);   // FMC_Bank5_6->SDCMR 
  TargetInterface.delay(10);
  
  // MRD register program
  TargetInterface.pokeWord(0xA0000150, 0x00046814);   // FMC_Bank5_6->SDCMR 
  TargetInterface.delay(10);
  
  // Set refresh count
  TargetInterface.pokeWord(0xA0000154, 0x00000B80);   // FMC_Bank5_6->SDRTR 
  
  // Disable write protection
  //TargetInterface.pokeWord(0xA0000140, 0x00001954);   // FMC_Bank5_6->SDCR[0] 
  
}

function SDRAM_Reset() {
  Reset();
  EnableExtSDRAM();
}

function EnableTrace(traceInterfaceType) {
  // TODO: Enable trace
}

