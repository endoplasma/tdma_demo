#include "rf231_slotted_hal.h"
#include <stm32f4xx.h>                    /* STM32F4xx Definitions              */
#include <nvic.h>                         /* STM32F4xx Definitions              */
#include <clock.h>
#include <string.h>                       /* for storing DMA data */
#include "rf231_slotted_registermap.h"
#include "rf231_slotted.h"


#ifdef SLOTTED_KOORDINATOR
#ifdef JITTER_SIMULATION
#warning Building for Slotted Koordinator in simulation mode
#else
#warning Building for Slotted Koordinator
#endif /* JITTER_SIMULATION */
#else
#warning Building for Slotted Client
#endif /* SLOTTED_KOORDINATOR */

/******************************************************************************
 * SPI Makros 
 ******************************************************************************/
/* Start the SPI transaction by pulling the Slave Select low. */
#define HAL_SPI_TRANSFER_OPEN() {		\
  HAL_SS_LOW(); 

#define HAL_SPI_TRANSFER_WRITE(to_write) {		\
    volatile uint16_t dummy = SPIx->DR;			\
    while ((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE) {	\
      ;};						\
    SPIx->DR = (to_write); }

#define HAL_SPI_TRANSFER_WAIT()  {				\
    while ((SPIx->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {;} \
    ; }

#define HAL_SPI_TRANSFER_READ() (SPIx->DR)

#define HAL_SPI_TRANSFER_CLOSE()					\
  while ((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE) {;			\
  }; /* see RM0090 - 25.3.8 */						\
  while ((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY) {;};			\
  /* End the transaction by pulling the Slave Select High. */		\
  HAL_SS_HIGH();							\
  }

__INLINE uint8_t spiWrite(uint8_t byte)
{
  HAL_SPI_TRANSFER_WRITE(byte);
  HAL_SPI_TRANSFER_WAIT();
  return HAL_SPI_TRANSFER_READ();
  }

#define HAL_SPI_TRANSFER(to_write) (spiWrite(to_write))

/******************************************************************************
 * Extern and static Variable Definitions
 ******************************************************************************/

extern void rf231_slotted_IC_irqh(uint32_t capture);
//extern void rf231_slotted_OC_irqh(void);

PROCESS_NAME(rf231_slotted_process);

static uint8_t dma_buffer[2+127+5+3];

void rf230_interrupt(void);

extern hal_rx_frame_t rxframe[RF230_CONF_RX_BUFFERS];
extern uint8_t rxframe_head,rxframe_tail;

extern uint8_t volatile state;

extern proto_conf_t rf231_slotted_config;

uint32_t slotTime;

void hal_set_oc(uint32_t oc_value);
/*----------------------------------------------------------------------------*/
/** \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf230_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 */
uint8_t
hal_register_read(uint8_t address)
{
  uint8_t register_value;
  /* Add the register read command to the register address. */
  address |= 0x80;

  HAL_SPI_TRANSFER_OPEN();
  HAL_SPI_TRANSFER(address);
  register_value = HAL_SPI_TRANSFER(0);
  HAL_SPI_TRANSFER_CLOSE();
  return register_value;
}
  
/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf230_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 */
void
hal_register_write(uint8_t address, uint8_t value)
{
  /* Add the Register Write (short mode) command to the address. */
  address = 0xc0 | address;
  
  HAL_SPI_TRANSFER_OPEN();
  HAL_SPI_TRANSFER(address);
  HAL_SPI_TRANSFER(value);
  HAL_SPI_TRANSFER_CLOSE();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf230_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 */
uint8_t
hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position)
{
  /* Read current register value and mask out subregister. */
  uint8_t register_value = hal_register_read(address);
  register_value &= mask;
  register_value >>= position; /* Align subregister value. */
   
  return register_value;
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf230_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 */
void
hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
		      uint8_t value)
{
  /* Read current register value and mask area outside the subregister. */
  volatile uint8_t register_value = hal_register_read(address);
  register_value &= ~mask;
  
  /* Start preparing the new subregister value. shift in place and mask. */
  value <<= position;
  value &= mask;
  value |= register_value; /* Set the new subregister value. */
  
  /* Write the modified register value. */
  hal_register_write(address, value);
}
     
/*----------------------------------------------------------------------------*/
/** \brief  Transfer a frame from the radio transceiver to a RAM buffer
 *
 *          This version is optimized for use with contiki RF230BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *          If the frame length is out of the defined bounds, the length, lqi and crc
 *          are set to zero.
 *
 *  \param  rx_frame    Pointer to the data structure where the frame is stored.
 */
void
hal_frame_read(hal_rx_frame_t *rx_frame)
{
  uint8_t *rx_data;
  uint8_t frame_length;
				
  /*Send frame read (long mode) command.*/
  HAL_SPI_TRANSFER_OPEN();
  HAL_SPI_TRANSFER(0x20);

  /*Read frame length. This includes the checksum. */
  frame_length = HAL_SPI_TRANSFER(0);

  /*Check for correct frame length. Bypassing this test can result in a buffer overrun! */
  if ( 0 || ((frame_length >= HAL_MIN_FRAME_LENGTH) && (frame_length <= HAL_MAX_FRAME_LENGTH))) {

    rx_data = (rx_frame->data);
    rx_frame->length = frame_length;

    /*Transfer frame buffer to RAM buffer */

    HAL_SPI_TRANSFER_WRITE(0);
    HAL_SPI_TRANSFER_WAIT();
    do{
      *rx_data++ = HAL_SPI_TRANSFER_READ();
      HAL_SPI_TRANSFER_WRITE(0);

      /* CRC was checked in hardware, but redoing the checksum here ensures the rx buffer
       * is not being overwritten by the next packet. Since that lengthy computation makes
       * such overwrites more likely, we skip it and hope for the best.
       * Without the check a full buffer is read in 320us at 2x spi clocking.
       * The 802.15.4 standard requires 640us after a greater than 18 byte frame.
       * With a low interrupt latency overwrites should never occur.
       */
      //          crc = _crc_ccitt_update(crc, tempData);

      HAL_SPI_TRANSFER_WAIT();
      
    } while (--frame_length > 0);
    
    
    /*Read LQI value for this frame.*/
    rx_frame->lqi = HAL_SPI_TRANSFER_READ();
    
    /* If crc was calculated set crc field in hal_rx_frame_t accordingly.
     * Else show the crc has passed the hardware check.
     */
    rx_frame->crc   = true;
    
  } else {
    /* Length test failed */
    rx_frame->length = 0;
    rx_frame->lqi    = 0;
    rx_frame->crc    = false;
  }
  
  HAL_SPI_TRANSFER_CLOSE();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
 
void
hal_frame_write(uint8_t *write_buffer, uint8_t length)
{
  /* Download to the Frame Buffer.
   * When the FCS is autogenerated there is no need to transfer the last two bytes
   * since they will be overwritten.
   */
  int i = 2;
  dma_buffer[0] = 0x60;
  dma_buffer[1] = length;
  memcpy(&dma_buffer[2],write_buffer,length);


  /*Send frame read (long mode) command.*/
  HAL_SPI_TRANSFER_OPEN();
  HAL_SPI_TRANSFER(0x60);
  HAL_SPI_TRANSFER(length);
  do{
    HAL_SPI_TRANSFER(dma_buffer[i++]);
  } while (--length > 0);
  
  HAL_SPI_TRANSFER_CLOSE();


  /* /\* Send Frame Transmit (long mode) command and frame length *\/ */
  /* dma_buffer[0] = 0x60; */
  /* dma_buffer[1] = length; */
  /* memcpy(&dma_buffer[2],write_buffer,length); */

  /* /\* Disable SPI_DMAx_STREAM_TX by clearing EN bit *\/ */
  /* SPI_DMAx_STREAM_TX->CR &= ~DMA_SxCR_EN; */
  /* /\* Disable SPI_DMAx_STREAM_RX by clearing EN bit *\/ */
  /* SPI_DMAx_STREAM_RX->CR &= ~DMA_SxCR_EN; */

  /* /\* Clear Transfer Complete Interrupt Flag - important for following DMA transactions *\/ */
  /* SPI_DMA_IFCR_TX |= SPI_DMA_IFCR_CTCIF_TX; */
  /* SPI_DMA_IFCR_RX |= SPI_DMA_IFCR_CTCIF_RX; */

  /* /\* Write to SPI_DMAx_STREAM_TX NDTR register *\/ */
  /* SPI_DMAx_STREAM_TX->NDTR = 2+length; */
  /* /\* Write to SPI_DMAx_STREAM_RX NDTR register *\/ */
  /* SPI_DMAx_STREAM_RX->NDTR = 2+length; */

  /* /\* Write to SPI_DMAx_STREAM_TX M0AR *\/ */
  /* SPI_DMAx_STREAM_TX->M0AR = (uint32_t)dma_buffer; */
  /* /\* Write to SPI_DMAx_STREAM_RX M0AR *\/ */
  /* SPI_DMAx_STREAM_RX->M0AR = (uint32_t)dma_buffer; */

  /* /\* Enable SPI_DMAx_STREAM_TX by setting EN bit *\/ */
  /* SPI_DMAx_STREAM_TX->CR |= DMA_SxCR_EN; */
  /* /\* Enable SPI_DMAx_STREAM_RX by setting EN bit *\/ */
  /* SPI_DMAx_STREAM_RX->CR |= DMA_SxCR_EN; */

  /* /\* Assert SS *\/ */
  /* HAL_SS_LOW(); */

  /* /\* Enable Tx buffer DMA *\/ */
  /* SPIx->CR2 |= SPI_CR2_TXDMAEN; */
  /* /\* Enable Rx buffer DMA *\/ */
  /* SPIx->CR2 |= SPI_CR2_RXDMAEN; */
}


/****************************************************************************
 * Interrupt Service Routines
 *****************************************************************************/

/* HAL_RF231_ISR() */
/* { */
/*   volatile uint8_t state; */
/*   uint8_t interrupt_source; /\* used after HAL_SPI_TRANSFER_OPEN/CLOSE block *\/ */
    
/*   /\*Read Interrupt source.*\/ */
/*   /\*Send Register address and read register content.*\/ */
/*   HAL_SPI_TRANSFER_OPEN(); */
/*   HAL_SPI_TRANSFER_WRITE(0x80 | RG_IRQ_STATUS); */
/*   HAL_SPI_TRANSFER_WAIT(); /\* AFTER possible interleaved processing *\/ */
/*   interrupt_source = HAL_SPI_TRANSFER(0); */
/*   HAL_SPI_TRANSFER_CLOSE(); */

/*   /\*Handle the incomming interrupt. Prioritized.*\/ */
/*   if ((interrupt_source & HAL_RX_START_MASK)){ */
/*     /\*********************** */
/*      * RX_START IRQ */
/*      **********************\/ */
        
/*   } else if (interrupt_source & HAL_TRX_END_MASK){ */
/*     /\*********************** */
/*      * TRX_END IRQ */
/*      **********************\/ */
/*     state = hal_subregister_read(SR_TRX_STATUS); */
/*     if((state == BUSY_RX_AACK) || (state == RX_ON) || (state == BUSY_RX) || (state == RX_AACK_ON)) */
/*       { */
/* 	/\* TRX_END IRQ was caused by a received packet * Buffer the */
/* 	 * frame and call rf230_interrupt to schedule poll for */
/* 	 * rf230 receive process *\/ */

/* 	hal_frame_read(&rxframe[rxframe_tail]); */
/* 	rxframe_tail++;  */
/* 	if (rxframe_tail >= RF230_CONF_RX_BUFFERS) */
/* 	  { */
/* 	    rxframe_tail=0; */
/* 	  } */
/* 	//	rf230_interrupt(); */
	 
/*       } */
              
/*   } else if (interrupt_source & HAL_TRX_UR_MASK){ */
/*     /\*********************** */
/*      * TRX_UR IRQ */
/*      **********************\/ */
/*   } else if (interrupt_source & HAL_PLL_UNLOCK_MASK){ */
/*     /\*********************** */
/*      * PLL_UNLOCK IRQ */
/*      **********************\/ */
/*   } else if (interrupt_source & HAL_PLL_LOCK_MASK){ */
/*     /\*********************** */
/*      * PLL_LOCK IRQ */
/*      **********************\/ */
/*   } else if (interrupt_source & HAL_BAT_LOW_MASK){ */
/*     /\*  Disable BAT_LOW interrupt to prevent endless */
/*      *  interrupts. The interrupt will continously be asserted */
/*      *  while the supply voltage is less than the user-defined */
/*      *  voltage threshold. *\/ */
/*     /\*********************** */
/*      * LOW_BAT IRQ */
/*      **********************\/ */
/*   } else { */
/*     /\*********************** */
/*      * UNDEFINED MASKED IRQ */
/*      **********************\/ */
/*   } */
/* } */


/**
 * Interrupthandler to stop the dma transfer
 */
void SPI_DMAx_STREAM_RX_IRQHandler(void)
{
  /* Clear Transfer Complete Interrupt Flag */
  SPI_DMA_IFCR_RX |= SPI_DMA_IFCR_CTCIF_RX;
  /* Release SS */
  HAL_SS_HIGH();
}


/**
 * Interupthandler Routine for TIMx
 */
void TIMx_IRQHandler(void)
{
  volatile uint32_t capture;
  uint8_t interrupt_source; /* used after HAL_SPI_TRANSFER_OPEN/CLOSE block */
#ifdef JITTER_SIMULATION
  uint16_t jitter = 0;
#endif /* JITTER_SIMULATION */
  if (TIMx->SR & TIM_IC_IRQ_FLAG)	
    {
      /******************************************
       * Input Capture Interrupt detected - New Packet received
       *******************************************/
      /* rf231 issued an interrupt
       *
       * read the values of IRQ status register to clear the interrupt
       * flags an get info about what caused the IRQ */
    
      /*Read Interrupt source.*/
      /*Send Register address and read register content.*/
      HAL_SPI_TRANSFER_OPEN();
      HAL_SPI_TRANSFER_WRITE(0x80 | RG_IRQ_STATUS);
      HAL_SPI_TRANSFER_WAIT(); /* AFTER possible interleaved processing */
      interrupt_source = HAL_SPI_TRANSFER(0);
      HAL_SPI_TRANSFER_CLOSE();
      /* reset the IRQ Flag */
      TIMx->SR &= ~TIM_IC_IRQ_FLAG;
      
      if ((interrupt_source & HAL_RX_START_MASK)) { 
	/***************************************************
         * RX START Interrupt
         ***************************************************/
	capture = TIMx->CCR_IC;
#ifndef SLOTTED_KOORDINATOR
	if(state == RF231_STATE_IDLE){
	  rf231_slotted_IC_irqh(capture);
	  process_post(&rf231_slotted_process, INPUT_CAPTURE_EVENT, NULL);
	} 
#endif /* SLOTTED_KOORDINATOR */

      } else if (interrupt_source & HAL_TRX_END_MASK){
	/***************************************************
         * TRX END Interrupt
         ***************************************************/
	if(state == RF231_STATE_IDLE){
	  process_post(&rf231_slotted_process, HANDLE_PACKET_EVENT, NULL);
	} else if(state == RF231_STATE_SEND){
	  process_post(&rf231_slotted_process, FRAME_SEND_EVENT, NULL);
	}
      }
    }
  else if (TIMx->SR & TIM_OC_IRQ_FLAG)
    {
      /******************************************
       * Output Compare IRQ - Time to send Frame
       *******************************************/
      /* Clear IRQ Flag*/
      TIMx->SR &= ~TIM_OC_IRQ_FLAG;

#ifdef SLOTTED_KOORDINATOR
      /* Set Compare Value for next send and clear IRQ Flag	*/
#ifdef JITTER_SIMULATION
#define JITTER_TICKS         32
#define JITTER_MASK          (JITTER_TICKS * 2 - 1)
      /* generate random Jitter to simulate the effect */
      /* mask the last 6 bit -> jitter of 64/Ticks_Per_Second */
      jitter = random_rand();
      jitter = (jitter & JITTER_MASK);
      if(jitter == 0){
	jitter = JITTER_TICKS;
      }

      TIMx->CCR_OC=TIMx->CCR_OC + TDMA_PERIOD_TICKS + jitter - JITTER_TICKS;
#else /* JITTER_SIMULATION */
      /* set the next BEACON send time */
      TIMx->CCR_OC=TIMx->CCR_OC + TDMA_PERIOD_TICKS;
#endif /* JITTER_SIMULATION */
#endif /* SLOTTED_KOORDINATOR */
      
      /* Generate Output Pulse by toggling the output signal polarity
       * change compare mode output signal from High level to low to end the pulse */
      TIMx->CCMR_OC &= ~(TIM_CCMR2_OC4M);
      TIMx->CCMR_OC |= (TIM_CCMR2_OC4M_2);
      /* set compare mode output signal back to high for the next period */
      TIMx->CCMR_OC &= ~(TIM_CCMR2_OC4M);
      TIMx->CCMR_OC |= (TIM_CCMR2_OC4M_0);

#ifdef JITTER_SIMULATION /* JITTER_SIMULATION */

      if ((random_rand() & (0x7 << 8)) == 0) {
	//TIMx->CCER &= ~(CCER_OC_CCE);
	TIMx->CCMR_OC &= ~(TIM_CCMR2_OC4M);
      }
#endif /* JITTER_SIMULATION */

    }
#ifndef SLOTTED_KOORDINATOR
  else if (TIMx->SR & TIM_BEACON_MISSED_IRQ_FLAG)
    {
      /******************************************
       * Beacon Missed Timer expired
       *******************************************/
      /* Clear IRQ Flag*/
      TIMx->SR &= ~TIM_BEACON_MISSED_IRQ_FLAG;
      process_post(&rf231_slotted_process, BEACON_MISSED_EVENT, NULL);
    }
#endif
  else if (TIMx->SR & TIM_TX_MODE_IRQ_FLAG)
    {
      /******************************************
       * TX_MODE Timer expired
       *******************************************/
      /* Clear IRQ Flag*/
      TIMx->SR &= ~TIM_TX_MODE_IRQ_FLAG;
      process_post(&rf231_slotted_process, TX_MODE_TIMER_EVENT, NULL);
    }
}

/**
 * set a new Output compare Value
 */
void hal_set_oc(uint32_t oc_value)
{
  TIMx->CCR_OC=oc_value;
}

/**
 * add a value to the current OC value
 */
void hal_update_oc(uint32_t oc_value)
{
  TIMx->CCR_OC=TIMx->CCR_OC + oc_value;
}

uint32_t hal_get_oc()
{
  return TIMx->CCR_OC;
}
/*--------------------------------------------------------------------------*/
/**
 * Initialise the hardware
 * 
 */
int
hal_init(void) 
{
  uint16_t tmpccmrx = 0;        /** Capture/Compare Mode Register Value */
  uint16_t tmpccer = 0;         /** Capture/Compare Enabled Register Value */
  uint32_t tmpreg;              /** temporary register */

  /****** Conifgure the Timer *************************************************/
  /* Enable the clocksource */
  RCC->APB1ENR |= RCC_APB1ENR_TIMxEN;
  /* Set Timer to disabled,upcounting,Edge-aligned,no prescaler */
  TIMx->CR1 = 0;
  TIMx->ARR = 0xFFFFFFFF;
#if (TIM_PSC != 21)
#error Wrong Prescaler!
#endif
  TIMx->PSC = TIM_PSC - 1;
  TIMx->EGR = TIM_EGR_UG;

  /* Reset/Disable All Capture and Compare Units */
  TIMx->CCER = 0;

  /* Setup Input Capture - load register, clear all bits regarding out
   * IC channel, set predefined values and write back
   */
  tmpccmrx = TIMx->CCMR_IC;
  tmpccmrx &= ~CCMR_IC_MASK;
  tmpccmrx |= (CCMR_IC_CHAN | CCMR_IC_FILTER | CCMR_IC_PRESCALER);
  TIMx->CCMR_IC = tmpccmrx;

  tmpccer = TIMx->CCER;
  tmpccer &= ~CCER_IC_MASK;
  tmpccer |=  (CCER_IC_CCE | CCER_IC_CCP);
  TIMx->CCER = tmpccer;
  
  /* Setup Output Compare -  load register, clear all bits regarding our
   * OC channel, set predefined values and write back
   */
  tmpccmrx = TIMx->CCMR_OC;
  tmpccmrx &= ~CCMR_OC_MASK;
  tmpccmrx |= (CCMR_OC_CHAN | CCMR_OC_FE | CCMR_OC_PE |  CCMR_OC_M | CCMR_OC_CE);
  TIMx->CCMR_OC = tmpccmrx;

  tmpccer = TIMx->CCER;
  tmpccer &= ~CCER_OC_MASK;
  tmpccer |=  (CCER_OC_CCE | CCER_OC_CCP);
  TIMx->CCER = tmpccer;

  /* Setup TX_MODE_Timer -  load register, clear all bits regarding our
   * OC channel, set predefined values and write back
   */
  tmpccmrx = TIMx->CCMR_TX_MODE;
  tmpccmrx &= ~CCMR_TX_MODE_MASK;
  tmpccmrx |= (CCMR_TX_MODE_CHAN | CCMR_TX_MODE_FE | CCMR_TX_MODE_PE |  CCMR_TX_MODE_M | CCMR_TX_MODE_CE);
  TIMx->CCMR_TX_MODE = tmpccmrx;

  tmpccer = TIMx->CCER;
  tmpccer &= ~CCER_TX_MODE_MASK;
  tmpccer |=  (CCER_TX_MODE_CCE | CCER_TX_MODE_CCP);
  TIMx->CCER = tmpccer;


#ifndef SLOTTED_KOORDINATOR
  /* Setup BEACON_MISSED_Time -  load register, clear all bits regarding our
   * OC channel, set predefined values and write back
   */
  tmpccmrx = TIMx->CCMR_BEACON_MISSED;
  tmpccmrx &= ~CCMR_BEACON_MISSED_MASK;
  tmpccmrx |= (CCMR_BEACON_MISSED_CHAN | CCMR_BEACON_MISSED_FE | CCMR_BEACON_MISSED_PE |  CCMR_BEACON_MISSED_M | CCMR_BEACON_MISSED_CE);
  TIMx->CCMR_BEACON_MISSED = tmpccmrx;

  tmpccer = TIMx->CCER;
  tmpccer &= ~CCER_BEACON_MISSED_MASK;
  tmpccer |=  (CCER_BEACON_MISSED_CCE | CCER_BEACON_MISSED_CCP);
  TIMx->CCER = tmpccer;
#endif

  /**
   * Set special function for Input and Output
   * 
   */
  //RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;         /* Enable GPIOC clock                 */
  //  GPIOA->MODER  &= ~TIM_GPIO_MASk;
  //  GPIOA->MODER  |= TIM_GPIO_MODE;
  //  GPIOA->AFR[1] |= TIM_GPIO_AF;          /* PD8 USART3_Tx, PD9 USART3_Rx (AF7) */

  /* enable GPIO Clocks */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /*
   * Initialise the Pins used for the AT86RF231
   * We need:
   * RST
   * IRQ
   * SLP_TR
   * SPI PINS (MISO MOSI SEL SCLK)
   */
  /* Enable the SPI clock */
  RCC->SPI_APBxENR |= RCC_APBxENR_SPIxEN;

  /******************************************************************************
   * SPI Configuration
   ******************************************************************************
   * Selecet the Special Functions to connect the Pins to the SPI Controller
   * blank out function selections and then rewrite with correct Functions
   ******************************************************************************/
  SCKPORT->AFR[SCKPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)SCKPIN & (uint32_t)0x07) * 4)) ;
  SCKPORT->AFR[SCKPIN >> 0x03] |= ((uint32_t)SPIx_AF << ((uint32_t)((uint32_t)SCKPIN & (uint32_t)0x07) * 4)) ;
  MISOPORT->AFR[MISOPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)MISOPIN & (uint32_t)0x07) * 4)) ;
  MISOPORT->AFR[MISOPIN >> 0x03] |= ((uint32_t)SPIx_AF << ((uint32_t)((uint32_t)MISOPIN & (uint32_t)0x07) * 4)) ;
  MOSIPORT->AFR[MOSIPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)MOSIPIN & (uint32_t)0x07) * 4)) ;
  MOSIPORT->AFR[MOSIPIN >> 0x03] |= ((uint32_t)SPIx_AF << ((uint32_t)((uint32_t)MOSIPIN & (uint32_t)0x07) * 4)) ;
  /******************************************************************************
   * SSPORT configuration
   ******************************************************************************/
  SSPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)SSPIN * 2));	/* unselect mode */
  SSPORT->MODER |= ((uint32_t)0x1 << ((uint32_t)SSPIN * 2));	/* select output mode */
  SSPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)SSPIN * 2)); /* reset speed */
  SSPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)SSPIN * 2)); /* set speed to 50 MHz */
  SSPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)SSPIN);	/* select push-pull */
  SSPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)SSPIN * 2)); /* no pull-up/-down */
  HAL_SS_HIGH();

  /******************************************************************************
   * SCKPORT configuration
   ******************************************************************************/
  SCKPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)SCKPIN * 2));	/* unselect mode */
  SCKPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)SCKPIN * 2));	/* select AF mode */
  SCKPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)SCKPIN * 2)); /* reset speed */
  SCKPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)SCKPIN * 2)); /* set speed to 50 MHz */
  SCKPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)SCKPIN);	/* select push-pull */
  SCKPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)SCKPIN * 2)); /* no pull-up/-down */
  SCKPORT->PUPDR |= ((uint32_t)0x2 << ((uint32_t)SCKPIN * 2)); /* set pull-down */ 

  /******************************************************************************
   * MISOPORT configuration
   ******************************************************************************/
  MISOPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)MISOPIN * 2));	/* unselect mode */
  MISOPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)MISOPIN * 2));	/* select AF mode */
  MISOPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)MISOPIN * 2)); /* reset speed */
  MISOPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)MISOPIN * 2)); /* set speed to 50 MHz */
  MISOPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)MISOPIN);	/* select push-pull */
  MISOPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)MISOPIN * 2)); /* no pull-up/-down */
  MISOPORT->PUPDR |= ((uint32_t)0x1 << ((uint32_t)MISOPIN * 2)); /* set pull-up */ 

  /******************************************************************************
   * MOSIPORT configuration
   ******************************************************************************/
  MOSIPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)MOSIPIN * 2));	/* unselect mode */
  MOSIPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)MOSIPIN * 2));	/* select AF mode */
  MOSIPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)MOSIPIN * 2)); /* reset speed */
  MOSIPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)MOSIPIN * 2)); /* set speed to 50 MHz */
  MOSIPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)MOSIPIN);	/* select push-pull */
  MOSIPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)MOSIPIN * 2)); /* no pull-up/-down */
  MOSIPORT->PUPDR |= ((uint32_t)0x2 << ((uint32_t)MOSIPIN * 2)); /* set pull-down */ 
  
  /******************************************************************************
   * SPI Controller configuration 
   *----------------------------------------------------------------------------*
   * 7.5 MHz max.
   * MSB first
   * CPOL = 0
   * CPHA = 0
   ******************************************************************************/
  RCC->SPI_APBxRSTR |= RCC_APBxRSTR_SPIxRST;        /** Enable SPI reset state */
  RCC->SPI_APBxRSTR &= ~RCC_APBxRSTR_SPIxRST;       /** Release SPI from reset state */
  /* set MSB first, CPOL=CPHA=0, Baud=fPCLK/16 ~= 5.3 MHz, Master */
  SPIx->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | (SPI_CR1_BR_1 | SPI_CR1_BR_0) | SPI_CR1_MSTR); 
  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPIx->I2SCFGR &= (uint16_t)~SPI_I2SCFGR_I2SMOD;

  SPIx->CR1 |= SPI_CR1_SPE;                         /** Enable SPIx */


  /******************************************************************************
   * Controll PIN  Configuration
   ******************************************************************************/
  /** RST pin configuration */
  RSTPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)RSTPIN * 2));	/* unselect mode */
  RSTPORT->MODER |= ((uint32_t)0x1 << ((uint32_t)RSTPIN * 2));	/* select output mode */
  RSTPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)RSTPIN * 2)); /* reset speed */
  RSTPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)RSTPIN * 2)); /* set speed to 50 MHz */
  RSTPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)RSTPIN);	/* select push-pull */
  RSTPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)RSTPIN * 2)); /* no pull-up/-down */
  RSTPORT->ODR &= ~((uint32_t)0x1 << (uint32_t)RSTPIN);	/* set output low */
  /** SLP_TR pin configuration */
  SLPTRPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)SLPTRPIN * 2));	/* unselect mode */
  SLPTRPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)SLPTRPIN * 2));	/* select output mode */
  SLPTRPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)SLPTRPIN * 2)); /* reset speed */
  SLPTRPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)SLPTRPIN * 2)); /* set speed to 50 MHz */
  SLPTRPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)SLPTRPIN);	/* select push-pull */
  SLPTRPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)SLPTRPIN * 2)); /* no pull-up/-down */
  SLPTRPORT->ODR &= ~((uint32_t)0x1 << (uint32_t)SLPTRPIN);	/* set output low */
  /** IRQ pin configuration */
  IRQPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)IRQPIN * 2));	/* unselect mode */
  IRQPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)IRQPIN * 2));	/* select AF mode */
  IRQPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)IRQPIN * 2)); /* reset speed */
  IRQPORT->OSPEEDR |= ((uint32_t)0x3 << ((uint32_t)IRQPIN * 2)); /* set speed to 100 MHz */
  IRQPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)IRQPIN);	/* select push-pull */
  IRQPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)IRQPIN * 2)); /* no pull-up/-down */
  /** Connect IRQ pin to AF1 (TIM1/TIM2) or AF2 (TIM3-5) */
  /* blank out function selections on IRQ pin and then rewrite with TIMx_AF */
  IRQPORT->AFR[IRQPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)IRQPIN & (uint32_t)0x07) * 4)) ;
  IRQPORT->AFR[IRQPIN >> 0x03] |= ((uint32_t)TIMx_AF << ((uint32_t)((uint32_t)IRQPIN & (uint32_t)0x07) * 4)) ;
  /** Connect SLP_TR pin to AF1 (TIM1/TIM2) or AF2 (TIM3-5) */
  /* blank out function selections on IRQ pin and then rewrite with TIMx_AF */
  SLPTRPORT->AFR[SLPTRPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)SLPTRPIN & (uint32_t)0x07) * 4)) ;
  SLPTRPORT->AFR[SLPTRPIN >> 0x03] |= ((uint32_t)TIMx_AF << ((uint32_t)((uint32_t)SLPTRPIN & (uint32_t)0x07) * 4)) ;

  /******************************************************************************
   * Initialise Random Number Generator to simulate the effect of
   * jitter at Koordinator side
   ******************************************************************************/
#ifdef JITTER_SIMULATION  
#ifdef SLOTTED_KOORDINATOR
  /* initialise with seed 0 - stm32f4 generates random number via
   * noise, we dont need a seed */
  random_init(0); 
#endif /* SLOTTED_KOORDINATOR */
#endif /* JITTER_SIMULATION */


  /****************************************************************************
   * Configure DMA Controller
   *****************************************************************************/
  RCC->AHB1ENR |= RCC_AHB1_SPI_DMAx; /* enable DMA clock */

  /****** TX STREAM ************************************************************/
  /* Get the SPI_DMAx_STREAM_TX CR value */
  tmpreg = SPI_DMAx_STREAM_TX->CR;
  /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
  tmpreg &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                         DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | \
                         DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | \
                         DMA_SxCR_DIR));
  /****** Configure SPI_DMAx_STREAM_TX: ****************************************
   * Set CHSEL bits to SPI_DMAx_CHANNEL
   * Set DIR bits to 01 (Mem->Periph) 
   * Set PINC bit to 0 (disable) 
   * Set MINC bit to 1 (enable) 
   * Set PSIZE bits to 00 (byte)
   * Set MSIZE bits to 00 (byte)
   * Set CIRC bit to 0 (disable)
   * Set PL bits to 00 (low)
   * Set MBURST bits to 00 (single)
   * Set PBURST bits to 00 (single) 
   ******************************************************************************/
  tmpreg |= SPI_DMAx_CHANNEL | DMA_SxCR_DIR_0 | DMA_SxCR_MINC;
  /* Write to SPI_DMAx_STREAM_TX CR register */
  SPI_DMAx_STREAM_TX->CR = tmpreg;
  /* Clear DMDIS and FTH bits in the SPI_DMAx_STREAM_TX FCR register */
  SPI_DMAx_STREAM_TX->FCR &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  /* Write to SPI_DMAx_STREAM_TX PAR */
  SPI_DMAx_STREAM_TX->PAR = SPIx_BASE+0x0C;

  /****** RX STREAM **/
  /* Get the SPI_DMAx_STREAM_RX CR value */
  tmpreg = SPI_DMAx_STREAM_RX->CR;
  /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
  tmpreg &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                         DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | \
                         DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | \
                         DMA_SxCR_DIR));
  /* Configure SPI_DMAx_STREAM_RX: */
  /* Set CHSEL bits to SPI_DMAx_CHANNEL */
  /* Set DIR bits to 00 (Periph->Mem) */
  /* Set PINC bit to 0 (disable) */
  /* Set MINC bit to 1 (enable) */
  /* Set PSIZE bits to 00 (byte) */
  /* Set MSIZE bits to 00 (byte) */
  /* Set CIRC bit to 0 (disable) */
  /* Set PL bits to 00 (low) */
  /* Set MBURST bits to 00 (single) */
  /* Set PBURST bits to 00 (single) */
  /* Enable Transfer complete interrupt */
  tmpreg |= SPI_DMAx_CHANNEL | DMA_SxCR_MINC | DMA_SxCR_TCIE;
  /* Write to SPI_DMAx_STREAM_RX CR register */
  SPI_DMAx_STREAM_RX->CR = tmpreg;
  /* Clear DMDIS and FTH bits in the SPI_DMAx_STREAM_TX FCR register */
  SPI_DMAx_STREAM_RX->FCR &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  /* Write to SPI_DMAx_STREAM_TX PAR */
  SPI_DMAx_STREAM_RX->PAR = SPIx_BASE+0x0C;


  /****************************************************************************
   * Enable and Reset Interrupts
   *****************************************************************************/
  /* Enable and reset the Timer Interrupts */
  TIMx->SR &= ~TIM_IC_IRQ_FLAG;
  TIMx->SR &= ~TIM_OC_IRQ_FLAG;
  TIMx->SR &= ~TIM_TX_MODE_IRQ_FLAG;

  TIMx->DIER |= TIM_OC_IE;
  TIMx->DIER |= TIM_IC_IE;
  TIMx->DIER |= TIM_TX_MODE_IE;

#ifndef SLOTTED_KOORDINATOR
  TIMx->SR &= ~TIM_BEACON_MISSED_IRQ_FLAG;
  TIMx->DIER |= TIM_BEACON_MISSED_IE;
#endif

  IRQ_init_enable(TIMx_IRQn,1,0);

  return 1;
}

/*---------------------------------------------------------------*/
int
hal_start_counter()
{
  /* TIM enable counter */
  TIMx->CR1 |= TIM_CR1_CEN;

  return 1;
}

/*---------------------------------------------------------------*/
int
hal_stop_counter()
{
  /* TIM enable counter */
  TIMx->CR1 &= ~TIM_CR1_CEN;
  return 1;
}

/*---------------------------------------------------------------*/
int
hal_reset_counter()
{
  hal_stop_counter();
  TIMx->CNT = 0;
  hal_start_counter();
  return 1;
}

/*---------------------------------------------------------------*/
/**
 * set a Value for the next transmission time
 */
int
hal_set_TX_Timer(uint32_t time)
{
  TIMx->CCR_OC=time;
  return 1;
}

/*---------------------------------------------------------------*/
/**
 * update the value of the next transmission time using the 
 * old_time + new time
 */
int
hal_update_TX_Timer(uint32_t time)
{
  TIMx->CCR_OC=TIMx->CCR_OC + time;
  return 1;
}

/*---------------------------------------------------------------*/
/**
 * set a Value for the next TX_MODE state change time
 */
int
hal_set_TX_Mode_Timer(uint32_t time)
{
  TIMx->CCR_TX_MODE=time;
  return 1;
}

/*---------------------------------------------------------------*/
/**
 * update the value of the next TX_MODE state change time using the 
 * old_time + new time
 */
int
hal_update_TX_Mode_Timer(uint32_t time)
{
  TIMx->CCR_TX_MODE=TIMx->CCR_TX_MODE + time;
  return 1;
}

/*---------------------------------------------------------------*/
/**
 * set a Value for the next transmission time
 */
int
hal_set_Beacon_Missed_Timer(uint32_t time)
{
  TIMx->CCR_BEACON_MISSED=time;
  return 1;
}

/*---------------------------------------------------------------*/
/**
 * update the value of the next transmission time using the 
 * old_time + new time
 */
int
hal_update_Beacon_Missed_Timer(uint32_t time)
{
  TIMx->CCR_BEACON_MISSED=TIMx->CCR_BEACON_MISSED + time;
  return 1;
}
