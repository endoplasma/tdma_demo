#include "contiki-conf.h"

/*============================ INCLUDE =======================================*/
#include <stdlib.h>

#include "uwb_hal.h"

/*
 * KUSZ development platform
 */
#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include <nvic.h>                  /* STM32F4xx Definitions              */
#include <clock.h>

#include <string.h>

/* used for storing DMA data */
static uint8_t dma_dummy;
static uint16_t hal_system_time = 0;

#define delay_us( us )   ( clock_delay_usec( us ) )


void hal_time_isr_func(void)
{
  hal_system_time++;
}

extern void uwb_tx_interrupt(void);
extern void uwb_rx_interrupt(void);
extern void uwb_dma_interrupt(void);

void TIMx_IRQHandler(void)
{
  volatile uint32_t capture;
  if (TIMx->SR & TIM_SR_CCxIF_IRQ0)	/* IRQ0 */
  {
    capture = TIMx->CCRx_IRQ0;
    uwb_rx_interrupt();
  }
  if (TIMx->SR & TIM_SR_CCxIF_IRQ1)	/* IRQ1 */
  {
    capture = TIMx->CCRx_IRQ1;
    uwb_tx_interrupt();
  }
  else if (TIMx->SR & TIM_SR_UIF)
  {
    TIMx->SR &= ~TIM_SR_UIF;
    hal_time_isr_func();
  }
}

void hal_init(void)
{
  uint32_t tmpreg;
  /*Reset variables used in file.*/
  hal_system_time = 0;

  /*!< Enable the SPI clock */
  RCC->SPI_APBxENR |= RCC_APBxENR_SPIxEN; /* Enable SPIx clock */

  /*!< Enable GPIO clocks */
  /* notice that this depends on the used SPI-Port and other signal-pins!!! */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOxEN; /* Enable GPIOx clock */

  /*!< SPI pins configuration *************************************************/

  /*!< Connect pins to AF5 (SPI2) or AF10 (SPI1) */
  /* blank out function selections and then rewrite with SPIx_AF */
  SCKPORT->AFR[SCKPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)SCKPIN & (uint32_t)0x07) * 4)) ;
  SCKPORT->AFR[SCKPIN >> 0x03] |= ((uint32_t)SPIx_AF << ((uint32_t)((uint32_t)SCKPIN & (uint32_t)0x07) * 4)) ;
  MISOPORT->AFR[MISOPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)MISOPIN & (uint32_t)0x07) * 4)) ;
  MISOPORT->AFR[MISOPIN >> 0x03] |= ((uint32_t)SPIx_AF << ((uint32_t)((uint32_t)MISOPIN & (uint32_t)0x07) * 4)) ;
  MOSIPORT->AFR[MOSIPIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)MOSIPIN & (uint32_t)0x07) * 4)) ;
  MOSIPORT->AFR[MOSIPIN >> 0x03] |= ((uint32_t)SPIx_AF << ((uint32_t)((uint32_t)MOSIPIN & (uint32_t)0x07) * 4)) ;

  /*!< SPI SS pin configuration */
  SSPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)SSPIN * 2));	/* unselect mode */
  SSPORT->MODER |= ((uint32_t)0x1 << ((uint32_t)SSPIN * 2));	/* select output mode */
  SSPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)SSPIN * 2)); /* reset speed */
  SSPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)SSPIN * 2)); /* set speed to 50 MHz */
  SSPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)SSPIN);	/* select push-pull */
  SSPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)SSPIN * 2)); /* no pull-up/-down */
  HAL_SS_HIGH();

  /*!< SPI SCK pin configuration */
  SCKPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)SCKPIN * 2));	/* unselect mode */
  SCKPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)SCKPIN * 2));	/* select AF mode */
  SCKPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)SCKPIN * 2)); /* reset speed */
  SCKPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)SCKPIN * 2)); /* set speed to 50 MHz */
  SCKPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)SCKPIN);	/* select push-pull */
  SCKPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)SCKPIN * 2)); /* no pull-up/-down */
  SCKPORT->PUPDR |= ((uint32_t)0x2 << ((uint32_t)SCKPIN * 2)); /* set pull-down */ 

  /*!< SPI MISO pin configuration */
  MISOPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)MISOPIN * 2));	/* unselect mode */
  MISOPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)MISOPIN * 2));	/* select AF mode */
  MISOPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)MISOPIN * 2)); /* reset speed */
  MISOPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)MISOPIN * 2)); /* set speed to 50 MHz */
  MISOPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)MISOPIN);	/* select push-pull */
  MISOPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)MISOPIN * 2)); /* no pull-up/-down */
  MISOPORT->PUPDR |= ((uint32_t)0x1 << ((uint32_t)MISOPIN * 2)); /* set pull-up */ 

  /*!< SPI MOSI pin configuration */
  MOSIPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)MOSIPIN * 2));	/* unselect mode */
  MOSIPORT->MODER |= ((uint32_t)0x2 << ((uint32_t)MOSIPIN * 2));	/* select AF mode */
  MOSIPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)MOSIPIN * 2)); /* reset speed */
  MOSIPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)MOSIPIN * 2)); /* set speed to 50 MHz */
  MOSIPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)MOSIPIN);	/* select push-pull */
  MOSIPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)MOSIPIN * 2)); /* no pull-up/-down */
  MOSIPORT->PUPDR |= ((uint32_t)0x2 << ((uint32_t)MOSIPIN * 2)); /* set pull-down */ 

  /*!< SPI configuration *************************************************/
  /*!< Enable SPI2 reset state */
  RCC->SPI_APBxRSTR |= RCC_APBxRSTR_SPIxRST;
  /*!< Release SPI2 from reset state */
  RCC->SPI_APBxRSTR &= ~RCC_APBxRSTR_SPIxRST;

#if PLATFORM_TYPE == STM3240G_EVAL
  SPIx->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_MSTR | SPI_CR1_CPHA); /* set MSB first, CPOL=0, CPHA=1, Baud=fPCLK/4, Master */
#else
  SPIx->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_MSTR | SPI_CR1_CPHA); /* set MSB first, CPOL=0, CPHA=1, Baud=fPCLK/8, Master */
#endif

  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPIx->I2SCFGR &= (uint16_t)~SPI_I2SCFGR_I2SMOD;

  /*!< Enable SPIx */
  SPIx->CR1 |= SPI_CR1_SPE;

  /*!< Control pins configuration *************************************************/

  /*!< RST pin configuration */
  RSTPORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)RSTPIN * 2));	/* unselect mode */
  RSTPORT->MODER |= ((uint32_t)0x1 << ((uint32_t)RSTPIN * 2));	/* select output mode */
  RSTPORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)RSTPIN * 2)); /* reset speed */
  RSTPORT->OSPEEDR |= ((uint32_t)0x2 << ((uint32_t)RSTPIN * 2)); /* set speed to 50 MHz */
  RSTPORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)RSTPIN);	/* select push-pull */
  RSTPORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)RSTPIN * 2)); /* no pull-up/-down */
  RSTPORT->ODR &= ~((uint32_t)0x1 << (uint32_t)RSTPIN);	/* set output low */

  /*!< IRQ0 pin configuration */
  IRQ0PORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)IRQ0PIN * 2));	/* unselect mode */
  IRQ0PORT->MODER |= ((uint32_t)0x2 << ((uint32_t)IRQ0PIN * 2));	/* select AF mode */
  IRQ0PORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)IRQ0PIN * 2)); /* reset speed */
  IRQ0PORT->OSPEEDR |= ((uint32_t)0x3 << ((uint32_t)IRQ0PIN * 2)); /* set speed to 100 MHz */
  IRQ0PORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)IRQ0PIN);	/* select push-pull */
  IRQ0PORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)IRQ0PIN * 2)); /* no pull-up/-down */

  /*!< Connect IRQ0 pin to AF1 (TIM1/TIM2) or AF2 (TIM3-5) */
  /* blank out function selections on IRQ pin and then rewrite with TIMx_AF */
  IRQ0PORT->AFR[IRQ0PIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)IRQ0PIN & (uint32_t)0x07) * 4)) ;
  IRQ0PORT->AFR[IRQ0PIN >> 0x03] |= ((uint32_t)TIMx_AF << ((uint32_t)((uint32_t)IRQ0PIN & (uint32_t)0x07) * 4)) ;

  /*!< IRQ1 pin configuration */
  IRQ1PORT->MODER &= ~((uint32_t)0x3 << ((uint32_t)IRQ1PIN * 2));	/* unselect mode */
  IRQ1PORT->MODER |= ((uint32_t)0x2 << ((uint32_t)IRQ1PIN * 2));	/* select AF mode */
  IRQ1PORT->OSPEEDR &= ~((uint32_t)0x3 << ((uint32_t)IRQ1PIN * 2)); /* reset speed */
  IRQ1PORT->OSPEEDR |= ((uint32_t)0x3 << ((uint32_t)IRQ1PIN * 2)); /* set speed to 100 MHz */
  IRQ1PORT->OTYPER &= ~((uint32_t)0x1 << (uint32_t)IRQ1PIN);	/* select push-pull */
  IRQ1PORT->PUPDR &= ~((uint32_t)0x3 << ((uint32_t)IRQ1PIN * 2)); /* no pull-up/-down */

  /*!< Connect IRQ1 pin to AF1 (TIM1/TIM2) or AF2 (TIM3-5) */
  /* blank out function selections on IRQ pin and then rewrite with TIMx_AF */
  IRQ1PORT->AFR[IRQ1PIN >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)IRQ1PIN & (uint32_t)0x07) * 4)) ;
  IRQ1PORT->AFR[IRQ1PIN >> 0x03] |= ((uint32_t)TIMx_AF << ((uint32_t)((uint32_t)IRQ1PIN & (uint32_t)0x07) * 4)) ;

  /*!< Timer configuration *************************************************/
  /* TIMx clock enable */
  RCC->APB1ENR |= RCC_APB1ENR_TIMxEN;

  /* Disable the IRQ-channels: Reset the CCxE Bit */
  TIMx->CCER &= ~(TIM_CCER_CCxE_IRQ0);
  TIMx->CCER &= ~(TIM_CCER_CCxE_IRQ1);

  /* Write to TIMx CCMRx and CCER registers */
  TIMx->CCMRx_IRQ0 &= ~CCMRx_CCx_MASK_IRQ0; /* reset all bits regarding CCx */
  TIMx->CCMRx_IRQ0 |= TIM_CCMRx_IC_TI_MAP_IRQ0; /* map ICx to TIx, no filter, no prescaler */
  TIMx->CCMRx_IRQ1 &= ~CCMRx_CCx_MASK_IRQ1; /* reset all bits regarding CCx */
  TIMx->CCMRx_IRQ1 |= TIM_CCMRx_IC_TI_MAP_IRQ1; /* map ICx to TIx, no filter, no prescaler */

  TIMx->CCER &= ~CCER_CCx_MASK_IRQ0; /* reset all bits regarding CCx */
  TIMx->CCER |= TIM_CCER_CCxE_IRQ0; /* rising & enable CCx */
  TIMx->CCER &= ~CCER_CCx_MASK_IRQ1; /* reset all bits regarding CCx */
  TIMx->CCER |= TIM_CCER_CCxE_IRQ1; /* rising & enable CCx */

  IRQ_init_enable(TIMx_IRQn,1,0);

  /* TIM enable counter */
  TIMx->CR1 |= TIM_CR1_CEN;

  HAL_ENABLE_OVERFLOW_INTERRUPT();
  /* Enable the CCx Interrupt Requests */

  /*!< DMA configuration *************************************************/
  RCC->AHB1ENR |= RCC_AHB1_SPI_DMAx; /* enable DMA clock */

  /* TX STREAM */
  /* Get the SPI_DMAx_STREAM_TX CR value */
  tmpreg = SPI_DMAx_STREAM_TX->CR;
  /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
  tmpreg &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                         DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | \
                         DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | \
                         DMA_SxCR_DIR));
  /* Configure SPI_DMAx_STREAM_TX: */
  /* Set CHSEL bits to SPI_DMAx_CHANNEL */
  /* Set DIR bits to 01 (Mem->Periph) */
  /* Set PINC bit to 0 (disable) */
  /* Set MINC bit to 1 (enable) */
  /* Set PSIZE bits to 00 (byte) */
  /* Set MSIZE bits to 00 (byte) */
  /* Set CIRC bit to 0 (disable) */
  /* Set PL bits to 00 (low) */
  /* Set MBURST bits to 00 (single) */
  /* Set PBURST bits to 00 (single) */
  tmpreg |= SPI_DMAx_CHANNEL | DMA_SxCR_DIR_0 | DMA_SxCR_MINC;
  /* Write to SPI_DMAx_STREAM_TX CR register */
  SPI_DMAx_STREAM_TX->CR = tmpreg;
  /* Clear DMDIS and FTH bits in the SPI_DMAx_STREAM_TX FCR register */
  SPI_DMAx_STREAM_TX->FCR &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  /* Write to SPI_DMAx_STREAM_TX PAR */
  SPI_DMAx_STREAM_TX->PAR = SPIx_BASE+0x0C;

  /* RX STREAM */
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

  IRQ_init_enable(SPI_DMAx_STREAM_RX_IRQn,0,0);
}

void SPI_DMAx_STREAM_RX_IRQHandler(void)
{
  /* Clear Transfer Complete Interrupt Flag */
  SPI_DMA_IFCR_RX |= SPI_DMA_IFCR_CTCIF_RX;

  /* Release SS */
  HAL_SS_HIGH();

  uwb_dma_interrupt();
}

uint8_t hal_spi_dma_busy(void)
{
	return (((SPI_DMAx_STREAM_TX->CR & DMA_SxCR_EN) == DMA_SxCR_EN) || ((SPI_DMAx_STREAM_RX->CR & DMA_SxCR_EN) == DMA_SxCR_EN));
}

void hal_spi_dma_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, uint8_t length)
{
  /* use DMA to write buffer to SPI / read buffer from SPI
   * - received data can be placed in the same buffer if pointers are the same
   * - received data gets dumped if rx_buffer is NULL
   * - zeros are send if tx_buffer is NULL
   * 
   * CS gets de-asserted by the RX-interrupt...
   */

  /* Assert SS */
  HAL_SS_LOW();

  /* Disable SPI_DMAx_STREAM_TX by clearing EN bit */
  SPI_DMAx_STREAM_TX->CR &= ~DMA_SxCR_EN;
  /* Disable SPI_DMAx_STREAM_RX by clearing EN bit */
  SPI_DMAx_STREAM_RX->CR &= ~DMA_SxCR_EN;

  /* Clear Transfer Complete Interrupt Flag - important for following DMA transactions */
  SPI_DMA_IFCR_TX |= SPI_DMA_IFCR_CTCIF_TX;
  SPI_DMA_IFCR_RX |= SPI_DMA_IFCR_CTCIF_RX;

  /* Write to SPI_DMAx_STREAM_TX NDTR register */
  SPI_DMAx_STREAM_TX->NDTR = length;
  /* Write to SPI_DMAx_STREAM_RX NDTR register */
  SPI_DMAx_STREAM_RX->NDTR = length;

  /* Write to SPI_DMAx_STREAM_TX M0AR */
  if (tx_buffer == NULL)
  {
    /* use dummy as tx-pointer / don't increment address */
    SPI_DMAx_STREAM_TX->M0AR = (uint32_t)&dma_dummy;
    SPI_DMAx_STREAM_TX->CR &= ~DMA_SxCR_MINC;
  }
  else
  {
    /* use given tx-pointer / increment address */
    SPI_DMAx_STREAM_TX->M0AR = (uint32_t)tx_buffer;
    SPI_DMAx_STREAM_TX->CR |= DMA_SxCR_MINC;
  }
  /* Write to SPI_DMAx_STREAM_RX M0AR */
  if (rx_buffer == NULL)
  {
    /* use dummy as rx-pointer / don't increment address */
    SPI_DMAx_STREAM_RX->M0AR = (uint32_t)&dma_dummy;
    SPI_DMAx_STREAM_RX->CR &= ~DMA_SxCR_MINC;
  }
  else
  {
    /* use given rx-pointer / increment address */
    SPI_DMAx_STREAM_RX->M0AR = (uint32_t)rx_buffer;
    SPI_DMAx_STREAM_RX->CR |= DMA_SxCR_MINC;
  }

    /* Enable SPI_DMAx_STREAM_TX by setting EN bit */
    SPI_DMAx_STREAM_TX->CR |= DMA_SxCR_EN;
    /* Enable SPI_DMAx_STREAM_RX by setting EN bit */
    SPI_DMAx_STREAM_RX->CR |= DMA_SxCR_EN;

    /* Enable Tx buffer DMA */
    SPIx->CR2 |= SPI_CR2_TXDMAEN;
    /* Enable Rx buffer DMA */
    SPIx->CR2 |= SPI_CR2_RXDMAEN;
}
