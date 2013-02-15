#ifndef UWB_HAL_H
#define UWB_HAL_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include "contiki-conf.h"
#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */

#define STM3240G_EVAL      7
#define STM32F4_Discovery  8

#if PLATFORM_TYPE == STM3240G_EVAL
/* KUSZ development platform */
/* using SPI2 on PB12-15 - unsoldered R41 & R53 to free pins - therefore USB HS OTG & Ethernet (can be fixed - reassign pin) not working! */
#define IRQ0PORT              GPIOB
#define IRQ0PIN               10 /* PB10/TIM2_CH3 */
#define IRQ1PORT              GPIOB
#define IRQ1PIN               11 /* PB11/TIM2_CH4 */
#define RSTPORT               GPIOB
#define RSTPIN                0 /* PB0 */

#define SPIPORT               GPIOB
#define SSPIN                 12 /* PB12/SPI2_NSS */
#define SCKPIN                13 /* PB13/SPI2_SCK */
#define MISOPIN               14 /* PB14/SPI2_MISO */
#define MOSIPIN               15 /* PB15/SPI2_MOSI */

#define RCC_AHB1ENR_GPIOxEN   RCC_AHB1ENR_GPIOBEN

#define SPIx                  SPI2
#define SPI_APBxENR           APB1ENR
#define RCC_APBxENR_SPIxEN    RCC_APB1ENR_SPI2EN
#define SPI_APBxRSTR          APB1RSTR
#define RCC_APBxRSTR_SPIxRST  ((uint32_t)0x00004000)
#define SPIx_AF               5
#define SPIx_BASE             SPI2_BASE

#define SPI_DMAx                        DMA1
#define RCC_AHB1_SPI_DMAx               RCC_AHB1ENR_DMA1EN
#define SPI_DMAx_STREAM_RX              DMA1_Stream3
#define SPI_DMAx_STREAM_TX              DMA1_Stream4
#define SPI_DMA_IFCR_RX                 SPI_DMAx->LIFCR
#define SPI_DMA_IFCR_TX                 SPI_DMAx->HIFCR
#define SPI_DMA_IFCR_CTCIF_RX           DMA_LIFCR_CTCIF3
#define SPI_DMA_IFCR_CTCIF_TX           DMA_HIFCR_CTCIF4
#define SPI_DMAx_CHANNEL                (0)
#define SPI_DMAx_STREAM_RX_IRQn         DMA1_Stream3_IRQn
#define SPI_DMAx_STREAM_RX_IRQHandler   DMA1_Stream3_IRQHandler

#define TIMx                  TIM2
#define RCC_APB1ENR_TIMxEN    RCC_APB1ENR_TIM2EN
#define TIMx_IRQn             TIM2_IRQn
#define TIMx_IRQHandler       TIM2_IRQHandler
#define TIMx_AF               1
#define CCRx_IRQ0                  CCR3
#define CCMRx_IRQ0                 CCMR2
#define CCMRx_CCx_MASK_IRQ0        ((uint16_t)0x00FF)
#define TIM_CCMRx_IC_TI_MAP_IRQ0   TIM_CCMR2_CC3S_0
#define CCER_CCx_MASK_IRQ0         ((uint16_t)0x0F00)
#define TIM_CCER_CCxE_IRQ0         TIM_CCER_CC3E
#define TIM_SR_CCxIF_IRQ0          TIM_SR_CC3IF
#define TIM_DIER_CCxIE_IRQ0        TIM_DIER_CC3IE
#define CCRx_IRQ1                  CCR4
#define CCMRx_IRQ1                 CCMR2
#define CCMRx_CCx_MASK_IRQ1        ((uint16_t)0xFF00)
#define TIM_CCMRx_IC_TI_MAP_IRQ1   TIM_CCMR2_CC4S_0
#define CCER_CCx_MASK_IRQ1         ((uint16_t)0xF000)
#define TIM_CCER_CCxE_IRQ1         TIM_CCER_CC4E
#define TIM_SR_CCxIF_IRQ1          TIM_SR_CC4IF
#define TIM_DIER_CCxIE_IRQ1        TIM_DIER_CC4IE

#elif PLATFORM_TYPE == STM32F4_Discovery
/* KUSZ development platform */
/* using SPI1 on PA4-7 */
#define IRQPORT               GPIOA
#define IRQPIN                0 /* PA0/TIM5_CH1 -> "TRIG"  - required for input capture!!! */
#define RSTPORT               GPIOA
#define RSTPIN                1 /* PA1 -> "RST" */
#define SLPTRPORT             GPIOA
#define SLPTRPIN              3 /* PA3 -> "RT" */

#define SPIPORT               GPIOA
#define SSPIN                 4 /* PA4/SPI1_NSS -> "PROG" */
#define SCKPIN                5 /* PA5/SPI1_SCK -> "CCLK" */
#define MISOPIN               6 /* PA6/SPI1_MISO -> "DIN" */
#define MOSIPIN               7 /* PA7/SPI1_MOSI -> "B/P" */

#define RCC_AHB1ENR_GPIOxEN   RCC_AHB1ENR_GPIOAEN

#if RF231_HAS_PA
#define PAENPORT              GPIOC
#define PAENPIN               4
#define HGMPORT               GPIOC
#define HGMPIN                5
#define RCC_AHB1ENR_PAGPIOxEN RCC_AHB1ENR_GPIOCEN
#endif /* RF231_HAS_PA */

#define SPIx                  SPI1
#define SPI_APBxENR           APB2ENR
#define RCC_APBxENR_SPIxEN    RCC_APB2ENR_SPI1EN
#define SPI_APBxRSTR          APB2RSTR
#define RCC_APBxRSTR_SPIxRST  RCC_APB2RSTR_SPI1RST
#define SPIx_AF               5
#define SPIx_BASE             SPI1_BASE

#define SPI_DMAx                        DMA2
#define RCC_AHB1_SPI_DMAx               RCC_AHB1ENR_DMA2EN
#define SPI_DMAx_STREAM_RX              DMA2_Stream0
#define SPI_DMAx_STREAM_TX              DMA2_Stream3
#define SPI_DMA_LIFCR_CTCIF_RX          DMA_LIFCR_CTCIF0
#define SPI_DMA_LIFCR_CTCIF_TX          DMA_LIFCR_CTCIF3
#define SPI_DMAx_CHANNEL                (DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0)
#define SPI_DMAx_STREAM_RX_IRQn         DMA2_Stream0_IRQn
#define SPI_DMAx_STREAM_RX_IRQHandler   DMA2_Stream0_IRQHandler

#define TIMx                  TIM5
#define RCC_APB1ENR_TIMxEN    RCC_APB1ENR_TIM5EN
#define TIMx_IRQn             TIM5_IRQn
#define TIMx_IRQHandler       TIM5_IRQHandler
#define CCRx                  CCR1
#define CCMRx                 CCMR1
#define CCMRx_CCx_MASK        ((uint16_t)0x00FF)
#define TIM_CCMRx_IC_TI_MAP   TIM_CCMR1_CC1S_0
#define CCER_CCx_MASK         ((uint16_t)0x000F)
#define TIM_CCER_CCxE         TIM_CCER_CC1E
#define TIM_SR_CCxIF          TIM_SR_CC1IF
#define TIM_DIER_CCxIE        TIM_DIER_CC1IE
#define TIMx_AF               2

#else

#error "PLATFORM_TYPE undefined in uwb_hal.h"

#endif

/* For architectures that have all SPI signals on the same port */
#ifndef SSPORT
#define SSPORT SPIPORT
#endif

#ifndef SCKPORT
#define SCKPORT SPIPORT
#endif

#ifndef MOSIPORT
#define MOSIPORT SPIPORT
#endif

#ifndef MISOPORT
#define MISOPORT SPIPORT
#endif

#define HAL_SS_HIGH( )   ( SSPORT->BSRRL = ((uint32_t)1 << (uint32_t)SSPIN) ) /**< This macro pulls the SS pin high. */
#define HAL_SS_LOW( )    ( SSPORT->BSRRH = ((uint32_t)1 << (uint32_t)SSPIN) ) /**< This macro pulls the SS pin low. */

#define HAL_DEASSERT_RST( )   ( RSTPORT->BSRRL = ((uint32_t)1 << (uint32_t)RSTPIN) ) /**< This macro pulls the RST pin high. */
#define HAL_ASSERT_RST( )    ( RSTPORT->BSRRH = ((uint32_t)1 << (uint32_t)RSTPIN) ) /**< This macro pulls the RST pin low. */
#define HAL_GET_RST( )        !!( RSTPORT->IDR & ((uint32_t)1 << (uint32_t)RSTPIN) ) /**< Read current state of the RST pin (High/Low). */

/* IRQs are connected to TIMx_CHx */
/* disable CHx input-capture int to disable interrupts */
#define HAL_ENABLE_IRQ0( ) { TIMx->SR &= ~TIM_SR_CCxIF_IRQ0; TIMx->DIER |= TIM_DIER_CCxIE_IRQ0; }
#define HAL_DISABLE_IRQ0( ) ( TIMx->DIER &= ~TIM_DIER_CCxIE_IRQ0 )
#define HAL_ENABLE_IRQ1( ) { TIMx->SR &= ~TIM_SR_CCxIF_IRQ1; TIMx->DIER |= TIM_DIER_CCxIE_IRQ1; }
#define HAL_DISABLE_IRQ1( ) ( TIMx->DIER &= ~TIM_DIER_CCxIE_IRQ1 )

#define HAL_CHECK_IRQ0( ) ( (TIMx->SR & TIM_SR_CCxIF_IRQ0) == TIM_SR_CCxIF_IRQ0 )
#define HAL_CHECK_IRQ1( ) ( (TIMx->SR & TIM_SR_CCxIF_IRQ1) == TIM_SR_CCxIF_IRQ1 )

/* TIMx is used for radio timing */
/* disable TIMx update int to disable overflow interrupt */
#define HAL_ENABLE_OVERFLOW_INTERRUPT( ) ( TIMx->DIER |= TIM_DIER_UIE )
#define HAL_DISABLE_OVERFLOW_INTERRUPT( ) ( TIMx->DIER &= ~TIM_DIER_UIE )

#define HAL_SPI_TRANSFER_OPEN() { \
  HAL_SS_LOW(); /* Start the SPI transaction by pulling the Slave Select low. */
#define HAL_SPI_TRANSFER_WRITE(to_write) { volatile uint16_t dummy = SPIx->DR; while ((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE) {;}; SPIx->DR = (to_write); }
#define HAL_SPI_TRANSFER_WAIT()  { while ((SPIx->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {;}; }
#define HAL_SPI_TRANSFER_READ() (SPIx->DR)
#define HAL_SPI_TRANSFER_CLOSE() \
  while ((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE) {;}; /* see RM0090 - 25.3.8 */ \
  while ((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY) {;}; \
  HAL_SS_HIGH(); /* End the transaction by pulling the Slave Select High. */ \
}
#define HAL_SPI_TRANSFER(to_write) (spiWrite(to_write))
__INLINE uint8_t spiWrite(uint8_t byte)
{
	HAL_SPI_TRANSFER_WRITE(byte);
	HAL_SPI_TRANSFER_WAIT();
	return HAL_SPI_TRANSFER_READ();
}

void hal_init(void);
uint8_t hal_spi_dma_busy(void);
void hal_spi_dma_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, uint8_t length);

#endif
