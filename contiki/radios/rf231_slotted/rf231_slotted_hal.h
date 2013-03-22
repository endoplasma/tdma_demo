#ifndef RF231_SLOTTED_HAL_H
#define RF231_SLOTTED_HAL_H


/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include "lib/random.h"
#include "contiki-conf.h"
#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */

#define IRQ0PORT              GPIOA
#define IRQ0PIN               0 /* PA0/TIM5_CH1 -> "TRIG"  - required for input capture!!! */
#define RSTPORT               GPIOA
#define RSTPIN                1 /* PA1 -> "RST" */
#define IRQ1PORT              GPIOA
#define IRQ1PIN               3 /* PA3 -> "RT" */

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
#define SPI_DMA_IFCR_RX                 SPI_DMAx->LIFCR
#define SPI_DMA_IFCR_TX                 SPI_DMAx->LIFCR
#define SPI_DMA_IFCR_CTCIF_RX           DMA_LIFCR_CTCIF0
#define SPI_DMA_IFCR_CTCIF_TX           DMA_LIFCR_CTCIF3
#define SPI_DMAx_CHANNEL                (DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0)
#define SPI_DMAx_STREAM_RX_IRQn         DMA2_Stream0_IRQn
#define SPI_DMAx_STREAM_RX_IRQHandler   DMA2_Stream0_IRQHandler


/****************** TIMER DEFINITIONS *********************************/
/*
 *
 */
#define TIMx                  TIM2                          /** The timer */
#define TIMx_AF               1                             /** Alternate Function */
#define RCC_APB1ENR_TIMxEN    RCC_APB1ENR_TIM2EN            /** Enable Timer 2 */
#define TIMx_IRQn             TIM2_IRQn                     /** The timer */
#define TIMx_IRQHandler       TIM2_IRQHandler               /** Interrupt Handler */
#define TIM_GPIO_MASk         0xFFFFFFCC                    /** PORT 0 and 3 */
#define TIM_GPIO_MODE         0x00000022                    /** AF Mode for PORT 0 and 3 */
#define TIM_GPIO_AF           0x00001001                    /** The Alternate Function */
/**
 * Timer 2 is connected to the APB1 clock Source and runs with a
 * frequency of 84 MHz.  We want a clock cycle length of 250 ns. This
 * leads to a Prescaler Value of 84 MHz / 250 ns = 21.
 */
#define SYSTEM_CLOCK_FREQ     168000000	                    /** Clock Speed of the System */
#define TIM_CLOCK_FREQ        (SYSTEM_CLOCK_FREQ / 2)       /** Frequency of the timer clock source */
#define TIM_RESOLUTION_NS     250                           /** Resolution of the timer module. */
#if (TIM_CLOCK_FREQ * TIM_RESOLUTION_NS % 1000000000)
#error RF231 SLOTTED TDMA - desired TIM_RESOLUTION_NS is not possible
#endif
#define TIM_PSC               (TIM_CLOCK_FREQ / 1000 / 1000 * TIM_RESOLUTION_NS / 1000)   /** Timer Prescaler Value */
//#define TIM_TICKS_PER_SECOND                   /** Timer ticks per second */

/**
 * Input Capture Definitions
 */
#define CCR_IC                CCR1                          /** Input Register */
#define CCMR_IC               CCMR1                         /** Register with IC Channel */
#define CCMR_IC_MASK          (TIM_CCMR1_CC1S | \
                               TIM_CCMR1_IC1F | \
                               TIM_CCMR1_IC1PSC)            /** Mask for Input Capture */
#define CCMR_IC_CHAN          TIM_CCMR1_CC1S_0              /** IC1 Mapped on TI1 */
#define CCMR_IC_FILTER        0                             /** No Filtering */
#define CCMR_IC_PRESCALER     0                             /** No Prescaler */
#define CCER_IC_MASK          (TIM_CCER_CC1E | \
                               TIM_CCER_CC1P | \
                               TIM_CCER_CC1NP )             /** Input Capture Enable Mask */
#define CCER_IC_CCE           TIM_CCER_CC1E                 /** Capture Enabled */
#define CCER_IC_CCP           0                             /** Rising Edge */
#define TIM_IC_IRQ_FLAG       TIM_SR_CC1IF                   /** Input Capture Interupt */
#define TIM_IC_OC_FLAG        TIM_SR_CC1OF                  /** Input Capture Overcapture */
#define TIM_IC_IE             TIM_DIER_CC1IE                /** INput Capture Overcapture */

/**
 * Output Compare Definitions
 */
#define CCR_OC                CCR4                          /** Capture Register */
#define CCMR_OC               CCMR2                         /** Register with OC Channel */
#define CCMR_OC_MASK          (TIM_CCMR2_CC4S | \
                               TIM_CCMR2_OC4FE | \
                               TIM_CCMR2_OC4PE | \
                               TIM_CCMR2_OC4M | \
                               TIM_CCMR2_OC4CE )            /** Mask for Output Compare */
#define CCMR_OC_CHAN          0                             /** OC4 as Output */
#define CCMR_OC_FE            0                             /** fast mode disabled */
#define CCMR_OC_PE            0                             /** no preload register */
#define CCMR_OC_M             TIM_CCMR2_OC4M_0              /** active level on match */
#define CCMR_OC_CE            0                             /** clear disabled */
#define CCER_OC_MASK          (TIM_CCER_CC4E | \
                               TIM_CCER_CC4P | \
                               TIM_CCER_CC4NP )             /** Output Compare Enable Mask */
#define CCER_OC_CCE           TIM_CCER_CC4E                 /** Output Enabled */
#define CCER_OC_CCP           0                             /** Output High */
#define TIM_OC_IRQ_FLAG       TIM_SR_CC4IF                  /** Output Compare Interupt */
#define TIM_OC_OC_FLAG        TIM_SR_CC4OF                  /** Output Compare Overcapture */
#define TIM_OC_IE             TIM_DIER_CC4IE                /** Output Compare Overcapture */

/**
 * Timing definitions
 */
#define TDMA_PERIOD_NS        1000000                          /** the Period length in ns */
//#define TDMA_SLOTTIME_NS       800                           /** the slot position in ns */


#define TDMA_PERIOD_TICKS     (TDMA_PERIOD_NS / TIM_RESOLUTION_NS)
//#define PERIOD_TICKS          PERIOD_US * 1000 / TIM_RESOLUTION_NS  /** the Period length in actual clock ticks */
//#define SLOT_TICKS            SLOT_US * 1000 /  TIM_RESOLUTION_NS  /** the Period length in actual clock ticks */

#define FILTER_FACTOR         (1/2)                         /** alpha value for median calculation via IIF */

int hal_init(void);

#endif
