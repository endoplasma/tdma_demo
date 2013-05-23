#ifndef RF231_SLOTTED_HAL_H
#define RF231_SLOTTED_HAL_H


/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include "lib/random.h"
#include "contiki-conf.h"
#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include "rf231_slotted.h"
#include "sys/process.h"
#include <nvic.h>                  /* STM32F4xx Definitions              */
#include <clock.h>
#include "rf231_slotted_registermap.h"
#include "rf231_slotted.h"



/******************************************************************************
 * AT86 RF231 PORT Definitions
 ******************************************************************************
 ****** IRQ, SLPTR and RST ***************************************************/
#define IRQPORT               GPIOA
#define IRQPIN                0 /* PA0/TIM2 CH1 - used in IC mode  */
#define RSTPORT               GPIOA
#define RSTPIN                1 /* PA1 */
#define SLPTRPORT             GPIOA
#define SLPTRPIN              3 /* PA3  TIM2 CH3 - used in OC mode */
/****** SPI Ports *************************************************************/
#define SPIPORT               GPIOA
#define SSPORT                SPIPORT
#define SSPIN                 4 /* PA4/SPI1 /SEL Slave select  */
#define SCKPORT               SPIPORT
#define SCKPIN                5 /* PA5/SPI1 CLKM */
#define MISOPORT              SPIPORT
#define MISOPIN               6 /* PA6/SPI1 MISO */
#define MOSIPORT              SPIPORT
#define MOSIPIN               7 /* PA7/SPI1 MOSI */

#define RCC_AHB1ENR_GPIOxEN   RCC_AHB1ENR_GPIOAEN

#if RF231_HAS_PA
#define PAENPORT              GPIOC
#define PAENPIN               4
#define HGMPORT               GPIOC
#define HGMPIN                5
#define RCC_AHB1ENR_PAGPIOxEN RCC_AHB1ENR_GPIOCEN
#endif /* RF231_HAS_PA */

/***** SPI Controller *********************************************************/
#define SPIx                  SPI1
#define SPI_APBxENR           APB2ENR
#define RCC_APBxENR_SPIxEN    RCC_APB2ENR_SPI1EN
#define SPI_APBxRSTR          APB2RSTR
#define RCC_APBxRSTR_SPIxRST  RCC_APB2RSTR_SPI1RST
#define SPIx_AF               5
#define SPIx_BASE             SPI1_BASE

/****** SPI DMA Controller ****************************************************/
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

/*******************************************************************************
 ******** TIMER DEFINITIONS ****************************************************
 *******************************************************************************/
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


/******************************************************************************
 * TDMA Timing definitions
 ******************************************************************************/
#define PHY_TIME_PER_BYTE_NS                 (32000)
#define PHY_SYNCH_HEADER_NS                  (192000)

#define TDMA_SLOTTIME_NS                     (714000)                                  /** the slot position in ns */
#define TDMA_BEACON_FRAME_NS                 (PHY_SYNCH_HEADER_NS + BEACON_LENGTH * PHY_TIME_PER_BYTE_NS)
#define TDMA_RESPONSE_FRAME_NS               (PHY_SYNCH_HEADER_NS + RESPONSE_LENGTH * PHY_TIME_PER_BYTE_NS)

#define CLIENT_PROCESSING_TIME_NS            (105000)      /**< the processing time needed by the client */
#define KOORD_PROCESSING_TIME_NS             (105000)      /**< the processing time needed by the client */

#define TDMA_GUARD_TIME_NS                   (10000)

#define HARDWARE_DELAY_NS                    (16000)

#define TDMA_PERIOD_NS        (TDMA_BEACON_FRAME_NS + CLIENT_PROCESSING_TIME_NS + (MAX_CLIENTS * (TDMA_RESPONSE_FRAME_NS + TDMA_GUARD_TIME_NS)) + KOORD_PROCESSING_TIME_NS)
#define TDMA_PERIOD_US        (TDMA_PERIOD_NS / 100);
#define TDMA_PERIOD_TICKS     (TDMA_PERIOD_NS / TIM_RESOLUTION_NS)    /** The Period in timer ticks */
#define CLIENT_PROCESSING_TIME_TICKS         (CLIENT_PROCESSING_TIME_NS / TIM_RESOLUTION_NS)  /**< the processing time in ticks */
#define KOORD_PROCESSING_TIME_TICKS          (CLIENT_PROCESSING_TIME_NS / TIM_RESOLUTION_NS)  /**< the processing time in ticks */
#define TDMA_SLOT_TICKS       (TDMA_SLOTTIME_NS / TIM_RESOLUTION_NS)  /** The Slot Time in timer ticks */
#define TDMA_BEACON_TICKS     (TDMA_BEACON_FRAME_NS / TIM_RESOLUTION_NS)
#define HARDWARE_DELAY_TICKS                (HARDWARE_DELAY_NS / TIM_RESOLUTION_NS)

#define FILTER_FACTOR                        (1/2)                         /** alpha value for median calculation via IIF */

/******************************************************************************
 * Define the Timer Channels for Interrupt and Event generation
 ******************************************************************************
 * Input Capture Channel for Frame Timing                                     */
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
#define TIM_IC_IRQ_FLAG       TIM_SR_CC1IF                  /** Input Capture Interupt */
#define TIM_IC_OC_FLAG        TIM_SR_CC1OF                  /** Input Capture Overcapture */
#define TIM_IC_IE             TIM_DIER_CC1IE                /** INput Capture Overcapture */

/******************************************************************************
 * Output Compare Definitions - Send Timing
 ******************************************************************************/
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
#define TIM_OC_OC_FLAG        TIM_SR_CC4OF                  /** Output Compare Overcapture Flag */
#define TIM_OC_IE             TIM_DIER_CC4IE                /** Output Compare Interrupt Enabled */

/******************************************************************************
 * Definitions for the TX_MODE timer
 ******************************************************************************/
#define CCR_TX_MODE           CCR2                          /** Capture Register */
#define CCMR_TX_MODE          CCMR1                         /** Register with OC Channel */
#define CCMR_TX_MODE_MASK     (TIM_CCMR1_CC2S | \
                               TIM_CCMR1_OC2FE | \
                               TIM_CCMR1_OC2PE | \
                               TIM_CCMR1_OC2M | \
                               TIM_CCMR1_OC2CE )            /** Mask for Output Compare */

#define CCMR_TX_MODE_CHAN          0                             /** OC4 as Output */
#define CCMR_TX_MODE_FE            0                             /** fast mode disabled */
#define CCMR_TX_MODE_PE            0                             /** no preload register */
#define CCMR_TX_MODE_M             0                             /** freeze the output */
#define CCMR_TX_MODE_CE            0                             /** clear disabled */
#define CCER_TX_MODE_MASK          (TIM_CCER_CC2E | \
                                    TIM_CCER_CC2P | \
                                    TIM_CCER_CC2NP )             /** Output Compare Enable Mask */
#define CCER_TX_MODE_CCE           TIM_CCER_CC2E                 /** Output Enabled */
#define CCER_TX_MODE_CCP           0                             /** Output High */
#define TIM_TX_MODE_IRQ_FLAG       TIM_SR_CC2IF                  /** Output Compare Interupt */
#define TIM_TX_MODE_OC_FLAG        TIM_SR_CC2OF                  /** Output Compare Overcapture */
#define TIM_TX_MODE_IE             TIM_DIER_CC2IE                /** Output Compare Interrupt Enabled*/

/******************************************************************************
 * Definitions for the Beacon_missed timer
 ******************************************************************************/
#define CCR_BEACON_MISSED           CCR3                          /** Capture Register */
#define CCMR_BEACON_MISSED          CCMR2                         /** Register with OC Channel */
#define CCMR_BEACON_MISSED_MASK     (TIM_CCMR2_CC3S | \
                                     TIM_CCMR2_OC3FE | \
                                     TIM_CCMR2_OC3PE | \
                                     TIM_CCMR2_OC3M | \
                                     TIM_CCMR2_OC3CE )            /** Mask for Output Compare */

#define CCMR_BEACON_MISSED_CHAN          0                             /** OC4 as Output */
#define CCMR_BEACON_MISSED_FE            0                             /** fast mode disabled */
#define CCMR_BEACON_MISSED_PE            0                             /** no preload register */
#define CCMR_BEACON_MISSED_M             TIM_CCMR2_OC3M_0              /** active level on match */
#define CCMR_BEACON_MISSED_CE            0                             /** clear disabled */
#define CCER_BEACON_MISSED_MASK          (TIM_CCER_CC3E | \
                                          TIM_CCER_CC3P | \
                                          TIM_CCER_CC3NP )             /** Output Compare Enable Mask */
#define CCER_BEACON_MISSED_CCE           TIM_CCER_CC3E                 /** Output Enabled */
#define CCER_BEACON_MISSED_CCP           0                             /** Output High */
#define TIM_BEACON_MISSED_IRQ_FLAG       TIM_SR_CC3IF                  /** Output Compare Interupt */
#define TIM_BEACON_MISSED_OC_FLAG        TIM_SR_CC3OF                  /** Output Compare Overcapture */
#define TIM_BEACON_MISSED_IE             TIM_DIER_CC3IE                /** Output Compare Interrupt Enabled*/


/****************************************************************************
 * Makro definitions
 *****************************************************************************/
/** This macro pulls the SS pin high. */
#define HAL_SS_HIGH( )   ( SSPORT->BSRRL = ((uint32_t)1 << (uint32_t)SSPIN) ) 
/** This macro pulls the SS pin low. */
#define HAL_SS_LOW( )    ( SSPORT->BSRRH = ((uint32_t)1 << (uint32_t)SSPIN) ) 
/** wait for some time and do nothing */
#define delay_us( us )   ( clock_delay_usec( us ) )

#define hal_set_slptr_high( ) ( SLPTRPORT->BSRRL = ((uint32_t)1 << (uint32_t)SLPTRPIN) ) /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low( )  ( SLPTRPORT->BSRRH = ((uint32_t)1 << (uint32_t)SLPTRPIN) ) /**< This macro pulls the SLP_TR pin low. */
#define hal_get_slptr( )      !!( SLPTRPORT->IDR & ((uint32_t)1 << (uint32_t)SLPTRPIN) ) /**< Read current state of the SLP_TR pin (High/Low). */

#define hal_set_rst_high( )   ( RSTPORT->BSRRL = ((uint32_t)1 << (uint32_t)RSTPIN) ) /**< This macro pulls the RST pin high. */
#define hal_set_rst_low( )    ( RSTPORT->BSRRH = ((uint32_t)1 << (uint32_t)RSTPIN) ) /**< This macro pulls the RST pin low. */
#define hal_get_rst( )        !!( RSTPORT->IDR & ((uint32_t)1 << (uint32_t)RSTPIN) ) /**< Read current state of the RST pin (High/Low). */

/****************************************************************************
 * Masks
 *****************************************************************************/
#define HAL_BAT_LOW_MASK       ( 0x80 ) /**< Mask for the BAT_LOW interrupt. */
#define HAL_TRX_UR_MASK        ( 0x40 ) /**< Mask for the TRX_UR interrupt. */
#define HAL_TRX_END_MASK       ( 0x08 ) /**< Mask for the TRX_END interrupt. */
#define HAL_RX_START_MASK      ( 0x04 ) /**< Mask for the RX_START interrupt. */
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) /**< Mask for the PLL_UNLOCK interrupt. */
#define HAL_PLL_LOCK_MASK      ( 0x01 ) /**< Mask for the PLL_LOCK interrupt. */
#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */


/****************************************************************************
 * Typedefs
 *****************************************************************************/
/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
} hal_rx_frame_t;

/* RX_START event handler callback type. Is called with timestamp in
 * IEEE 802.15.4 symbols and frame length. See
 * hal_set_rx_start_event_handler(). */
typedef void (*hal_rx_start_isr_event_handler_t)(uint32_t const isr_timestamp, uint8_t const frame_length);

/* RRX_END event handler callback type. Is called with timestamp in
 * IEEE 802.15.4 symbols and frame length. See
 * hal_set_trx_end_event_handler(). */
typedef void (*hal_trx_end_isr_event_handler_t)(uint32_t const isr_timestamp);

typedef void (*rx_callback_t) (uint16_t data);



/*============================ PROTOTYPES ====================================*/
int hal_init( void );
uint8_t hal_register_read( uint8_t address );
void hal_register_write( uint8_t address, uint8_t value );
uint8_t hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position );
void hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position,
                            uint8_t value );

void hal_reset_flags( void );
uint8_t hal_get_bat_low_flag( void );
void hal_clear_bat_low_flag( void );

hal_trx_end_isr_event_handler_t hal_get_trx_end_event_handler( void );
void hal_set_trx_end_event_handler( hal_trx_end_isr_event_handler_t trx_end_callback_handle );
void hal_clear_trx_end_event_handler( void );

hal_rx_start_isr_event_handler_t hal_get_rx_start_event_handler( void );
void hal_set_rx_start_event_handler( hal_rx_start_isr_event_handler_t rx_start_callback_handle );
void hal_clear_rx_start_event_handler( void );

uint8_t hal_get_pll_lock_flag( void );
void hal_clear_pll_lock_flag( void );

void hal_frame_read(hal_rx_frame_t *rx_frame);
void hal_frame_write( uint8_t *write_buffer, uint8_t length );
//void hal_sram_read( uint8_t address, uint8_t length, uint8_t *data );
//void hal_sram_write( uint8_t address, uint8_t length, uint8_t *data );



#endif /* RF231_SLOTTED_HAL_H */
