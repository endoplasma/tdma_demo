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
#define TIMx                  TIM2                /** The timer */
#define RCC_APB1ENR_TIMxEN    RCC_APB1ENR_TIM2EN  /** Enable Timer 2 */
#define TIMx_IRQn             TIM2_IRQn           /** The timer */
#define TIMx_IRQHandler       TIM2_IRQHandler     /** Interrupt Handler */

#define CCMR_IC               CCMR1               /** Register with IC Channel */
#define CCMR_IC_CHAN         



#define CCMR_OC               CCMR2               /** Register with OC Channel */


int hal_init(void);
