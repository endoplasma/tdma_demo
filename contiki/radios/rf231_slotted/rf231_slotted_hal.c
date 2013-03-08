#include "rf231_slotted_hal.h"

/**
 * Interupthandler Routine for TIMx
 */
void TIMx_IRQHandler(void)
{
  volatile uint32_t capture;
  if (TIMx->SR & TIM_IC_IRQ_FLAG)	/* Input Capture Interrupt detected */
  {
    capture = TIMx->CCR_IC;
    TIMx->SR &= ~TIM_IC_IRQ_FLAG;

  }
  if (TIMx->SR & TIM_OC_IRQ_FLAG)	/* Output Capture Interrupt detected */
  {

    TIMx->CCR_OC=TIMx->CCR_OC + PERIOD;
    TIMx->SR &= ~TIM_OC_IRQ_FLAG;

    TIMx->CCMR_OC &= ~(TIM_CCMR2_OC4M);
    TIMx->CCMR_OC |= (TIM_CCMR2_OC4M_2);

    TIMx->CCMR_OC &= ~(TIM_CCMR2_OC4M);
    TIMx->CCMR_OC |= (TIM_CCMR2_OC4M_0);
    
    /* Polarity to faling */
    /* OC flag löschen
       Flanke umdrehen
       etrf manuell auslösen
       Flanke drehen
       OC löschen */

  }
  else if (TIMx->SR & TIM_SR_UIF)
  {
    TIMx->SR &= ~TIM_SR_UIF;
    // hal_time_isr_func();
  }
}

/*--------------------------------------------------------------------------*/
/**
 * Intialise the hardware
 * 
 */
int
hal_init(void) 
{
  uint16_t tmpccmrx = 0;        /** Capture/Compare Mode Register Value */
  uint16_t tmpccer = 0;         /** Capture/Compare Enabled Register Value */

  /****** Conifgure the Timer *************************************************/
  /* Enable the clocksource */
  RCC->APB1ENR |= RCC_APB1ENR_TIMxEN;
  /* Set Timer to disabled,upcounting,Edge-aligned,no prescaler */
  TIMx->CR1 = 0;
  TIMx->ARR = 0xFFFFFFFF;
  TIMx->PSC = 84-1;
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

  TIMx->CCR_OC = PERIOD;

  /**
   * Set special function for Input and Output
   * 
   */
  //RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;         /* Enable GPIOC clock                 */
//  GPIOA->MODER  &= ~TIM_GPIO_MASk;
//  GPIOA->MODER  |= TIM_GPIO_MODE;
//  GPIOA->AFR[1] |= TIM_GPIO_AF;          /* PD8 USART3_Tx, PD9 USART3_Rx (AF7) */

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

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

  /* Enable OC Interrupt */
  TIMx->DIER |= TIM_OC_IE;

  IRQ_init_enable(TIMx_IRQn,1,0);

  /* TIM enable counter */
  TIMx->CR1 |= TIM_CR1_CEN;


  //  HAL_ENABLE_OVERFLOW_INTERRUPT();

  return 1;
}
