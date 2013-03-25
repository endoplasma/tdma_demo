#include "rf231_slotted_hal.h"

#ifdef SLOTTED_KOORDINATOR
#warning Koordinator mode
#endif

#ifdef JITTER_SIMULATION
#warning Simulation mode
#endif

extern void rf231_slotted_IC_irqh(uint32_t capture);
//extern void rf231_slotted_OC_irqh(void);

/**
 * Interupthandler Routine for TIMx
 */
void TIMx_IRQHandler(void)
{
  volatile uint32_t capture;
#ifdef JITTER_SIMULATION
  uint16_t jitter = 0;
#endif /* JITTER_SIMULATION */
  if (TIMx->SR & TIM_IC_IRQ_FLAG)	/* Input Capture Interrupt detected */
    {
      /* store the capture value, reset the irq flag, and call the irqh */
      capture = TIMx->CCR_IC;
      TIMx->SR &= ~TIM_IC_IRQ_FLAG;
      rf231_slotted_IC_irqh(capture);
    }
  if (TIMx->SR & TIM_OC_IRQ_FLAG)	/* Output Capture Interrupt detected */
    {
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
      //else {
    	//  TIMx->CCER |= (CCER_OC_CCE);
      //}
#endif /* JITTER_SIMULATION */
    }
  else if (TIMx->SR & TIM_SR_UIF)
    {
      TIMx->SR &= ~TIM_SR_UIF;
      // hal_time_isr_func();
    }
}

/**
 * set a new Output compare Value
 */
void hal_update_oc(uint32_t oc_value)
{
	TIMx->CCR_OC=oc_value;
}

/**
 * add a value to the current OC value
 */
void hal_update_oc_incr(uint32_t oc_value)
{
	TIMx->CCR_OC=TIMx->CCR_OC + oc_value;
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

  /****** Conifgure the Timer *************************************************/
  /* Enable the clocksource */
  RCC->APB1ENR |= RCC_APB1ENR_TIMxEN;
  /* Set Timer to disabled,upcounting,Edge-aligned,no prescaler */
  TIMx->CR1 = 0;
  TIMx->ARR = 0xFFFFFFFF;
#if (TIM_PSC != 21)
#error ghfdg
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

  TIMx->CCR_OC = TIMx->CNT + TDMA_PERIOD_TICKS;

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

  /**
   * Initialise Random Number Generator to simulate the effect of
   * jitter at Koordinator side
   */
#ifdef JITTER_SIMULATION  
#ifdef SLOTTED_KOORDINATOR
  /* initialise with seed 0 - stm32f4 generats random number via
     noise, we don need a seed */
  random_init(0); 
#endif /* SLOTTED_KOORDINATOR */
#endif /* JITTER_SIMULATION */

  /* Enable reset OC and IC Interrupt */
  TIMx->SR &= ~TIM_IC_IRQ_FLAG;
  TIMx->SR &= ~TIM_OC_IRQ_FLAG;
  TIMx->DIER |= TIM_OC_IE;
  TIMx->DIER |= TIM_IC_IE;
  
  IRQ_init_enable(TIMx_IRQn,1,0);

  /* TIM enable counter */
  TIMx->CR1 |= TIM_CR1_CEN;


  //  HAL_ENABLE_OVERFLOW_INTERRUPT();

  return 1;
}
