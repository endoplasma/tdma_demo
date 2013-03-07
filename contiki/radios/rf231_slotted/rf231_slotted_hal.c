#include "rf231_slotted_hal.h"

/**
 * Interupthandler Routine for TIMx
 */
void TIMx_IRQHandler(void)
{
  volatile uint32_t capture;
  if (TIMx->SR & TIM_SR_CCxIF_IRQ0)	/* IRQ0 */
  {
    capture = TIMx->CCRx_IRQ0;

  }
  if (TIMx->SR & TIM_SR_CCxIF_IRQ1)	/* IRQ1 */
  {
    capture = TIMx->CCRx_IRQ1;
    // capture compare 4 verarbeiten
    TIM2->CCR4=TIM2->CCR4+PERIOD;
    // OC4 zurücksetzen
  }
  else if (TIMx->SR & TIM_SR_UIF)
  {
    TIMx->SR &= ~TIM_SR_UIF;
    hal_time_isr_func();
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
  unit16_t tmpccer = 0;         /** Capture/Compare Enabled Register Value */

  /****** Conifgure the Timer *************************************************/
  /* Enable the clocksource */
  RCC->APB1ENR |= RCC_APB1ENR_TIMxEN;
  /* Set Timer to disabled,upcounting,Edge-aligned,no prescaler */
  TIMx->CR1 = 0;
  /* Reset/Disable All Capture and Compare Units */
  TIMx->CCER = 0;

  /* Select Channel and set filter  */
  tmpccmrx = TIMx->CCMR_IC;
  tmpccmrx &= ((uint16_t)~TIM_CCMR1_CC1S) & ((uint16_t)~TIM_CCMR1_IC1F);
  tmpccmrx |= (uint16_t)(TIM_ICSEL | (uint16_t)(TIM_ICFilter << (uint16_t)4));


  /**
   * 1. set Input channel IC1
   */

  /* Write to TIMx CCMRx and CCER registers */
  TIMx->CCMRx_IRQ0 &= ~CCMRx_CCx_MASK_IRQ0; /* reset all bits regarding CCx */
  TIMx->CCMRx_IRQ0 |= TIM_CCMRx_IC_TI_MAP_IRQ0; /* map ICx to TIx, no filter, no prescaler */

  TIMx->CCER &= ~CCER_CCx_MASK_IRQ0; /* reset all bits regarding CCx */
  TIMx->CCER |= TIM_CCER_CCxE_IRQ0; /* rising & enable CCx */

  TIMx->CCER &= ~
capture compare 4 aktivieren


  IRQ_init_enable(TIMx_IRQn,1,0);

  /* TIM enable counter */
  TIMx->CR1 |= TIM_CR1_CEN;

  HAL_ENABLE_OVERFLOW_INTERRUPT();

  return 1;
}
