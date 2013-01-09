/*
 * Copyright (c) 2012, TU Dortmund University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki OS
 *
 */
/*---------------------------------------------------------------------------*/
/**
* \file
*			ADC implementation for STM3240G-EVAL.
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#include "stm32f4xx.h"                  /* STM32F4xx Definitions              */
#include "adc-sensor.h"

#define ADC_VALUE_MAX      (0xFFF)

/*----------------------------------------------------------------------------
  Function that initializes ADC
 *----------------------------------------------------------------------------*/
static void init(void) {
  /* Setup and initialize ADC converter                                       */
  RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;   /* Enable ADC3 clock                  */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;  /* Enable GPIOF clock                 */
  GPIOF->MODER |= GPIO_MODER_MODER9;    /* PF9 is in Analog mode              */

  ADC3->SQR1   =   0;           
  ADC3->SQR2   =   0;           
  ADC3->SQR3   =  (7UL << 0);           /* SQ1 = channel 7                   */
  ADC3->SMPR1  =  (7UL << 6);           /* Channel 7 sample time is 480 cyc. */
  ADC3->SMPR2  =   0;                   /* Clear register                     */

	ADC3->CR2   |=  ADC_CR2_CONT;
	
  /* secret sauce? */
	ADC3->CR2   |=  (1UL << 3);           /* Initialize calibration registers   */
  while (ADC3->CR2 & (1UL << 3));       /* Wait for initialization to finish  */
  ADC3->CR2   |=  (1UL << 2);           /* Start calibration                  */
  while (ADC3->CR2 & (1UL << 2));       /* Wait for calibration to finish     */
}

static void activate(void) {
	/* ADC enable */
  ADC3->CR2 |= ADC_CR2_ADON;
  ADC3->CR2 |= ADC_CR2_SWSTART;         /* Start conversion                   */
}

static void deactivate(void) {
	/* ADC disable */
  ADC3->CR2 &= ~ADC_CR2_ADON;
}

static int active(void) {
  return (ADC3->CR2 & ADC_CR2_ADON) ? 1 : 0;
}

static int value(int type) {
  if (!active()) {
    return 0;
  }
	
	return (ADC3->DR & ADC_VALUE_MAX);
}

/*---------------------------------------------------------------------------*/
static int configure(int type, int value) {
  switch(type){
    case SENSORS_HW_INIT:
      init();
      return 1;
    case SENSORS_ACTIVE:
      if(value)        
        activate();
      else
        deactivate();
      return 1;
  }
       
  return 0;
}
/*---------------------------------------------------------------------------*/
static int status(int type) {
  switch(type) {
    case SENSORS_READY:
      return active();
  }
  
  return 0;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(adc_sensor, ADC_SENSOR,
	       value, configure, status);

