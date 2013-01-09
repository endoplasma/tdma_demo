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
*			Battery voltage measurement implementation for STM3240G-EVAL.
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#include "stm32f4xx.h"                  /* STM32F4xx Definitions              */
#include "dev/battery-sensor.h"

#define ADC_VALUE_MAX      (0xFFF)

/*----------------------------------------------------------------------------
  Function that initializes ADC
 *----------------------------------------------------------------------------*/
static void init(void) {
  /* Setup and initialize ADC converter                                       */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   /* Enable ADC1 clock                  */

  ADC->CCR |= ADC_CCR_VBATE;

  ADC1->SQR1   =   0;           
  ADC1->SQR2   =   0;           
  ADC1->SQR3   =  (18UL << 0);          /* SQ1 = channel 18                   */
  ADC1->SMPR1  =  (7UL << (3*(18-10))); /* Channel 18 sample time is 480 cyc. */
  ADC1->SMPR2  =   0;                   /* Clear register                     */

	ADC1->CR2   |=  ADC_CR2_CONT;
	
  /* secret sauce? */
	ADC1->CR2   |=  (1UL << 3);           /* Initialize calibration registers   */
  while (ADC1->CR2 & (1UL << 3));       /* Wait for initialization to finish  */
  ADC1->CR2   |=  (1UL << 2);           /* Start calibration                  */
  while (ADC1->CR2 & (1UL << 2));       /* Wait for calibration to finish     */
}

static void activate(void) {
	/* ADC enable */
  ADC1->CR2 |= ADC_CR2_ADON;
  ADC1->CR2 |= ADC_CR2_SWSTART;         /* Start conversion                   */
}

static void deactivate(void) {
	/* ADC disable */
  ADC1->CR2 &= ~ADC_CR2_ADON;
}

static int active(void) {
  return (ADC1->CR2 & ADC_CR2_ADON) ? 1 : 0;
}

static int value(int type) {
  if (!active()) {
    return 0;
  }
	
	/* in mV ! */
	return (ADC1->DR & ADC_VALUE_MAX)*2*3300/0xFFF;
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
SENSORS_SENSOR(battery_sensor, BATTERY_SENSOR,
	       value, configure, status);

