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
*			LEDs specific implementation for STM32F4-Discovery
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include "contiki-conf.h"
#include "dev/leds.h"



/*---------------------------------------------------------------------------*/
void leds_arch_init(void) {   
  /* Enable clock for GPIOD                                  */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ;

  /* Configure LED (PD12-PD15) pins as push-pull outputs*/
	/*										 orange 		   		 greeen 			    		red 		      			blue */
  GPIOD->MODER  &= ~((3UL << 2 * 12) | (3UL << 2 * 13)  | (3UL << 2 * 14)  | (3UL << 2 * 15)); //set to 0
  GPIOD->MODER  |=   (1UL << 2 * 12) | (1UL << 2 * 13)  | (1UL << 2 * 14)  | (1UL << 2 * 15) ; //set to value
  GPIOD->OTYPER &= ~((1UL <<     12) | (1UL <<     13)  | (1UL <<     14)  | (1UL <<     15)); //set type: PP

}

//*---------------------------------------------------------------------------*/
unsigned char leds_arch_get(void) {
	unsigned char leds = 0x00;
	if (GPIOD->ODR & (1 << 13))
		leds |= LEDS_YELLOW; // eigentlich orange
	if (GPIOD->ODR & (1 << 12))
		leds |= LEDS_GREEN; 
	if (GPIOD->ODR & (1 << 14))
		leds |= LEDS_RED;
	if (GPIOD->ODR & (1 << 15))
		leds |= LEDS_BLUE;
	return leds;
}


//*---------------------------------------------------------------------------*/
void leds_arch_set(unsigned char leds) {

	if (leds & LEDS_GREEN)
		GPIOD->BSRRL |= (1 << 12);
	else
		GPIOD->BSRRH |= (1 << 12);

	if (leds & LEDS_YELLOW)
		GPIOD->BSRRL |= (1 << 13);
	else
		GPIOD->BSRRH |= (1 << 13);

	if (leds & LEDS_RED)
		GPIOD->BSRRL |= (1 << 14);
	else
		GPIOD->BSRRH |= (1 << 14);

	if (leds & LEDS_BLUE)
		GPIOD->BSRRL |= (1 << 15);
	else
		GPIOD->BSRRH |= (1 << 15);
}

