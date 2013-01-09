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
*			LEDs specific implementation for STM3240G-EVAL.
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include "contiki-conf.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
void leds_arch_init(void) {   
  /* Enable clock for GPIOC, GPIOG and GPIOI                                  */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOIEN;

  /* Configure LED (PG6, PG8, PI9, PC7) pins as push-pull outputs */
  GPIOG->MODER  &= ~((3UL << 2 * 6) | (3UL << 2 * 8));
  GPIOG->MODER  |=   (1UL << 2 * 6) | (1UL << 2 * 8);
  GPIOG->OTYPER &= ~((1UL <<     6) | (1UL <<     8));

  GPIOI->MODER  &= ~(3UL << 2 * 9);
  GPIOI->MODER  |=  (1UL << 2 * 9);
  GPIOI->OTYPER &= ~(1UL <<     9);

  GPIOC->MODER  &= ~(3UL << 2 * 7);
  GPIOC->MODER  |=  (1UL << 2 * 7);
  GPIOC->OTYPER &= ~(1UL <<     7);
}

//*---------------------------------------------------------------------------*/
unsigned char leds_arch_get(void) {
	unsigned char leds = 0x00;
	if (GPIOG->ODR & (1 << 6))
		leds |= LEDS_GREEN;
	if (GPIOG->ODR & (1 << 8))
		leds |= LEDS_YELLOW;
	if (GPIOI->ODR & (1 << 9))
		leds |= LEDS_RED;
	if (GPIOC->ODR & (1 << 7))
		leds |= LEDS_BLUE;
	return leds;
}


//*---------------------------------------------------------------------------*/
void leds_arch_set(unsigned char leds) {

	if (leds & LEDS_GREEN)
		GPIOG->BSRRL |= (1 << 6);
	else
		GPIOG->BSRRH |= (1 << 6);

	if (leds & LEDS_YELLOW)
		GPIOG->BSRRL |= (1 << 8);
	else
		GPIOG->BSRRH |= (1 << 8);

	if (leds & LEDS_RED)
		GPIOI->BSRRL |= (1 << 9);
	else
		GPIOI->BSRRH |= (1 << 9);

	if (leds & LEDS_BLUE)
		GPIOC->BSRRL |= (1 << 7);
	else
		GPIOC->BSRRH |= (1 << 7);
}

