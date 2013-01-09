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
*			USART3 implementation for STM32F407.
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include "contiki-conf.h"
#include <stdio.h>
#include <stdlib.h>

#include "usart3.h"
#include "nvic.h"
#include "sys/energest.h"
#include "dev/watchdog.h"
#include "lib/ringbuf.h"

static int (*usart3_input_handler)(unsigned char c);

static volatile uint8_t transmitting;

#ifdef USART3_CONF_TX_BUFSIZE
#define USART3_TX_BUFSIZE USART3_CONF_TX_BUFSIZE
#else /* USART3_CONF_TX_BUFSIZE */
#define USART3_TX_BUFSIZE 64
#endif /* USART3_CONF_TX_BUFSIZE */

static struct ringbuf txbuf;
static uint8_t txbuf_data[USART3_TX_BUFSIZE];

void usart3_set_input(int (*input)(unsigned char c)) {
  usart3_input_handler = input;
}

void usart3_writeb(unsigned char c) {
//  watchdog_periodic();
  /* Put the outgoing byte on the transmission buffer. If the buffer
     is full, we just keep on trying to put the byte into the buffer
     until it is possible to put it there. */
  while (ringbuf_put(&txbuf, c) == 0);

  /* If there is no transmission going, we need to start it by putting
     the first byte into the UART. */
  if (transmitting == 0) {
    transmitting = 1;
    USART3->DR = ringbuf_get(&txbuf);	/* put in UART */
    USART3->CR1 |= USART_CR1_TXEIE;		/* enable TX register empty int */
  }
}

/*---------------------------------------------------------------------------*/
#if ! WITH_UIP /* If WITH_UIP is defined, putchar() is defined by the SLIP driver */
#endif /* ! WITH_UIP */
/*---------------------------------------------------------------------------*/
/**
 * Initalize the RS232 port.
 *
 */
void usart3_init(unsigned long ubr)
{
  RCC->APB1ENR  |= RCC_APB1ENR_USART3EN;         /* Enable USART3 clock                */
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIODEN;         /* Enable GPIOC clock                 */

  GPIOD->MODER  &= 0xFFF0FFFF;
  GPIOD->MODER  |= 0x000A0000;
  GPIOD->AFR[1] |= 0x00000077;          /* PD8 USART3_Tx, PD9 USART3_Rx (AF7) */

  /* Configure USART3: 115200 baud @ 42MHz, 8 bits, 1 stop bit, no parity      */
  USART3->BRR = (22 << 4) | 12;
  USART3->CR3 = 0x0000;
  USART3->CR2 = 0x0000;
  USART3->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE;
 
  transmitting = 0;
  
  ringbuf_init(&txbuf, txbuf_data, sizeof(txbuf_data));  
    
	IRQ_init_enable(USART3_IRQn, 0, 0);
}

void USART3_IRQHandler(void)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  
	/* check if rx register not empty and int enabled */
  if ((USART3->SR & USART_SR_RXNE) && (USART3->CR1 & USART_CR1_RXNEIE)) {
		uint8_t c = USART3->DR;
		if (usart3_input_handler != NULL) {
			usart3_input_handler(c);
		}  
  }
	/* check if tx register empty and int enabled */
  if ((USART3->SR & USART_SR_TXE) && (USART3->CR1 & USART_CR1_TXEIE)) {
		if (ringbuf_elements(&txbuf) == 0) {
			transmitting = 0;
			USART3->CR1 &= ~USART_CR1_TXEIE;
		} else {
			USART3->DR = ringbuf_get(&txbuf);
		}
  }  
  
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
