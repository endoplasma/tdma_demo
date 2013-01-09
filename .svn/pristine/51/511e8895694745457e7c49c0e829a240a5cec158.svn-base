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
*			UART4 implementation for STM32F407.
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include "contiki-conf.h"
#include <stdio.h>
#include <stdlib.h>

#include "uart4.h"
#include "nvic.h"
#include "sys/energest.h"
#include "dev/watchdog.h"
#include "lib/ringbuf.h"

static int (*uart4_input_handler)(unsigned char c);

static volatile uint8_t transmitting;

#ifdef UART4_CONF_TX_BUFSIZE
#define UART4_TX_BUFSIZE UART4_CONF_TX_BUFSIZE
#else /* UART4_CONF_TX_BUFSIZE */
#define UART4_TX_BUFSIZE 64
#endif /* UART4_CONF_TX_BUFSIZE */

static struct ringbuf txbuf;
static uint8_t txbuf_data[UART4_TX_BUFSIZE];

void uart4_set_input(int (*input)(unsigned char c)) {
  uart4_input_handler = input;
}

void uart4_writeb(unsigned char c) {
//  watchdog_periodic();
  /* Put the outgoing byte on the transmission buffer. If the buffer
     is full, we just keep on trying to put the byte into the buffer
     until it is possible to put it there. */
  while (ringbuf_put(&txbuf, c) == 0);

  /* If there is no transmission going, we need to start it by putting
     the first byte into the UART. */
  if (transmitting == 0) {
    transmitting = 1;
    UART4->DR = ringbuf_get(&txbuf);	/* put in UART */
    UART4->CR1 |= USART_CR1_TXEIE;		/* enable TX register empty int */
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
void uart4_init(unsigned long ubr)
{
  RCC->APB1ENR  |= RCC_APB1ENR_UART4EN;         /* Enable UART4 clock                */
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;         /* Enable GPIOC clock                 */

  GPIOC->MODER  &= 0xFF0FFFFF;
  GPIOC->MODER  |= 0x00A00000;
  GPIOC->AFR[1] |= 0x00008800;          /* PC10 UART4_Tx, PC11 UART4_Rx (AF8) */

  /* Configure UART4: 115200 baud @ 42MHz, 8 bits, 1 stop bit, no parity      */
  UART4->BRR = (22 << 4) | 12;
  UART4->CR3 = 0x0000;
  UART4->CR2 = 0x0000;
  UART4->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE;
 
  transmitting = 0;
  
  ringbuf_init(&txbuf, txbuf_data, sizeof(txbuf_data));  
    
	IRQ_init_enable(UART4_IRQn, 0, 0);
}

void UART4_IRQHandler(void)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  
	/* check if rx register not empty and int enabled */
  if ((UART4->SR & USART_SR_RXNE) && (UART4->CR1 & USART_CR1_RXNEIE)) {
		uint8_t c = UART4->DR;
		if (uart4_input_handler != NULL) {
			uart4_input_handler(c);
		}  
  }
	/* check if tx register empty and int enabled */
  if ((UART4->SR & USART_SR_TXE) && (UART4->CR1 & USART_CR1_TXEIE)) {
		if (ringbuf_elements(&txbuf) == 0) {
			transmitting = 0;
			UART4->CR1 &= ~USART_CR1_TXEIE;
		} else {
			UART4->DR = ringbuf_get(&txbuf);
		}
  }  
  
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
