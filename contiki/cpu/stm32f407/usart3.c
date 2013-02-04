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
#define USART3_TX_BUFSIZE 128
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

void usart3_writebuff(unsigned char* buf, int len) {

  while (len) {
    /* check the empty items in the ringbuff */
    while (ringbuf_put(&txbuf, *buf) == 0);
    buf++;
    len--;
#ifndef USART3_TX_USE_DMA1_CH4 
    if (transmitting == 0) {
      transmitting = 1;
      USART3->DR = ringbuf_get(&txbuf);	/* put in UART */
      USART3->CR1 |= USART_CR1_TXEIE;		/* enable TX register empty int */
    }
#else
    if ((DMA1_Stream3->CR & DMA_SxCR_EN ) != DMA_SxCR_EN ) {
      /* set the start address of your data */      
      DMA1_Stream3->M0AR = (uint32_t) txbuf->data + txbuf->get_ptr;

      /* set the length of your data */ 
      DMA1_Stream3->NDTR = (txbuf->put_ptr - txbuf->get_ptr) & txbuf->mask;
      if (DMA1_Stream3->NDTR > (txbuf->mask + 1 - txbuf->get_ptr)) {
	DMA1_Stream3->NDTR = txbuf->mask + 1 - txbuf->get_ptr;
      }

      /* Enable transfer by setting EN bit */
      DMA1_Stream3->CR |= DMA_SxCR_EN;
    }
#endif

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

#ifdef USART3_TX_USE_DMA1_CH4
#warning DMA1 stream3 channel4 is used by USART3 for transmission!

  /*********************************************************************************
   * DMA Configuration
   *********************************************************************************/
  RCC->AHB1ENR |= RCC_AHB1_SPI_DMAx; /* enable DMA clock */

  /* Get the configuration register from DMA1_Stream3 */
  tmpreg = DMA1_Stream3->CR;
  /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
  tmpreg &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
			 DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | \
			 DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | \
			 DMA_SxCR_DIR));
  /*********************************************************************************
   * Configure SPI_DMAx_STREAM_TX:
   * Set CHSEL bits to SPI_DMAx_CHANNEL
   * Set DIR bits to 01 (Mem->Periph)
   * Set PINC bit to 0 (disable)
   * Set MINC bit to 1 (enable)
   * Set PSIZE bits to 00 (byte)
   * Set MSIZE bits to 00 (byte)
   * Set CIRC bit to 0 (disable)
   * Set PL bits to 00 (low)
   * Set MBURST bits to 00 (single)
   * Set PBURST bits to 00 (single) 
   *********************************************************************************/
  tmpreg |= DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0 | DMA_SxCR_MINC;
  /* Write to DMA controller 1 Strem 3 CR register */
  DMA1_Stream3->CR = tmpreg;
  /* Clear DMDIS and FTH bits in the FCR register */
  DMA1_Stream3->FCR &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  /* set the address of uart3 tx register into PAR */
  DMA1_Stream3->PAR = USART3_BASE + 0x04;

  /* Enable UART DMA mode */
  USART3->CR3 &= DMAT;

  IRQ_init_enable(DMA1_Stream3_IRQn,0,0);
#endif
 
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


 void DMA1_Stream3_IRQHandler(void) {
   /* Clear Transfer Complete Interrupt Flag */
   DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
   
   txbuf->get_ptr = (DMA1_Stream3->M0AR - txbuf->data) & txbuf->mask;
   if (((r->put_ptr - r->get_ptr) & r->mask ) > 0 ) {
     /* set the start address of your data */      
     DMA1_Stream3->M0AR = (uint32_t) txbuf->data + txbuf->get_ptr;

     /* set the length of your data */ 
     DMA1_Stream3->NDTR = (txbuf->put_ptr - txbuf->get_ptr) & txbuf->mask;
     if (DMA1_Stream3->NDTR > (txbuf->mask + 1 - txbuf->get_ptr)) {
       DMA1_Stream3->NDTR = txbuf->mask + 1 - txbuf->get_ptr;
     }

     /* Enable transfer by setting EN bit */
     DMA1_Stream3->CR |= DMA_SxCR_EN;
   }

 }
