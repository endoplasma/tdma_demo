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
*			Real-timer header for STM32F407.
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#ifndef __RTIMER_ARCH_H__
#define __RTIMER_ARCH_H__

#include "contiki-conf.h"
#include "sys/clock.h"

/* http://courses.cs.tau.ac.il/embedded/projects/fall2009/contiki_lpc2148/ */
/* RTIMER_ARCH_SECOND ist maximal 65535 */
/* bei 168 MHz TIM1 Takt und Prescaler = 3360 ergeben sich 50000 Ticks/sec bzw. 20us/Tick */
/* alternativ 100us/Tick und 200us/Tick */

//#define RT_RESOLUTION RES_200US
#ifdef RT_CONF_RESOLUTION
#define RT_RESOLUTION RT_CONF_RESOLUTION
#else
#define RT_RESOLUTION RES_20US
#endif

#define RES_20US 0
#define RES_100US 1
#define RES_200US 2

#if RT_RESOLUTION == RES_20US
#define RT_PRESCALER 3360      // CK_CNT =  PCLK/3360 = 168 MHz/3360 = 50 kHz
#define RTIMER_ARCH_SECOND 50000   // One tick: 20 us.
#endif /* RT_RESOLUTION == RES_20US */

#if RT_RESOLUTION == RES_100US
#define RT_PRESCALER 16800      // CK_CNT =  PCLK/16800 = 168 MHz/16800 = 10 kHz
#define RTIMER_ARCH_SECOND 10000    // One tick: 100 us.
#endif /* RT_RESOLUTION == RES_100US */

#if RT_RESOLUTION == RES_200US
#define RT_PRESCALER 33600      // CK_CNT =  PCLK/33600 = 168 MHz/33600 = 5 kHz
#define RTIMER_ARCH_SECOND 5000    // One tick: 200 us.
#endif /* RT_RESOLUTION == RES_200US */

rtimer_clock_t rtimer_arch_now(void);
//#define rtimer_arch_now() clock_time()

// _swi(8) removed - 
//#WARNING
void rtimer_arch_disable_irq(void);
void rtimer_arch_enable_irq(void);

#endif /* __RTIMER_ARCH_H__ */
