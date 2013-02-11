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
*			Contiki main file for STM32F4-Discovery
* \author
*			Robert Budde <robert.budde@tu-dortmund.de>
*/
/*---------------------------------------------------------------------------*/

#include "stm32f4xx.h"                  /* STM32F4xx Definitions              */
#include "contiki-conf.h"

#include <string.h>

#ifdef USE_USART3
#include "usart3.h"
#warning usart3 in use
#endif /* USE_USART3 */

#ifdef USE_UART4
#include "uart4.h"
#warning uart4 in use
#endif /* USE_UART4 */

#include "unique_id.h"

#include "dev/watchdog.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/battery-sensor.h"
#include "dev/serial-line.h"

#include "contiki-net.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",lladdr.u8[0], lladdr.u8[1], lladdr.u8[2], lladdr.u8[3],lladdr.u8[4], lladdr.u8[5], lladdr.u8[6], lladdr.u8[7])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif


#define __FI        1                   /* Font index 16x24                   */
#if (__FI == 1)                         /* Font index  6x8                    */                         
  #define __FONT_WIDTH  16
  #define __FONT_HEIGHT 24
#else                                   /* Font index 16x24                   */
  #define __FONT_WIDTH   6
  #define __FONT_HEIGHT  8
#endif

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

char  text[40];
rimeaddr_t addr;

/* here we declare the sensors that are on the board */
SENSORS(&button_sensor);
//SENSORS(&button_sensor,&adc_sensor,&battery_sensor);

// LED COUNTER
uint32_t count = 0;

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {                       /* Main Program                       */
//************************************************************************************************************
/* this is contiki-code */
  //watchdog_init();
  
  /* Initialize hardware. */
  clock_init();
  
#ifdef USE_USB
  /* Initialize serial communications */
  USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
#endif /* USE_USB */

#ifdef USE_USART3
	usart3_init(115200);
#if USART3_INPUT
	usart3_set_input(serial_line_input_byte);
	serial_line_init();
#endif /* USART3_INPUT */

#else /* USE_USART3 */
#ifdef USE_UART4
	uart4_init(115200);
#if USART4_INPUT
	uart4_set_input(serial_line_input_byte);
	serial_line_init();
#endif /* USART4_INPUT */

#else /* USE_UART4 */
#warning No serial line defined
#endif /* USE_UART4 */
#endif /* USE_USART3 */

  
  // Led initialization
  leds_init();

  PRINTF("\r\nStarting ");
  PRINTF(CONTIKI_VERSION_STRING);
  PRINTF(" on %s \r\n", PLATFORM_NAME);

  /*
   * Initialize Contiki and our processes.
   */

  /* rtimer and ctimer should be initialized before radio duty cycling layers*/
  rtimer_init();

  process_init();

  process_start(&sensors_process, NULL);

//	while (button_sensor.value(0) == 0) {
  for (count = 0; count < 20; count++) {
    leds_toggle(LEDS_BLUE);
    clock_wait(100);
  }
  leds_on(LEDS_BLUE);

  //leds_on(LEDS_RED);
  //leds_on(LEDS_YELLOW);
  //leds_on(LEDS_GREEN);

  /* etimers must be started before ctimer_init */
  process_start(&etimer_process, NULL);
  ctimer_init();

  /* Start radio and radio receive process */
  NETSTACK_RADIO.init(); 

  /* makes use of cpu-specific RNG peripheral - no seed needed */
  random_init(0);

  /* Set addresses BEFORE starting tcpip process */
  addr.u8[0] = 0x02;	// these are required!
  addr.u8[1] = 0x00; // *((uint8_t*)0x1FFF7A10+7)+*((uint8_t*)0x1FFF7A10+9)+*((uint8_t*)0x1FFF7A10+11);
  addr.u8[2] = 0x00; // *((uint8_t*)0x1FFF7A10+6)+*((uint8_t*)0x1FFF7A10+8)+*((uint8_t*)0x1FFF7A10+10);
  addr.u8[3] = 0xFF;	// these are required!
  addr.u8[4] = 0xFE;	// these are required!
  addr.u8[5] = *((uint8_t*)0x1FFF7A10+4);
  addr.u8[6] = *((uint8_t*)0x1FFF7A10+2);
  addr.u8[7] = *((uint8_t*)0x1FFF7A10+0);

  memcpy(&uip_lladdr.addr, &addr.u8, sizeof(rimeaddr_t));
  rimeaddr_set_node_addr(&addr); 

  rf230_set_pan_addr(0xabcd,0xbabe,(uint8_t *)&addr.u8);
  rf230_set_channel(CHANNEL_802_15_4);
  rf230_set_txpower(0); /* max */
  PRINTF("EUI-64 MAC: %x-%x-%x-%x-%x-%x-%x-%x\n",addr.u8[0],addr.u8[1],addr.u8[2],addr.u8[3],addr.u8[4],addr.u8[5],addr.u8[6],addr.u8[7]);

  /* Initialize stack protocols */
  queuebuf_init();

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

#define ANNOUNCE_BOOT 1
#if ANNOUNCE_BOOT
  PRINTF("%s %s, channel %u , check rate %u Hz tx power %u\n",NETSTACK_MAC.name, NETSTACK_RDC.name, rf230_get_channel(),
    CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:NETSTACK_RDC.channel_check_interval()),
    rf230_get_txpower());	  
#if UIP_CONF_IPV6_RPL
  PRINTF("RPL Enabled\n");
#endif
#if UIP_CONF_ROUTER
  PRINTF("Routing Enabled\n");
#endif
#endif /* ANNOUNCE_BOOT */

  process_start(&tcpip_process, NULL);

  /* Autostart other processes */
  autostart_start(autostart_processes);

#if ANNOUNCE_BOOT
  PRINTF("Online\n");
#endif /* ANNOUNCE_BOOT */

  energest_init();
//  ENERGEST_ON(ENERGEST_TYPE_CPU);
//  watchdog_start();
 
  while (1) {                           /* Loop forever                       */

    //watchdog_periodic();
    process_run();

  }
}
