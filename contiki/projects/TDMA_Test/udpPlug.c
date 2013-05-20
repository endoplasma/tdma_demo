#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>
#include <stdio.h>
#include <dev/watchdog.h>
#include "dev/button-sensor.h"
#include "dev/leds.h"


#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"
#include "net/rpl/rpl.h"

#include "udpPlug.h"

PROCESS(tdma_demo_process, "TDMA Demo process");
AUTOSTART_PROCESSES(&tdma_demo_process);

PROCESS_THREAD(tdma_demo_process, ev, data)
{
	PROCESS_BEGIN();
  
	PRINTF("TDMA Demo started\r\n");
	SENSORS_ACTIVATE(button_sensor);
	
	while(1) {
		PROCESS_YIELD();
				
	}

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
