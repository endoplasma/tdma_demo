#include "contiki.h"
#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include <stdio.h> 
 
//-----------------------------------------------------------------   
PROCESS(blink_process, "blink example");
AUTOSTART_PROCESSES(&blink_process);
//-----------------------------------------------------------------
PROCESS_THREAD(blink_process, ev, data)
{   
  PROCESS_EXITHANDLER(goto exit);
  PROCESS_BEGIN();
 
  /* Initialize stuff here. */ 
 
	printf("++++++++++++++++++++++++++++++\r\n");
	printf("+  LESSON 1, FIRST EXERCISE  +\r\n");
	printf("++++++++++++++++++++++++++++++\r\n");
	printf("+ Blink app w/ button sensor +\r\n");
	printf("++++++++++++++++++++++++++++++\r\n\r\n");
 
	SENSORS_ACTIVATE(button_sensor);
	leds_on(LEDS_ALL);
	printf("+     All leds are on     +\r\n\r\n");   
	printf("Press the user button to begin\r\n\r\n");
 
  while(1) {
		/* Do the rest of the stuff here. */
	 
		static uint8_t push = 0;	// Keeps the number of times the user pushes the button sensor
	 
		PROCESS_WAIT_EVENT_UNTIL((ev==sensors_event) && (data == &button_sensor));
		
		if (push % 2 == 0) { 
			leds_toggle(LEDS_ALL);
			printf("[%d] TURNING OFF ALL LEDS ... [DONE]\r\n", push);
			push++;
		} else {
			leds_toggle(LEDS_ALL);
			printf("[%d] TURNING ON ALL LEDS ...   [DONE]\r\n", push);
			push++;
		}
		if (push == 255)
			push = 0; // Prevents overflowing		 
  }
 
	exit:
		leds_off(LEDS_ALL);
		PROCESS_END();
}
