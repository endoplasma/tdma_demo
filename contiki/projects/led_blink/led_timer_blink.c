#include "contiki.h"
#include "contiki-conf.h"
#include "dev/leds.h"
#include "adc-sensor.h"
#include "dev/button-sensor.h"
#include "dev/battery-sensor.h"
#include <stdio.h> /* For printf() */
 
//-----------------------------------------------------------------   
PROCESS(blink_timer_process, "blink with timer example");
AUTOSTART_PROCESSES(&blink_timer_process);
//-----------------------------------------------------------------
 
 
PROCESS_THREAD(blink_timer_process, ev, data)
{   
  PROCESS_EXITHANDLER(goto exit);
  PROCESS_BEGIN();
 
  /* Initialize stuff here. */ 
 
	printf("\r\n++++++++++++++++++++++++++++++\r\n");
	printf("+    LESSON 1, EXERCISE 2    +\r\n");
	printf("++++++++++++++++++++++++++++++\r\n");
	printf("+     Blink app w/ timer     +\r\n");
	printf("++++++++++++++++++++++++++++++\r\n\r\n");	
	
	SENSORS_ACTIVATE(button_sensor);
	SENSORS_ACTIVATE(battery_sensor);
	SENSORS_ACTIVATE(adc_sensor);
	leds_off(LEDS_ALL);
	printf("+       All leds are off     +\r\n\r\n");   
	printf("Press the user button to start\r\n\r\n");
 
    while(1) {
	/* Do the rest of the stuff here. */
 
	static uint32_t ticks = 0;
	static uint32_t seconds = 3;
	static struct etimer et; // Define the timer
 
	PROCESS_WAIT_EVENT();  // Waiting for a event, don't care which
 
	if(ev == sensors_event) {  // If the event it's provoked by the user button, then...
           if(data == &button_sensor) {		
		etimer_set(&et, CLOCK_SECOND*seconds);  // Set the timer
		printf("+       Timer started        +\r\n");
           }
        }
 
	if(etimer_expired(&et)) {  // If the event it's provoked by the timer expiration, then...
		leds_toggle(LEDS_BLUE);
		
		
		
		if (ticks % 2 == 0) {
			printf("+ LED BLUE .............. [ON] %d / %dmV\r\n", adc_sensor.value(0),battery_sensor.value(0));
                }
		else { 
			printf("+ LED BLUE ............. [OFF] %d / %dmV\r\n", adc_sensor.value(0),battery_sensor.value(0));
                }
		etimer_reset(&et);
		ticks++;
                }	
	}		
	exit:
		leds_off(LEDS_ALL);
		PROCESS_END();
}
