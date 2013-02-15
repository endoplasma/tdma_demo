#include "contiki.h"
#include <stdio.h>

#define delay_us( us )   ( clock_delay_usec( us ) )

/*---------------------------------------------------------------------------*/
PROCESS(IOtest_process, "IOtest process");
AUTOSTART_PROCESSES(&IOtest_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(IOtest_process, ev, data)
{
  PROCESS_BEGIN();

  /*
   * Basic tests for printf.
   * Include tests to write directly to stdout and stderr....
   */
  uint32_t i = 0;
  PROCESS_YIELD();
  // wait till all other processes are initilaised to avoid mutliple calls to printf
  //delay_us(5000000);

  printf("\n\r################################################################\n\r");
  printf("# Starting Newlib write Unit-Test:                             #\n\r");
  printf("################################################################\n\r");
  printf("\n\rThis is a Basic printf Test\n\r");
  printf("\n\rPrinting 20 times 64 chars to stdout: \n\r");
  while (i<20) {
	  printf("0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCD\r\n");
	  i++;
  }
  printf("################################################################\r\n");
  printf("\r\nPrinting 20 times 64 chars to stderr: \n\r");
  perror("\r\nThis is an Error!!!");
  printf("################################################################\r\n");
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
