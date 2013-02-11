#include "contiki.h"
#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(IOtest_process, "IOtest process");
AUTOSTART_PROCESSES(&IOtest_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(IOtest_process, ev, data)
{
  PROCESS_BEGIN();

  /*
   * Basic tests fpr printf. 
   * Include tests to write directly to stdout and stderr....
   */

  printf("Das ist ein test\n");
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
