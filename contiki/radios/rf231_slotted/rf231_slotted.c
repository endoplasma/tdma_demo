#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include <stm32f4xx.h> /* STM32F4xx Definitions */
#include <clock.h>
#define delay_us( us )   ( clock_delay_usec( us ) )
#include "dev/leds.h"
#include "dev/spi.h"
#include "rf231_slotted.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "sys/timetable.h"
#include "lib/random.h"

/* RS232 delays will cause 6lowpan fragment overruns! Use DEBUGFLOW instead. */
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
//#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#define PRINTSHORT(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#else
#define PRINTF(...)
#define PRINTSHORT(...)
#endif
#if DEBUG>1
/* Output format is suitable for text2pcap to convert to wireshark pcap file.
 * Use $text2pcap -e 0x809a (these_outputs) capture.pcap
 * Since the hardware calculates and appends the two byte checksum to Tx packets,
 * we just add two zero bytes to the packet dump. Don't forget to enable wireshark
 * 802.15.4 dissection even when the checksum is wrong!
 */
#endif

uint8_t volatile rf231_pending;

/* RF231 hardware delay times, from datasheet */
typedef enum{
    TIME_TO_ENTER_P_ON               = 510, /**<  Transition time from VCC is applied to P_ON - most favorable case! */
    TIME_P_ON_TO_TRX_OFF             = 510, /**<  Transition time from P_ON to TRX_OFF. */
    TIME_SLEEP_TO_TRX_OFF            = 880, /**<  Transition time from SLEEP to TRX_OFF. */
    TIME_RESET                       = 6,   /**<  Time to hold the RST pin low during reset */
    TIME_ED_MEASUREMENT              = 140, /**<  Time it takes to do a ED measurement. */
    TIME_CCA                         = 140, /**<  Time it takes to do a CCA. */
    TIME_PLL_LOCK                    = 150, /**<  Maximum time it should take for the PLL to lock. */
    TIME_FTN_TUNING                  = 25,  /**<  Maximum time it should take to do the filter tuning. */
    TIME_NOCLK_TO_WAKE               = 6,   /**<  Transition time from *_NOCLK to being awake. */
    TIME_CMD_FORCE_TRX_OFF           = 1,   /**<  Time it takes to execute the FORCE_TRX_OFF command. */
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 180, /**<  Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON. */
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,   /**<  Transition time from PLL active state to another. */
}radio_trx_timing_t;

/*---------------------------------------------------------------------------*/
PROCESS(rf231_process, "RF231 driver");
/*---------------------------------------------------------------------------*/

int rf231_init(void);
int rf231_on(void);
int rf231_off(void);

static int rf231_read(void *buf, unsigned short bufsize);
static int rf231_prepare(const void *data, unsigned short len);
static int rf231_transmit(unsigned short len);
static int rf231_send(const void *data, unsigned short len);
static int rf231_receiving_packet(void);
static int rf231_pending_packet(void);
static int rf231_cca(void);

const struct radio_driver rf231_slotted_driver =
  {
    rf231_init,
    rf231_prepare,
    rf231_transmit,
    rf231_send,
    rf231_read,
    rf231_cca,
    rf231_receiving_packet,
    rf231_pending_packet,
    rf231_on,
    rf231_off
  };

/*---------------------------------------------------------------------------*/
int
rf231_init(void)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static int
rf231_prepare(const void *payload, unsigned short payload_len)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static int 
rf231_transmit(unsigned short payload_len)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static int
rf231_send(const void *payload, unsigned short payload_len)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static int
rf231_read(void *buf, unsigned short bufsize)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static int
rf231_cca(void)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
int
rf231_receiving_packet(void)
{
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
rf231_pending_packet(void)
{
  return 0;
}

/*---------------------------------------------------------------------------*/
int
rf231_off(void)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
int
rf231_on(void)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
/* Process to handle input packets
 * Receive interrupts cause this process to be polled
 * It calls the core MAC layer which calls rf231_read to get the packet
 * rf231processflag can be printed in the main idle loop for debugging
 */
#if 0
uint8_t rf231processflag;
#define RF231PROCESSFLAG(arg) rf231processflag=arg
#else
#define RF231PROCESSFLAG(arg)
#endif

PROCESS_THREAD(rf231_process, ev, data)
{
  int len;
  PROCESS_BEGIN();
  RF231PROCESSFLAG(99);

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
  }

  PROCESS_END();
}
