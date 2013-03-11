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

/*---------------------------------------------------------------------------*/
PROCESS(rf231_slotted_process, "Slotted TDMA Driver");

uint8_t volatile rf231_pending;

/**
 * A struct to store information about the state and defines some basic global variables
 */

static uint32_t PeriodBuffer[NUM_PERIODS];      	/** Array to store the last measured Periods */
static uint8_t PeriodPos;					  		/** Position to store the next measurement */
static uint8_t PeriodCount;                         /** The number of stored Periods */
static uint32_t Period;                             /** The length of our send Period */


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
	PeriodPos = 0;
	PeriodCount = 0;
  hal_init();
  process_start(rf231_slotted_process, NULL);
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

void rf231_slotted_IC_irqh(uint32_t capture)
{
	/* write IC value into the IC buffer and generate a IC event to process the new value*/
#ifndef SLOTTED_KOORDINATOR
	static uint32_t last_capture = 0;
	/* set a new Output compare value */
	PeriodBuffer[PeriodPos] = capture - last_capture;
	last_capture = capture;
	process_post(&rf231_slotted_process, INPUT_CAPTURE_EVENT, NULL);
#endif /* SLOTTED_KOORDINATOR */

}

static void calculate_period()
{
	int i;
	uint32_t AvgPeriod;

	/* check if the last measured period was valid. If it was */
	/* if the number of stored Periods is smaller than the max number of periods
	 * only add it to the buffer.
	 */
	if (PeriodCount < NUM_PERIODS)
	{
		++PeriodCount;
		++PeriodPos;
	} else {
		AvgPeriod = 0;
		for (i=0; i < NUM_PERIODS; ++i)
		{
			AvgPeriod += PeriodBuffer[i];
		}
		AvgPeriod = AvgPeriod >> NUM_PERIODS_BASE; /** shift right to simulate division */
		Period = AvgPeriod;
		++PeriodPos;
	}
	if(PeriodPos >= NUM_PERIODS)
	{
		/* we reached the end of our buffer -> wrap around */
		PeriodPos = 0;
	}
}

/*---------------------------------------------------------------------------*/
/* Process to handle input packets
 * Receive interrupts cause this process to be polled
 * It calls the core MAC layer which calls rf231_read to get the packet
 * rf231processflag can be printed in the main idle loop for debugging
 */

PROCESS_THREAD(rf231_slotted_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
	  PROCESS_WAIT_EVENT();
	  if (ev == INPUT_CAPTURE_EVENT){
		  /* process the new IC Value that has been stored in the PeriodBuffer
		   * by the IRQ handler.
		   */
		  calculate_period();
	  }
  }

  PROCESS_END();
}
