#include <stdio.h>
#include <string.h>

#include "contiki.h"

#include <stm32f4xx.h> /* STM32F4xx Definitions */
#include <clock.h>

#define delay_us( us )   ( clock_delay_usec( us ) )

#include "dev/leds.h"
#include "dev/spi.h"
#include "uwb.h"

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

uint8_t ram_tx_buffer[1024];
uint8_t ram_rx_buffer[1024];

/*---------------------------------------------------------------------------*/
PROCESS(uwb_process, "UWB driver");
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static int init(void);
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
send(const void *payload, unsigned short payload_len)
{
  prepare(payload, payload_len);
  return transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
read(void *buf, unsigned short buf_len)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  return 0;
}

const struct radio_driver uwb_driver =
  {
    init,
    prepare,
    transmit,
    send,
    read,
    channel_clear,
    receiving_packet,
    pending_packet,
    on,
    off,
  };

int run_tests(void)
{
  uint8_t i,j;

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  PRINTF("uwb: SPI pattern test ...\r\n");
	i=0;
	ram_tx_buffer[i++] = 0x00;	/* skipped */
	ram_tx_buffer[i++] = 0xFF;
	ram_tx_buffer[i++] = 0xCC;
	ram_tx_buffer[i++] = 0x33;
	ram_tx_buffer[i++] = 0xAA;
	ram_tx_buffer[i++] = 0x55;
	ram_tx_buffer[i++] = 0x0F;
	ram_tx_buffer[i++] = 0xF0;
	ram_tx_buffer[i++] = 0x00;	/* skipped */
	HAL_SPI_TRANSFER_OPEN();
	HAL_SPI_TRANSFER(CMD_SPI_LOOPBACK);	/* loop-back cmd */
	for (j=0;j<(i-1);j++)
	{
		if (HAL_SPI_TRANSFER(ram_tx_buffer[j+1]) != ram_tx_buffer[j])
		{
			PRINTF("uwb: Pattern 0x%02x FAILED - was 0x%02x\r\n", ram_tx_buffer[j], ram_tx_buffer[j]);
			return -1;
		}
	}
	HAL_SPI_TRANSFER_CLOSE();
  PRINTF("uwb: ... PASSED\r\n");
	
  delay_us(5000);

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  PRINTF("uwb: SPI RAM_TX test with DMA ...\r\n");
	i=0;
	ram_tx_buffer[i++] = CMD_WRITE_RAM_TX;
	ram_tx_buffer[i++] = 127;	
	for (j=0;j<ram_tx_buffer[1];j++)
		ram_tx_buffer[i++] = j;
	ram_tx_buffer[i++] = 0x00;	/* additional zero required by PHY ??? */
  PRINTF("uwb: starting dma write ... ");
	hal_spi_dma_transfer(ram_tx_buffer, NULL, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
	PRINTF("done\r\n");
  delay_us(10);
  PRINTF("uwb: starting dma read ... ");
	ram_tx_buffer[0] = CMD_READ_RAM_TX;
	hal_spi_dma_transfer(ram_tx_buffer, ram_rx_buffer, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
	PRINTF("done\r\n");
	if (memcmp(&ram_tx_buffer[1],&ram_rx_buffer[2],ram_tx_buffer[1]+1) == 0)
		PRINTF("uwb: ... PASSED\r\n");
	else
	{
		PRINTF("uwb: ... buffers DIFFER - FAILED\r\n");	
		return -1;
	}
		
  delay_us(10000);

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  PRINTF("uwb: SPI RAM_RX test with DMA ...\r\n");
	i=0;
	ram_tx_buffer[i++] = CMD_WRITE_RAM_RX;
	ram_tx_buffer[i++] = 127;	
	for (j=0;j<ram_tx_buffer[1];j++)
		ram_tx_buffer[i++] = j;
	ram_tx_buffer[i++] = 0x00;	/* additional zero required by PHY ??? */
  PRINTF("uwb: starting dma write ... ");
	hal_spi_dma_transfer(ram_tx_buffer, NULL, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
	PRINTF("done\r\n");
  delay_us(10);
  PRINTF("uwb: starting dma read ... ");
	ram_tx_buffer[0] = CMD_READ_RAM_RX;
	hal_spi_dma_transfer(ram_tx_buffer, ram_rx_buffer, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
	PRINTF("done\r\n");
	if (memcmp(&ram_tx_buffer[1],&ram_rx_buffer[2],ram_tx_buffer[1]+1) == 0)
		PRINTF("uwb: ... PASSED\r\n");
	else
	{
		PRINTF("uwb: ... buffers DIFFER - FAILED\r\n");	
		return -1;
	}
	
  delay_us(10000);

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  PRINTF("uwb: Internal loopback test with interrupts...\r\n");
	HAL_DISABLE_IRQ0( );
	HAL_DISABLE_IRQ1( );
	i=0;
	ram_tx_buffer[i++] = CMD_WRITE_RAM_TX;
	ram_tx_buffer[i++] = 96;	
	for (j=0;j<ram_tx_buffer[1];j++)
		ram_tx_buffer[i++] = ~j;
	ram_tx_buffer[i++] = 0x00;	/* additional zero required by PHY ??? */
//  PRINTF("uwb: starting dma write ... ");
	hal_spi_dma_transfer(ram_tx_buffer, NULL, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
//	PRINTF("done\r\n");
  delay_us(2);
	
	HAL_SPI_TRANSFER_OPEN();
	HAL_SPI_TRANSFER(CMD_CONFIG_MODE);	/* config mode cmd */
	HAL_SPI_TRANSFER(0x09); // oder 0x0A
	HAL_SPI_TRANSFER(0xFF); /* all off */    // für external test: 0x00
	HAL_SPI_TRANSFER(0x00); // für external 0x10-0x13
	HAL_SPI_TRANSFER(0xEF); // (MODULE_TX_RS | MODULE_TX | MODULE_RX_RS | MODULE_RX | MODULE_LOOPBACK);
	HAL_SPI_TRANSFER(0x11); /* Preamble length... */
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER_CLOSE();
		
	/* wait for ints */
	PRINTF("uwb: waiting for tx-interrupt ... ");
	/* encoding time depends on frame length - so make it length-dependend */
	for (i=0;i<ram_tx_buffer[1];i++)
	{
		if (HAL_CHECK_IRQ1())
			break;
		delay_us(12);
	}
	if (i < 100)
		PRINTF("ok\r\n");
	else
	{
		PRINTF("FAILED\r\n");	
	}

	/* wait for ints */
	PRINTF("uwb: waiting for rx-interrupt ... ");
	for (i=0;i<50;i++)
	{
		if (HAL_CHECK_IRQ0())
			break;
		delay_us(5);
	}
	if (i < 100)
		PRINTF("ok\r\n");
	else
	{
		PRINTF("FAILED\r\n");	
	}
		
	/* read frame */
//  PRINTF("uwb: starting frame download ... ");
	HAL_SS_LOW();
	HAL_SPI_TRANSFER(CMD_READ_RAM_RX);
	HAL_SPI_TRANSFER(0x00);
	i = HAL_SPI_TRANSFER(0x00);
	if (i>0)
	{
		hal_spi_dma_transfer(NULL, ram_rx_buffer, i);
		while (hal_spi_dma_busy()) {;}
	}
	HAL_SS_HIGH();
//	PRINTF("done\r\n");
		
  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

	HAL_ENABLE_IRQ0( );
	HAL_ENABLE_IRQ1( );

	if ((i == ram_tx_buffer[1]) && (memcmp(&ram_tx_buffer[2],ram_rx_buffer,i) == 0))
		PRINTF("uwb: ... PASSED\r\n");
	else
	{
		PRINTF("uwb: ... buffers DIFFER - FAILED\r\n");	
		return -1;
	}
	
  delay_us(10000);

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  PRINTF("uwb: External loopback test with interrupts...\r\n");
	HAL_DISABLE_IRQ0( );
	HAL_DISABLE_IRQ1( );
	i=0;
	ram_tx_buffer[i++] = CMD_WRITE_RAM_TX;
	ram_tx_buffer[i++] = 96;	
	for (j=0;j<ram_tx_buffer[1];j++)
		ram_tx_buffer[i++] = j;
	ram_tx_buffer[i++] = 0x00;	/* additional zero required by PHY ??? */
//  PRINTF("uwb: starting dma write ... ");
	hal_spi_dma_transfer(ram_tx_buffer, NULL, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
//	PRINTF("done\r\n");
  delay_us(2);
	
	HAL_SPI_TRANSFER_OPEN();
	HAL_SPI_TRANSFER(CMD_CONFIG_MODE);	/* config mode cmd */
	HAL_SPI_TRANSFER(0x09); // oder 0x0A
	HAL_SPI_TRANSFER(0x00); /* whole analog frontend on */
	HAL_SPI_TRANSFER(0x10); /* enable rx-antenna (0x10) and set gain (0x10-0x13) */
	HAL_SPI_TRANSFER(0x6F); // (MODULE_TX_RS | MODULE_TX | MODULE_RX_RS | MODULE_RX | MODULE_LOOPBACK);
	HAL_SPI_TRANSFER(0x11); /* Preamble length... */
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER(0x00);
	HAL_SPI_TRANSFER_CLOSE();
		
	/* wait for ints */
	PRINTF("uwb: waiting for tx-interrupt ... ");
	/* encoding time depends on frame length - so make it length-dependend */
	for (i=0;i<ram_tx_buffer[1];i++)
	{
		if (HAL_CHECK_IRQ1())
			break;
		delay_us(12);
	}
	if (i < ram_tx_buffer[1])
		PRINTF("ok\r\n");
	else
	{
		PRINTF("FAILED\r\n");	
	}

	/* wait for ints */
	PRINTF("uwb: waiting for rx-interrupt ... ");
	for (i=0;i<50;i++)
	{
		if (HAL_CHECK_IRQ0())
			break;
		delay_us(5);
	}
	if (i < 50)
		PRINTF("ok\r\n");
	else
	{
		PRINTF("FAILED\r\n");	
	}
		
	/* read frame */
//  PRINTF("uwb: starting frame download ... ");
	HAL_SS_LOW();
	HAL_SPI_TRANSFER(CMD_READ_RAM_RX);
	HAL_SPI_TRANSFER(0x00);
	i = HAL_SPI_TRANSFER(0x00);
	if (i>0)
	{
		hal_spi_dma_transfer(NULL, ram_rx_buffer, i);
		while (hal_spi_dma_busy()) {;}
	}
	HAL_SS_HIGH();
//	PRINTF("done\r\n");
		
	HAL_ENABLE_IRQ0( );
	HAL_ENABLE_IRQ1( );

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

	if ((i == ram_tx_buffer[1]) && (memcmp(&ram_tx_buffer[2],ram_rx_buffer,i) == 0))
		PRINTF("uwb: ... PASSED\r\n");
	else
	{
		PRINTF("uwb: ... buffers DIFFER - FAILED\r\n");	
		return -1;
	}
	
		
	return 0;
}
	
int init(void)
{
  /* Wait in case VCC just applied */
  delay_us(100);
  /* Initialize Hardware Abstraction Layer */
  hal_init();
 
  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(10);
  HAL_DEASSERT_RST();
  delay_us(10);

  PRINTF("uwb: Reset\r\n");
	
  if (run_tests() == 0)
		PRINTF("uwb: ALL TESTS PASSED *****************\r\n");
	else
		PRINTF("uwb: AT LEAST ONE TEST FAILED *****************\r\n");
		

 /* Start the packet receive process */
  process_start(&uwb_process, NULL);
 
 /* Leave radio in on state (?)*/
  on();

  return 1;
}

int uwb_rx_interrupt(void)
{
  /* Poll the receive process */
//  process_poll(&uwb_process);
  
//  RIMESTATS_ADD(llrx);
  return 1;
}

int uwb_tx_interrupt(void)
{
	/* TX finished */

  /* Do full uwb-phy Reset */
	// does not work in self test - as this breaks rx
//  HAL_ASSERT_RST();
//  delay_us(2);
//  HAL_DEASSERT_RST();

  return 1;
}

/*---------------------------------------------------------------------------*/
/* Process to handle input packets
 * Receive interrupts cause this process to be polled
 * It calls the core MAC layer which calls uwb_read to get the packet
 * uwbprocessflag can be printed in the main idle loop for debugging
 */
#if 0
uint8_t uwbprocessflag;
#define UWBPROCESSFLAG(arg) uwbprocessflag=arg
#else
#define UWBPROCESSFLAG(arg)
#endif

PROCESS_THREAD(uwb_process, ev, data)
{
  PROCESS_BEGIN();
  UWBPROCESSFLAG(99);

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    UWBPROCESSFLAG(42);

//    len = rf230_read(packetbuf_dataptr(), PACKETBUF_SIZE);        

    PRINTF("uwb_rx\n");

		/* Do full uwb-phy Reset */
		HAL_ASSERT_RST();
		delay_us(10);
		HAL_DEASSERT_RST();

    UWBPROCESSFLAG(1);
  }

  PROCESS_END();
}
