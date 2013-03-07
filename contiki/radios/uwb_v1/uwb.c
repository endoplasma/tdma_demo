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

/* finite state machine */
#define UWB_STATE_OFF                  0 /* in reset */
#define UWB_STATE_ON                   1 /* after reset, without RX init */
#define UWB_STATE_IN_TEST              2 /* when tests are done */

#define UWB_STATE_LISTEN              11 /* after RX init or frame reception or transmission */
#define UWB_STATE_RX_LEN_DOWNLOAD     12 /* after RX interrupt - set in rx-int, dma started for length-download */
#define UWB_STATE_RX_FRAME_DOWNLOAD   13 /* after DMA interrupt in RXD-state - set in dma-int, dma started for frame download */
#define UWB_STATE_RX_PROCESS_POLLED   14 /* after DMA interrupt in DOWNLOAD-state - set in dma-int, uwbprocess polled, gets reset to LISTEN by uwbprocess */

#define USB_STATE_TX_FRAME_UPLOAD     21 /* set when the TX RAM gets loaded by DMA, gets reset to LISTEN (e.g. by dma-int) or can lead to CONFIG_UPLOAD */
#define USB_STATE_TX_CONFIG_UPLOAD    22 /* set when the CONFIG gets loaded by DMA, gets promoted to TX_ACTIVE by dma-int */
#define USB_STATE_TX_ACTIVE           23 /* set when the TX config was sent and the PHY is supposed to transmit, gets reset to LISTEN (e.g. by tx-int) */

uint8_t volatile uwb_fsm;

/*---------------------------------------------------------------------------*/
PROCESS(uwb_process, "UWB driver");
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static int init(void);
/*---------------------------------------------------------------------------*/



static int
enable_rx_mode(void)
{
  uint8_t cmd_buffer[16];

  /* keep silence here, as this is called in int-context */

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(1);
  HAL_DEASSERT_RST();
  delay_us(1);

  /* enable rx */
  cmd_buffer[0] = CMD_CONFIG_MODE;
  cmd_buffer[1] = 0x09;
  cmd_buffer[2] = 0xFC;
  cmd_buffer[3] = 0x14;
  cmd_buffer[4] = 0x28;
  cmd_buffer[5] = 0x00;
  /* just make sure there is no dma underway */
  while (hal_spi_dma_busy()) {;}
  hal_spi_dma_transfer(cmd_buffer, NULL, 6);

  HAL_ENABLE_IRQ0( );	/* enable rx-int */

  uwb_fsm = UWB_STATE_LISTEN;

  return 1;
}

static int
prepare(const void *payload, unsigned short payload_len)
{
  if (payload_len > 126)
    return RADIO_TX_ERR;

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(1);
  HAL_DEASSERT_RST();
//  delay_us(1);

  PRINTF("uwb: frame upload ... \r\n");
  ram_tx_buffer[0] = CMD_WRITE_RAM_TX;
  ram_tx_buffer[1] = (uint8_t)payload_len;
  memcpy(&ram_tx_buffer[2],payload,payload_len);
  ram_tx_buffer[payload_len+2] = 0x00;	/* additional zero required by PHY ??? */
//  PRINTF("uwb: starting dma write ... ");
  /* just make sure there is no dma underway */
  while (hal_spi_dma_busy()) {;}
  hal_spi_dma_transfer(ram_tx_buffer, NULL, ram_tx_buffer[1]+3);
//  PRINTF("done\r\n");

  uwb_fsm = USB_STATE_TX_FRAME_UPLOAD;

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
  uint8_t cmd_buffer[16];
  uint16_t i;

  if ((uwb_fsm != UWB_STATE_ON) && (uwb_fsm != UWB_STATE_LISTEN) && (uwb_fsm != USB_STATE_TX_FRAME_UPLOAD))
  {
    PRINTF("uwb: transmit - WRONG STATE: %u\r\n", uwb_fsm);	
    return RADIO_TX_ERR;
  }

  PRINTF("uwb: transmitting ... ");
  cmd_buffer[0] = CMD_CONFIG_MODE;
  cmd_buffer[1] = 0x0A;
  cmd_buffer[2] = 0xE3;
  cmd_buffer[3] = 0x00;
  cmd_buffer[4] = 0x0e; // should be 0x06 ???
  cmd_buffer[5] = 0x11;
  cmd_buffer[6] = 0x00;
  cmd_buffer[7] = 0x00;
  cmd_buffer[8] = 0x00;
  cmd_buffer[9] = 0x00;
  cmd_buffer[10] = 0x00;
  /* wait for previous dma to finish (e.g. "prepare") */
  while (hal_spi_dma_busy()) {;}
  hal_spi_dma_transfer(cmd_buffer, NULL, 11);

  uwb_fsm = USB_STATE_TX_CONFIG_UPLOAD;

  /* right here we could defer and wait for interrupt */

  /* wait for state TX_ACTIVE (which is set by dma-int) */
  while (uwb_fsm == USB_STATE_TX_CONFIG_UPLOAD) {;}

  /* wait for state LISTEN */
  /* encoding time depends on frame length - so make it length-dependend */
//  for (i=0;i<transmit_len;i++)
  for (i=0;i<100;i++)
  {
    if (uwb_fsm == UWB_STATE_LISTEN)
      break;
    delay_us(12);
  }

  /* here we could wait for an ACK if implemented - judge on tx-int for now */
//  if (i < transmit_len)
  if (i < 100)
  {
    PRINTF("SUCCESS\r\n");
    return RADIO_TX_OK;
  }
  else
  {
    PRINTF("no tx-interrupt - FAILED\r\n");	
    return RADIO_TX_ERR;
  }
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
  return (uwb_fsm > UWB_STATE_LISTEN);
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  PRINTF("uwb: turning radio ON\r\n");
  HAL_DEASSERT_RST();
  uwb_fsm = UWB_STATE_ON;
  PRINTF("uwb: enabling rx-mode\r\n");
  enable_rx_mode();
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  PRINTF("uwb: turning radio OFF\r\n");
  uwb_fsm = UWB_STATE_OFF;
  HAL_ASSERT_RST();
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
  uint8_t failed = 0;

  uwb_fsm = UWB_STATE_IN_TEST;
  HAL_DISABLE_IRQ0( );
  HAL_DISABLE_IRQ1( );

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
    ram_rx_buffer[j] = HAL_SPI_TRANSFER(ram_tx_buffer[j+1]);
    if (ram_rx_buffer[j] != ram_tx_buffer[j])
    {
      PRINTF("uwb: Pattern 0x%02x FAILED - was 0x%02x\r\n", ram_tx_buffer[j], ram_rx_buffer[j]);
      failed++;
      break;
    }
  }
  HAL_SPI_TRANSFER_CLOSE();
  if (failed == 0)
    PRINTF("uwb: ... PASSED\r\n");

  delay_us(10000);

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
    failed++;
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
    failed++;
  }

  delay_us(10000);

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  /* we need to cycle IRQs here to make sure that there are none pending - as we later check for ints! */
  /* do NOT remove unless you know what you are doing!!! */
  HAL_ENABLE_IRQ0();
  HAL_ENABLE_IRQ1();
  HAL_DISABLE_IRQ0();
  HAL_DISABLE_IRQ1();

  PRINTF("uwb: Internal loopback test with interrupts...\r\n");
  i=0;
  ram_tx_buffer[i++] = CMD_WRITE_RAM_TX;
  ram_tx_buffer[i++] = 96;
  for (j=0;j<ram_tx_buffer[1];j++)
    ram_tx_buffer[i++] = ~j;
  ram_tx_buffer[i++] = 0x00;	/* additional zero required by PHY ??? */
//  PRINTF("uwb: starting dma write ... ");
  hal_spi_dma_transfer(ram_tx_buffer, NULL, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
//  PRINTF("done\r\n");
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
    failed++;
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
    failed++;
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
//  PRINTF("done\r\n");

  /* do reset to release IRQs */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  if ((i == ram_tx_buffer[1]) && (memcmp(&ram_tx_buffer[2],ram_rx_buffer,i) == 0))
    PRINTF("uwb: ... PASSED\r\n");
  else
  {
    if (i != ram_tx_buffer[1])
      PRINTF("uwb: ... length is %u, expected %u - FAILED\r\n", i, ram_tx_buffer[1]);
    else
      PRINTF("uwb: ... buffers differ - FAILED\r\n");
    failed++;
  }

  delay_us(10000);

  /* Do full uwb-phy Reset */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);
	
	/* we need to cycle IRQs here to make sure that there are none pending - as we later check for ints! */
	/* do NOT remove unless you know what you are doing!!! */
	HAL_ENABLE_IRQ0();
	HAL_ENABLE_IRQ1();
	HAL_DISABLE_IRQ0();
	HAL_DISABLE_IRQ1();

  PRINTF("uwb: External loopback test with interrupts...\r\n");
  i=0;
  ram_tx_buffer[i++] = CMD_WRITE_RAM_TX;
  ram_tx_buffer[i++] = 96;
  for (j=0;j<ram_tx_buffer[1];j++)
    ram_tx_buffer[i++] = j;
  ram_tx_buffer[i++] = 0x00;	/* additional zero required by PHY ??? */
//  PRINTF("uwb: starting dma write ... ");
  hal_spi_dma_transfer(ram_tx_buffer, NULL, ram_tx_buffer[1]+3);
  while (hal_spi_dma_busy()) {;}
//  PRINTF("done\r\n");
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
    delay_us(20);
  }
  if (i < ram_tx_buffer[1])
    PRINTF("ok\r\n");
  else
  {
    PRINTF("FAILED\r\n");
    failed++;
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
    failed++;
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
//  PRINTF("done\r\n");

  /* do reset to release IRQs */
  HAL_ASSERT_RST();
  delay_us(2);
  HAL_DEASSERT_RST();
  delay_us(2);

  if ((i == ram_tx_buffer[1]) && (memcmp(&ram_tx_buffer[2],ram_rx_buffer,i) == 0))
    PRINTF("uwb: ... PASSED\r\n");
  else
  {
    if (i != ram_tx_buffer[1])
      PRINTF("uwb: ... length is %u, expected %u - FAILED\r\n", i, ram_tx_buffer[1]);
    else
      PRINTF("uwb: ... buffers differ - FAILED\r\n");
    failed++;
  }

  return failed;
}

int init(void)
{
  /* Wait in case VCC just applied */
  delay_us(100);
  /* Initialize Hardware Abstraction Layer */
  hal_init();

  /* Do uwb-phy Reset */
  HAL_ASSERT_RST();
  uwb_fsm = UWB_STATE_OFF;
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

void uwb_dma_interrupt(void)
{
  /* DMA transaction finished */
  if (uwb_fsm == UWB_STATE_RX_LEN_DOWNLOAD)
  {
    /* we downloaded the frame length - now download the frame! */
    if (ram_rx_buffer[2] > 0)
    {
      ram_tx_buffer[0] = CMD_READ_RAM_RX;
      hal_spi_dma_transfer(ram_tx_buffer, ram_rx_buffer, ram_rx_buffer[2]+3);
      uwb_fsm = UWB_STATE_RX_FRAME_DOWNLOAD;
    }
    else
    {
      /* directly poll process to handle zero-length frame */
      process_poll(&uwb_process);
      RIMESTATS_ADD(llrx);
      uwb_fsm = UWB_STATE_RX_PROCESS_POLLED;
    }
  }
  else if (uwb_fsm == UWB_STATE_RX_FRAME_DOWNLOAD)
  {
    /* download complete, poll process */
    process_poll(&uwb_process);
    RIMESTATS_ADD(llrx);
    uwb_fsm = UWB_STATE_RX_PROCESS_POLLED;
  }
  else if (uwb_fsm == USB_STATE_TX_FRAME_UPLOAD)
  {
    /* tx frame uploaded, move back to listen state */
    /* here we could also implement a mechanism that starts the tx by sending config right away */
    uwb_fsm = UWB_STATE_LISTEN;
//    PRINTF("d1");
  }
  else if (uwb_fsm == USB_STATE_TX_CONFIG_UPLOAD)
  {
    /* tx config uploaded, assume the phy is transmitting now */
    uwb_fsm = USB_STATE_TX_ACTIVE;
    /* enable tx-int */
    HAL_ENABLE_IRQ1( );
//    PRINTF("d2");
  }
}

void uwb_rx_interrupt(void)
{
  /* frame received */

  leds_on(LEDS_YELLOW);
  /* Disable rx-int */
  HAL_DISABLE_IRQ0( );

  /* start DMA download of length field */
  ram_tx_buffer[0] = CMD_READ_RAM_RX;
  hal_spi_dma_transfer(ram_tx_buffer, ram_rx_buffer, 3);
  uwb_fsm = UWB_STATE_RX_LEN_DOWNLOAD;
}

void uwb_tx_interrupt(void)
{
  /* TX finished */
  /* this should only happen in USB_STATE_TX_ACTIVE ! */

  /* disable tx-int */
  HAL_DISABLE_IRQ1();

  /* switch back to listen-state */
  enable_rx_mode();

//  PRINTF("d3");
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
  int len;
  PROCESS_BEGIN();
  UWBPROCESSFLAG(01);

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    UWBPROCESSFLAG(02);

    if (uwb_fsm != UWB_STATE_RX_PROCESS_POLLED)
      PRINTF("uwb: wrong state in process: %u\n",uwb_fsm);			

    packetbuf_clear();

    len = ram_rx_buffer[2];
    if ((len > 0) && (len < PACKETBUF_SIZE))
    {
      memcpy(packetbuf_dataptr(),&ram_rx_buffer[3],len); 
      packetbuf_set_datalen(len);
      UWBPROCESSFLAG(03);
      NETSTACK_RDC.input();
    }

    /* switch back to listen-state */
    PRINTF("uwb: enabling rx-mode\r\n");
    enable_rx_mode();

    PRINTF("uwb: read: %u bytes\n",len);

    UWBPROCESSFLAG(04);
  }

  PROCESS_END();
}
