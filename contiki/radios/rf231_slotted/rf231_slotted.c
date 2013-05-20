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
#include "dev/button-sensor.h"
#include "rf231_slotted_registermap.h"
#include "slotted_frame.h"

#ifdef SLOTTED_KOORDINATOR
#ifdef JITTER_SIMULATION
#warning Building for Slotted Koordinator in simulation mode
#else
#warning Building for Slotted Koordinator

#endif /* JITTER_SIMULATION */
#else
#warning Building for Slotted Client
#endif /* SLOTTED_KOORDINATOR */


/*---------------------------------------------------------------------------*/
/*****************************************************************************
 * Variable Definitions
 *****************************************************************************/
PROCESS(rf231_slotted_process, "Slotted TDMA Driver");

uint8_t volatile rf231_pending;
static uint8_t buffer[RF230_MAX_TX_FRAME_LENGTH];   /**< frame buffer */
static uint8_t sqn = 0;                             /**< the TDMA sequence Number */
static uint16_t nodeAddress = 0;                    /**< the mac address of the node */

#ifdef SLOTTED_KOORDINATOR
static uint8_t dummyPayload[BEACON_PAYLOAD_LENGTH]; /**< Koordinator dummy Paket */
#else
static uint8_t dummyPayload[RESPONSE_PAYLOAD_LENGTH]; /**< Client dummy Paket */
#endif

/**
 * A struct to store information about the state and defines some basic global variables
 */
typedef struct ringBuffer{
  uint32_t Buff[NUM_PERIODS];                   /** Array to store the last measured Periods */
  uint8_t PutPos;                               /** Position to store the next measurement */
  uint8_t Count;                                /** The number of stored Periods */
}ringBuffer;

uint32_t slot;
uint32_t Period;                                /** The length of our send Period */
static ringBuffer PeriodBuffer;                 /** A ring buffer to store the last measured periods */

uint32_t lastBeaconTime;

/**
 * A Struct that stores all Inforation used to configure the behaviour
 * of the TDMA Protocol.
 */
struct protConf{
  uint32_t numClients;             /** the nuber max number of
				       connected clients */
  uint32_t Period;                 /** The actual calulated Period */
  uint32_t clientSlotLength;       /** Slot length for a Cient in us
				       (192 + m*32 with m = length of
				       the packet) */
  uint32_t GuardInterval;          /** Guard INterval length in us */
  uint32_t beaconCount;            /** number of consecutive received
				       beacons */ 
}protocolConfig;

/* Received frames are buffered to rxframe in the interrupt routine in
   hal.c */
uint8_t rxframe_head,rxframe_tail;
hal_rx_frame_t rxframe[RF230_CONF_RX_BUFFERS];

extern uint32_t slotTime;

/**
 * The possible FSM states of the rf231 slotted process
 */
#define RF231_STATE_UNINIT           0 /** Uninitialised Statemachine */
#define RF231_STATE_INACTIVE         1 /** Machine deactivated */
#define RF231_STATE_IDLE             2 /** Koordinator idle */
#define RF231_STATE_HANDLE_PACKET    3 /** Packet received, busy handling it */
#define RF231_STATE_ACTIVE_PLL       4 /** PLL for sending activated */
#define RF231_STATE_SEND             5 /** Sending a packet */

#define RF231_STATE_SYNCHED          6 /** Client synchronised */
#define RF231_STATE_UNSYNCHED        7 /** Client unsynchronised */
#define RF231_STATE_BEACON_MISSED    8 /** Client detected a missed beacon */


uint8_t  volatile state = RF231_STATE_UNINIT;

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
/******************************************************************************
 * Prototypes
 ******************************************************************************/
int rf231_init(void);
int rf231_on(void);
int rf231_off(void);
static void rf231_warm_reset(void);
static void rf231_reset_state_machine(void);
static void rf231_wait_idle(void);
static bool rf231_is_ready_to_sendy(void);
uint8_t rf231_get_trx_state(void);
radio_status_t rf231_set_trx_state(uint8_t new_state);
void rf231_upload_packet(unsigned short payload_len);

static int rf231_read(void *buf, unsigned short bufsize);
static int rf231_prepare(const void *data, unsigned short len);
static int rf231_transmit(unsigned short len);
static int rf231_send(const void *data, unsigned short len);
static int rf231_receiving_packet(void);
static int rf231_pending_packet(void);
static int rf231_cca(void);

static int create_packet(void);
 
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
 
/*****************************************************************************
 * Radio Driver API Function definition
 *****************************************************************************/
/*---------------------------------------------------------------------------*/
 static void
 on(void)
{
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
#ifdef RF230BB_HOOK_RADIO_ON
  RF230BB_HOOK_RADIO_ON();
#endif

/* If radio is off (slptr high), turn it on */
  if (hal_get_slptr()) {
    ENERGEST_ON(ENERGEST_TYPE_LED_RED);

/* SPI based radios. The wake time depends on board capacitance.  Make
 * sure the delay is long enough, as using SPI too soon will reset the
 * MCU!  Use 2x the nominal value for safety. 1.5x is not long enough
 * for Raven!
 */
    hal_set_slptr_low();
    delay_us(2*TIME_SLEEP_TO_TRX_OFF);
  }

  /* if tdma koord: go directly to send mode after activation 
  * otherwise start listening */
#ifdef SLOTTED_KOORDINATOR
  rf231_set_trx_state(PLL_ON);
#else /* SLOTTED_KOORDINATOR */
  rf231_set_trx_state(RX_AACK_ON);
#endif /* SLOTTED_KOORDINATOR */

  rf231_wait_idle();
}


/*----------------------------------------------------------------------------*/
/**
 * \brief  turn off the radio.
 * \param  void
 * \return void 
 * 
 * This function turns the Hardware radio off. If there are any turn
 * off hooks defined they will bi executed before the radio is shut
 * down. Any ongoning transmission will be completed before the
 * hardware os turned off.
 */
 static void
off(void)
{
#ifdef RF230BB_HOOK_RADIO_OFF
  RF230BB_HOOK_RADIO_OFF();
#endif

  /* Wait for any transmission to end */
  rf231_wait_idle(); 

#if RADIOALWAYSON
/* Do not transmit autoacks when stack thinks radio is off */
  rf231_set_trx_state(RX_ON);
#else 
  /* Force the device into TRX_OFF. */   
  rf231_reset_state_machine();
#if RADIOSLEEPSWHENOFF

  /* Sleep Radio */
  hal_set_slptr_high();
  ENERGEST_OFF(ENERGEST_TYPE_LED_RED);
#endif
#endif /* RADIOALWAYSON */

   ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
}


/*----------------------------------------------------------------------------*/
/**
 * \brief  Initialise the rf231 slotted driver
 * \param  void
 * \return int - 1 if the initialisation was successfull
 *               0 otherwise
 * 
 * This function initialises the rf231_slotted driver. Address
 * configuration is done and the radio is turned on and starts sending
 * beacons or listens for them (depending on KOORDINATOR or CLIENT
 * Mode)
 * 
 */
 int
rf231_init(void)
{
  uint8_t i;
  uint8_t tvers, tmanu;

  PeriodBuffer.PutPos = 0;
  PeriodBuffer.Count = 0;
  protocolConfig.GuardInterval = 0;
  protocolConfig.Period = 0;
  protocolConfig.numClients = 0;
  protocolConfig.clientSlotLength = 0;
  protocolConfig.beaconCount = 0;

  state = RF231_STATE_INACTIVE;

/* set the address of this node */
  nodeAddress = (((*((uint8_t*)0x1FFF7A10+2)) << 8) | (*((uint8_t*)0x1FFF7A10+0)) );


/* set the dummy Payload with 0 bits */
#ifdef SLOTTED_KOORDINATOR
  memset(&dummyPayload, 0, sizeof(uint8_t) * BEACON_PAYLOAD_LENGTH);
#else
  memset(&dummyPayload, 0, sizeof(uint8_t) * RESPONSE_PAYLOAD_LENGTH);
#endif


  /* create a dummy packet */
#ifdef SLOTTED_KOORDINATOR
  buffer[0]=0xa0;            /* fcf*/
  buffer[1]=0x06;
  buffer[2]=0x00;            /* sqn */
  buffer[3]=TDMA_PAN_ID_1;   /* src PAN ID */
  buffer[4]=TDMA_PAN_ID_0;
  buffer[5]=(*((uint8_t*)0x1FFF7A10+2));       /* src address */
  buffer[6]=(*((uint8_t*)0x1FFF7A10+0));
  buffer[7]=0x03;                              /* cycle time */
  buffer[8]=0xe8;
  buffer[9]=0x03;                              /* clients */
  buffer[10]=0x11;          /* Payload start */
  buffer[11]=0x11;
  buffer[12]=0x11;
  buffer[13]=0x11;
  buffer[14]=0x22;         /* PL 2*/
  buffer[15]=0x22;
  buffer[16]=0x22;
  buffer[17]=0x22;
  buffer[18]=0x33;        /* pl 3 */
  buffer[19]=0x33;
  buffer[20]=0x33;
  buffer[21]=0x33;
  buffer[22]=0x00;        /* FCS */
  buffer[23]=0x00;
#else
  buffer[0]=0xa2;            /* fcf*/
  buffer[1]=0x26;
  buffer[2]=0x00;            /* sqn */
  buffer[3]=TDMA_PAN_ID_1;   /* dst PAN ID */
  buffer[4]=TDMA_PAN_ID_0;
  buffer[5]=0x2e;            /* dst address of koord */
  buffer[6]=0x1b;
  buffer[7]=(*((uint8_t*)0x1FFF7A10+2));       /* dst address */
  buffer[8]=(*((uint8_t*)0x1FFF7A10+0));
  buffer[9]=0x04;                              /* payload length */
  buffer[10]=0x11;          /* Payload start */
  buffer[11]=0x22;
  buffer[12]=0x33;
  buffer[13]=0x44;
  buffer[14]=0x00;         /* PL 2*/
  buffer[15]=0x00;
#endif

  /* Wait in case VCC just applied */
  delay_us(TIME_TO_ENTER_P_ON);
  /* Initialize Hardware Abstraction Layer */
  hal_init();

  /* Set receive buffers empty and point to the first */
  for (i=0;i<RF230_CONF_RX_BUFFERS;i++) rxframe[i].length=0;
  rxframe_head=0;rxframe_tail=0;
  
  /* Do full rf230 Reset */
  hal_set_rst_low();
  hal_set_slptr_low();

  /* On powerup a TIME_RESET delay is needed here, however on some other MCU reset
   * (JTAG, WDT, Brownout) the radio may be sleeping. It can enter an uncertain
   * state (sending wrong hardware FCS for example) unless the full wakeup delay
   * is done.
   * Wake time depends on board capacitance; use 2x the nominal delay for safety.
   * See www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=78725
   */
  delay_us(2*TIME_SLEEP_TO_TRX_OFF);
  hal_set_rst_high();

  /* Force transition to TRX_OFF */
  hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
  delay_us(TIME_P_ON_TO_TRX_OFF);
  
  /* Verify that it is a supported version */
  /* Note gcc optimizes this away if DEBUG is not set! */
  tvers = hal_register_read(RG_VERSION_NUM);
  tmanu = hal_register_read(RG_MAN_ID_0);
  
  if((tvers != RF230_REVA) && (tvers != RF230_REVB)) {
    PRINTF("rf230: Unsupported version %u\n",tvers);
  }
  if(tmanu != SUPPORTED_MANUFACTURER_ID) {
    PRINTF("rf230: Unsupported manufacturer ID %u\n",tmanu); 
  } 
  PRINTF("rf230: Version %u, ID %u\n",tvers,tmanu);
  
  rf231_warm_reset();

  process_start(&rf231_slotted_process, NULL);

  on();

#ifdef SLOTTED_KOORDINATOR
  rf231_set_trx_state(PLL_ON);
  rf231_upload_packet(24);
#else
  rf231_set_trx_state(RX_AACK_ON);
#endif

  hal_register_write(RG_IRQ_MASK, RF230_SUPPORTED_INTERRUPT_MASK);

#ifdef SLOTTED_KOORDINATOR
  hal_set_TX_Timer(TDMA_PERIOD_TICKS);
#endif
  hal_reset_counter();

  return 1;
}

/*---------------------------------------------------------------------------*/
/* Used to reinitialize radio parameters without losing pan and mac address, channel, power, etc. */
void
rf231_warm_reset(void) {

  /* Configure the interrupts */
  hal_register_write(RG_IRQ_MASK, RF230_SUPPORTED_INTERRUPT_MASK);
  uint8_t status = hal_register_read(RG_IRQ_MASK);

  /* Set up number of automatic retries 0-15 (0 implies PLL_ON sends
   * instead of the extended TX_ARET mode */
  hal_subregister_write(SR_MAX_FRAME_RETRIES, 0 );
 
  /* Set up carrier sense/clear channel assesment parameters for
   * extended operating mode */
  hal_subregister_write(SR_MAX_CSMA_RETRIES, 0 );

  /* set the short address and panID */
  hal_subregister_write(SR_SHORT_ADDR_0, (*((uint8_t*)0x1FFF7A10+0)));  
  hal_subregister_write(SR_SHORT_ADDR_1, (*((uint8_t*)0x1FFF7A10+2)));  
  hal_subregister_write(SR_PAN_ID_0, TDMA_PAN_ID_0);
  hal_subregister_write(SR_PAN_ID_1, TDMA_PAN_ID_1); 

  /* set IEEE address */
  hal_subregister_write(SR_IEEE_ADDR_0, (*((uint8_t*)0x1FFF7A10+0)));
  hal_subregister_write(SR_IEEE_ADDR_1, (*((uint8_t*)0x1FFF7A10+2)));
  hal_subregister_write(SR_IEEE_ADDR_2, (*((uint8_t*)0x1FFF7A10+4)));
  hal_subregister_write(SR_IEEE_ADDR_3, 0xfe);
  hal_subregister_write(SR_IEEE_ADDR_4, 0xff);
  hal_subregister_write(SR_IEEE_ADDR_5, 0x00);
  hal_subregister_write(SR_IEEE_ADDR_6, 0x00);
  hal_subregister_write(SR_IEEE_ADDR_7, 0x02);

  /* frame version settings */
  hal_subregister_write(SR_AACK_FVN_MODE, 3);
  /* set as PAN koordiantor */
  hal_subregister_write(SR_I_AM_COORD, 1 );
  /* disable auto acknowledge mode */ 
  hal_subregister_write(SR_AACK_DIS_ACK, 0 );  
  /* enable reserved frame filtering */
  hal_subregister_write(SR_AACK_FLTR_RES_FT, 1);
  hal_subregister_write(SR_AACK_UPLD_RES_FT, 0);

  /* enable promiscious mode for testing */
  hal_subregister_write(SR_AACK_PROM_MODE, 1);

  /* Receiver sensitivity. If nonzero rf231/128rfa1 saves 0.5ma in rx mode */
  /* Not implemented on rf230 but does not hurt to write to it */
#ifdef RF230_MIN_RX_POWER
#if RF230_MIN_RX_POWER > 84
#warning rf231 power threshold clipped to -48dBm by hardware register
 hal_register_write(RG_RX_SYN, 0xf);
#elif RF230_MIN_RX_POWER < 0
#error RF230_MIN_RX_POWER can not be negative!
#endif
  hal_register_write(RG_RX_SYN, RF230_MIN_RX_POWER/6 + 1); //1-15 -> -90 to -48dBm
#endif

  /* CCA energy threshold = -91dB + 2*SR_CCA_ED_THRESH. Reset defaults to -77dB */
  /* Use RF230 base of -91;  RF231 base is -90 according to datasheet */
#ifdef RF230_CONF_CCA_THRES
#if RF230_CONF_CCA_THRES < -91
#warning
#warning RF230_CONF_CCA_THRES below hardware limit, setting to -91dBm
#warning
  hal_subregister_write(SR_CCA_ED_THRES,0);  
#elif RF230_CONF_CCA_THRES > -61
#warning
#warning RF230_CONF_CCA_THRES above hardware limit, setting to -61dBm
#warning
  hal_subregister_write(SR_CCA_ED_THRES,15);  
#else
  hal_subregister_write(SR_CCA_ED_THRES,(RF230_CONF_CCA_THRES+91)/2);  
#endif
#endif

  /* Use automatic CRC unless manual is specified */
#if RF230_CONF_CHECKSUM
  hal_subregister_write(SR_TX_AUTO_CRC_ON, 0);
#else
  hal_subregister_write(SR_TX_AUTO_CRC_ON, 1);
#endif

#if RF231_HAS_PA
  hal_subregister_write(SR_PA_EXT_EN, 1);
#endif

/* Limit tx power for testing miniature Raven mesh */
#ifdef RF230_MAX_TX_POWER
  set_txpower(RF230_MAX_TX_POWER);  //0=3dbm 15=-17.2dbm
#endif

  
}


/******************************************************************************
 * Local static Functions
 ******************************************************************************/
/*----------------------------------------------------------------------------*/
/** \brief  This function return the Radio Transceivers current state.
 *
 *  \retval     P_ON               When the external supply voltage (VDD) is
 *                                 first supplied to the transceiver IC, the
 *                                 system is in the P_ON (Poweron) mode.
 *  \retval     BUSY_RX            The radio transceiver is busy receiving a
 *                                 frame.
 *  \retval     BUSY_TX            The radio transceiver is busy transmitting a
 *                                 frame.
 *  \retval     RX_ON              The RX_ON mode enables the analog and digital
 *                                 receiver blocks and the PLL frequency
 *                                 synthesizer.
 *  \retval     TRX_OFF            In this mode, the SPI module and crystal
 *                                 oscillator are active.
 *  \retval     PLL_ON             Entering the PLL_ON mode from TRX_OFF will
 *                                 first enable the analog voltage regulator. The
 *                                 transceiver is ready to transmit a frame.
 *  \retval     BUSY_RX_AACK       The radio was in RX_AACK_ON mode and received
 *                                 the Start of Frame Delimiter (SFD). State
 *                                 transition to BUSY_RX_AACK is done if the SFD
 *                                 is valid.
 *  \retval     BUSY_TX_ARET       The radio transceiver is busy handling the
 *                                 auto retry mechanism.
 *  \retval     RX_AACK_ON         The auto acknowledge mode of the radio is
 *                                 enabled and it is waiting for an incomming
 *                                 frame.
 *  \retval     TX_ARET_ON         The auto retry mechanism is enabled and the
 *                                 radio transceiver is waiting for the user to
 *                                 send the TX_START command.
 *  \retval     RX_ON_NOCLK        The radio transceiver is listening for
 *                                 incomming frames, but the CLKM is disabled so
 *                                 that the controller could be sleeping.
 *                                 However, this is only true if the controller
 *                                 is run from the clock output of the radio.
 *  \retval     RX_AACK_ON_NOCLK   Same as the RX_ON_NOCLK state, but with the
 *                                 auto acknowledge module turned on.
 *  \retval     BUSY_RX_AACK_NOCLK Same as BUSY_RX_AACK, but the controller
 *                                 could be sleeping since the CLKM pin is
 *                                 disabled.
 *  \retval     STATE_TRANSITION   The radio transceiver's state machine is in
 *                                 transition between two states.
 */
uint8_t
rf231_get_trx_state(void)
{
    return hal_subregister_read(SR_TRX_STATUS);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will reset the state machine (to TRX_OFF) from any of
 *          its states, except for the SLEEP state.
 */
static void
rf231_reset_state_machine(void)
{
    if (hal_get_slptr()) DEBUGFLOW('"');
    hal_set_slptr_low();
    delay_us(TIME_NOCLK_TO_WAKE);
    hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
    delay_us(TIME_CMD_FORCE_TRX_OFF);
}

/*---------------------------------------------------------------------------*/
static char
rf230_isidle(void)
{
  uint8_t radio_state;
  if (hal_get_slptr()) {
    DEBUGFLOW(']');
	return 1;
  } else {
  radio_state = hal_subregister_read(SR_TRX_STATUS);
  if (radio_state != BUSY_TX_ARET &&
      radio_state != BUSY_RX_AACK &&
      radio_state != STATE_TRANSITION &&
      radio_state != BUSY_RX && 
      radio_state != BUSY_TX) {
    return(1);
  } else {
//    printf(".%u",radio_state);
    return(0);
  }
  }
}

static void
rf231_wait_idle(void)
{
int i;
  for (i=0;i<10000;i++) {  //to avoid potential hangs
 // while (1) {
    if (rf230_isidle()) break;
  }
  if (i>=10000) {DEBUGFLOW('H');DEBUGFLOW('R');}
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will change the current state of the radio
 *          transceiver's internal state machine.
 *
 *  \param     new_state        Here is a list of possible states:
 *             - RX_ON        Requested transition to RX_ON state.
 *             - TRX_OFF      Requested transition to TRX_OFF state.
 *             - PLL_ON       Requested transition to PLL_ON state.
 *             - RX_AACK_ON   Requested transition to RX_AACK_ON state.
 *             - TX_ARET_ON   Requested transition to TX_ARET_ON state.
 *
 *  \retval    RADIO_SUCCESS          Requested state transition completed
 *                                  successfully.
 *  \retval    RADIO_INVALID_ARGUMENT Supplied function parameter out of bounds.
 *  \retval    RADIO_WRONG_STATE      Illegal state to do transition from.
 *  \retval    RADIO_BUSY_STATE       The radio transceiver is busy.
 *  \retval    RADIO_TIMED_OUT        The state transition could not be completed
 *                                  within resonable time.
 */
radio_status_t
rf231_set_trx_state(uint8_t new_state)
{
    uint8_t original_state;
  radio_status_t set_state_status;
	
    /* Check function paramter and current state of the radio
     *  transceiver.*/
    if (!((new_state == TRX_OFF)    ||
          (new_state == RX_ON)      ||
          (new_state == PLL_ON)     ||
          (new_state == RX_AACK_ON) ||
          (new_state == TX_ARET_ON))){
        return RADIO_INVALID_ARGUMENT;
    }
	if (hal_get_slptr()) {
        return RADIO_WRONG_STATE;
    }

    /* Wait for radio to finish previous operation */
    rf231_wait_idle();
 
    original_state = rf231_get_trx_state();
    if (new_state == original_state){
        return RADIO_SUCCESS;
    }

    /* At this point it is clear that the requested new_state is:
    * TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON. 
    *
    * The radio transceiver can be in one of the following states:
    * TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON, TX_ARET_ON. */
    if(new_state == TRX_OFF){
        rf231_reset_state_machine(); /* Go to TRX_OFF from any state. */
    } else {
        /* It is not allowed to go from RX_AACK_ON or TX_AACK_ON and
	 * directly to TX_AACK_ON or RX_AACK_ON respectively. Need to
	 * go via RX_ON or PLL_ON. */
        if ((new_state == TX_ARET_ON) &&
            (original_state == RX_AACK_ON)){
            /* First do intermediate state transition to PLL_ON, then
             * to TX_ARET_ON.  The final state transition to
             * TX_ARET_ON is handled after the if-else if. */
            hal_subregister_write(SR_TRX_CMD, PLL_ON);
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        } else if ((new_state == RX_AACK_ON) &&
                 (original_state == TX_ARET_ON)){
            /* First do intermediate state transition to RX_ON, then
             * to RX_AACK_ON.  The final state transition to
             * RX_AACK_ON is handled after the if-else if. */
            hal_subregister_write(SR_TRX_CMD, RX_ON);
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }

        /* Any other state transition can be done directly. */
        hal_subregister_write(SR_TRX_CMD, new_state);

        /* When the PLL is active most states can be reached in
         * 1us. However, from TRX_OFF the PLL needs time to
         * activate. */
        if (original_state == TRX_OFF){
            delay_us(TIME_TRX_OFF_TO_PLL_ACTIVE);
        } else {
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
    } /*  end: if(new_state == TRX_OFF) ... */

    /* Verify state transition. */
    set_state_status = RADIO_TIMED_OUT;

    if (rf231_get_trx_state() == new_state) {
        set_state_status = RADIO_SUCCESS;
    }

    return set_state_status;
}

/*---------------------------------------------------------------------------*/
bool
rf231_is_ready_to_sendy() {
	switch(rf231_get_trx_state()) {
		case BUSY_TX:
		case BUSY_TX_ARET:
			return false;
	}
	
	return true;
}

/*---------------------------------------------------------------------------*/
static void
flushrx(void)
{
  rxframe[rxframe_head].length=0;
}


static int create_packet(void)
{
  /* create structure to store result. */
  frame_create_params_t params;
  frame_result_t result;

  /* Build the FCF. */
#ifdef SLOTTED_KOORDINATOR
  params.fcf.frameType = BEACON_FRAME_TYPE;
  params.fcf.panIdCompression = false;
  params.fcf.destAddrMode = 0;
  params.dest_pid = 0;
  params.payload_len = BEACON_PAYLOAD_LENGTH;
#else
  params.fcf.frameType = RESPONSE_FRAME_TYPE;
  params.fcf.panIdCompression = true;
  params.fcf.destAddrMode = SHORTADDRMODE;
  params.dest_pid = TDMA_PAN_ID;
  params.dest_addr.addr16 = 0x2e1b;      /* adress of the
					 coordinator. better to read
					 from the received beacons */
  params.payload_len = MAX_RESPONSE_PAYLOAD;
#endif

/* parameter that are the same for client and coord */
  params.fcf.securityEnabled = false;
  params.fcf.framePending = false;
  params.fcf.ackRequired = false;
  params.fcf.frameVersion = TDMA_FRAME_VERSION;
  /* Increment and set the data sequence number. */
  params.seq = sqn++;
  /* Complete the addressing fields. */
  params.fcf.srcAddrMode = SHORTADDRMODE;
  params.src_pid = TDMA_PAN_ID;
  params.src_addr.addr16 = nodeAddress;

  /* Copy the payload data. */
params.payload =  dummyPayload;

#ifdef SLOTTED_KOORDINATOR
params.bhdr.cycleTime = 1000;
params.bhdr.maxClients = MAX_CLIENTS;
#endif

/* HACK:
 * FCF field ist not set correctly. set it hardcoded.
 *
 * TODO: correct FCF settings
 */
#ifdef SLOTTED_KOORDINATOR
 params.fcf.word_val = 0xa006;
#else
 params.fcf.word_val = 0xa226;
#endif

 result.frame = buffer;

  /* Create transmission frame. */
  frame_tx_create(&params, &result);

  hal_frame_write(result.frame, result.length);

}

/*---------------------------------------------------------------------------*/
static void 
ringbuffer_add(ringBuffer *buffer, uint32_t value)
{
  buffer->Buff[buffer->PutPos] = value;
  buffer->PutPos = ((buffer->PutPos + 1) & PERIOD_BUFFER_MASK);
  ++(buffer->PutPos);
  ++(buffer->Count);
  if(buffer->Count > PERIOD_BUFFER_LENGTH) {
    buffer->Count=PERIOD_BUFFER_LENGTH;
  }
}

void
rf231_upload_packet(unsigned short payload_len)
{
    hal_frame_write(buffer, payload_len);
}

/*---------------------------------------------------------------------------*/
static uint32_t
ringbuffer_get_last(ringBuffer *buffer)
{
  return buffer->Buff[buffer->PutPos];
}

/*---------------------------------------------------------------------------*/
static uint32_t
ringbuffer_clear(ringBuffer *buffer)
{
  buffer->Count = 0;
  buffer->PutPos = 0;
}



/*---------------------------------------------------------------------------*/
static int
rf231_prepare(const void *payload, unsigned short payload_len)
{
  /* leave as empty function.  
   *
   * preparations are made by the slotted process.  
   *
   * TODO Implement a meachanism to store the packets taht shall be
   * transmitted in the packet queue. At the Moment we are sending
   * empty packages only for testing */

  return 1;
}

/*---------------------------------------------------------------------------*/
static int 
rf231_transmit(unsigned short payload_len)
{

  /* leav as empty function.  transmission is timed by the slotted process */


  return 1; 
}

/*---------------------------------------------------------------------------*/
static int
rf231_send(const void *payload, unsigned short payload_len)
{
  /* leave as empty function.  
   *
   * sending is made by the slotted process.
   *
   * TODO Implement a meachanism to store the packets taht shall be
   * transmitted in the packet queue. At the Moment we are sending
   * empty packages only for testing */

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
  state = RF231_STATE_IDLE;
  /** start the timer */
  /** bring transceiver into RX_ON */
  return 1;
}

void rf231_slotted_IC_irqh(uint32_t capture)
{
  /* write IC value into the IC buffer and generate a IC event to process the new value*/
#ifndef SLOTTED_KOORDINATOR
  /* store the new value into our ringbuffer */
  ringbuffer_add(&PeriodBuffer, capture);
  lastBeaconTime = capture;
  //  hal_update_oc(capture + 1000);
  //process_post(&rf231_slotted_process, HANDLE_PACKET_EVENT, NULL);
#endif /* SLOTTED_KOORDINATOR */
}

static void calculate_period()
{
  int i;
  uint32_t AvgPeriod;

  /* substract last stored value with the first sotred and divide by number of periods */
  Period = (PeriodBuffer.Buff[(PeriodBuffer.PutPos) - 1] 
	    - PeriodBuffer.Buff[PeriodBuffer.PutPos])  >> NUM_PERIODS_BASE;

  /* /\* check if the last measured period was valid. If it was *\/ */
  /* /\* if the number of stored Periods is smaller than the max number of periods */
  /*  * only add it to the buffer. */
  /*  *\/ */
  /* if (PeriodCount < NUM_PERIODS) */
  /*   { */
  /*     ++PeriodCount; */
  /*     ++PeriodPos; */
  /*   } else { */
  /*   AvgPeriod = 0; */
  /*   for (i=0; i < NUM_PERIODS; ++i) */
  /*     { */
  /* 	AvgPeriod += PeriodBuffer[i]; */
  /*     } */
  /*   AvgPeriod = AvgPeriod >> NUM_PERIODS_BASE; /\** shift right to simulate division *\/ */
  /*   Period = AvgPeriod; */
  /*   ++PeriodPos; */
  /* } */
  /* if(PeriodPos >= NUM_PERIODS) */
  /*   { */
  /*     /\* we reached the end of our buffer -> wrap around *\/ */
  /*     PeriodPos = 0; */
  /*   } */
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
      //   calculate_period();
      /* set a new oc event */
//      hal_update_oc(PeriodBuffer.Buff[(PeriodBuffer.PutPos - 1) & PERIOD_BUFFER_MASK] + 1000);
    }
    if(ev==HANDLE_PACKET_EVENT){
      hal_frame_read(rxframe);
      hal_update_TX_Mode_Timer(lastBeaconTime + slotTime - 50);
    }

    if ((ev==sensors_event) && (data == &button_sensor)){
      /* print the curretn information to usart */
      printf("The Period is : %i\n\r", Period);
    }
  }

  PROCESS_END();
}
