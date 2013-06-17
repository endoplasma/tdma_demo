/*
 * Copyright (c) 2012, TU Dortmund University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki OS
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \file    Headerfle for the rf231 specific slotted phy driver
 * \author  Philipp Spliethoff <philipp.spliethoff@tu-dortmund.de>
 */
/*---------------------------------------------------------------------------*/
#ifndef RF231_SLOTTED_H
#define RF231_SLOTTED_H

/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include "rf231_slotted_hal.h"

#define DEBUGFLOW(x)
#define PRINTF(...) printf(__VA_ARGS__)

/*============================ EXTERN DEFINITIONS ============================*/
extern const struct radio_driver rf231_slotted_driver;

/*============================ MACROS ========================================*/
#define SUPPORTED_PART_NUMBER                   ( 2 )
#define RF230_REVA                              ( 1 )
#define RF230_REVB                              ( 2 )
#define SUPPORTED_MANUFACTURER_ID               ( 31 )
#define RF230_SUPPORTED_INTERRUPT_MASK          ( MASK_TRX_END | MASK_RX_START )


#define NUM_PERIODS           8               /**< The number of Periods used to
						   calculate the average. Use 
						   only powers of 2 */
#define NUM_PERIODS_BASE      3               /**< log2(NUM_PERIOD) used for fast 
						   division */
#define PERIOD_BUFFER_LENGTH  8               /**< length of the period buffer 
						   power of 2 */
#define PERIOD_BUFFER_MASK    NUM_PERIODS-1   /**< Mask for fast Buffer boundary 
						   wrapping */

/*============================ STATEMACHINE STATES ===========================*/
#define RF231_STATE_UNINIT           0 /** Uninitialised Statemachine */
#define RF231_STATE_INACTIVE         1 /** Machine deactivated */
#define RF231_STATE_IDLE             2 /** Koordinator idle */
#define RF231_STATE_HANDLE_PACKET    3 /** Packet received, busy handling it */
#define RF231_STATE_ACTIVE_PLL       4 /** PLL for sending activated */
#define RF231_STATE_SEND             5 /** Sending a packet */

#define RF231_STATE_SYNCHED          6 /** Client synchronised */
#define RF231_STATE_UNSYNCHED        7 /** Client unsynchronised */
#define RF231_STATE_BEACON_MISSED    8 /** Client detected a missed beacon */

/*============================ PROCESS EVENTS ================================*/
#define INPUT_CAPTURE_EVENT           20
#define HANDLE_PACKET_EVENT           21
#define BEACON_MISSED_EVENT           22
#define TX_MODE_TIMER_EVENT           23
#define BEACON_RECEIVED_EVENT         24
#define FRAME_SEND_EVENT              25

/*============================ TDMA PARAMETER ================================*/
/* change those parameters to modifiy the TDMA Protokoll flow */
#define MAX_CLIENTS                   3
#define PAYLOAD_PER_CLIENT            4
#define MAX_RESPONSE_PAYLOAD          4

#define BEACON_HEADER_LENGTH          12
#define RESPONSE_HEADER_LENGTH        12

#define BEACON_PAYLOAD_LENGTH          (MAX_CLIENTS * PAYLOAD_PER_CLIENT)
#define RESPONSE_PAYLOAD_LENGTH        (MAX_RESPONSE_PAYLOAD)

#define BEACON_LENGTH                 (BEACON_HEADER_LENGTH + BEACON_PAYLOAD_LENGTH)
#define RESPONSE_LENGTH               (RESPONSE_HEADER_LENGTH + RESPONSE_PAYLOAD_LENGTH)

            
#ifdef SLOTTED_KOORDINATOR
#define RF231_MAX_TX_FRAME_LENGTH     (BEACON_HEADER_LENGTH + (MAX_CLIENTS * PAYLOAD_PER_CLIENT) + 2)
#else 
#define RF231_MAX_TX_FRAME_LENGTH     (RESPONSE_HEADER_LENGTH + MAX_RESPONSE_PAYLOAD + 2)
#endif /* SLOTTED_KOORDINATOR */


#define TDMA_PAN_ID                  (0xff12)
#define TDMA_PAN_ID_0                  (0x12)
#define TDMA_PAN_ID_1                  (0xff)



/*============================ TYPE DEFS =====================================*/
#define RADIO_STATUS_START_VALUE                  ( 0x40 )

/** \brief  This enumeration defines the possible return values for the TAT API
 *          functions.
 *
 *          These values are defined so that they should not collide with the
 *          return/status codes defined in the IEEE 802.15.4 standard.
 *
 */
typedef enum{
    RADIO_SUCCESS = RADIO_STATUS_START_VALUE,  /**< The requested service was
						  performed successfully. */
    RADIO_UNSUPPORTED_DEVICE,                  /**< The connected device is not
						  an Atmel AT86RF230. */
    RADIO_INVALID_ARGUMENT,                    /**< One or more of the supplied
						  function arguments are
						  invalid. */
    RADIO_TIMED_OUT,                           /**< The requested service timed
						  out. */
    RADIO_WRONG_STATE,                         /**< The end-user tried to do an
						  invalid state transition. */
    RADIO_BUSY_STATE,                          /**< The radio transceiver is
						  busy receiving or
						  transmitting. */
    RADIO_STATE_TRANSITION_FAILED,             /**< The requested state
						  transition could not be
						  completed. */
    RADIO_CCA_IDLE,                            /**< Channel is clear, available
						  to transmit a new frame. */
    RADIO_CCA_BUSY,                            /**< Channel busy. */
    RADIO_TRX_BUSY,                            /**< Transceiver is busy
						  receiving or transmitting
						  data. */
    RADIO_BAT_LOW,                             /**< Measured battery voltage is
						  lower than voltage
						  threshold. */
    RADIO_BAT_OK,                              /**< Measured battery voltage is
						  above the voltage
						  threshold. */
    RADIO_CRC_FAILED,                          /**< The CRC failed for the
						  actual frame. */
    RADIO_CHANNEL_ACCESS_FAILURE,              /**< The channel access failed
						  during the auto mode. */
    RADIO_NO_ACK,                              /**< No acknowledge frame was
						  received. */
}radio_status_t;


/**
 * A structure to stores all information used to configure the state
 * of the TDMA Protocol.
 *
 */
 typedef struct{
  uint32_t numClients;             /**< the nuber max number of connected
				        clients */
  uint32_t Period;                 /**< The actual calulated Period */
  uint32_t clientSlotLength;       /**< Slot length for a Cient in us (192 +
				        m*32 with m = length of the packet) */
  uint32_t guardInterval;          /**< Guard INterval length in us */
  uint32_t clientProcessing;       /**< The client Processing Time */
  uint32_t slotOffsett;            /**< The actual slot offset to the received
				        Beacon Fram */
  uint32_t beaconCount;            /**< number of consecutive received
				        beacons */ 
}proto_conf_t;

/**
 * A struct to store the last measuered period lengths to calculate
 * the correct send time
 *
 */
typedef struct ringBuffer{
  uint32_t Buff[NUM_PERIODS];      /**< Array to store the last measured
				      Periods */
  uint8_t PutPos;                  /**< Position to store the next
				      measurement */
  uint8_t Count;                   /**< The number of stored Periods */
}ring_buffer_t;


#endif /* RF231_SLOTTED_H */
