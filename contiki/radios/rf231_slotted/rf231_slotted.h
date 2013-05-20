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
    RADIO_SUCCESS = RADIO_STATUS_START_VALUE,  /**< The requested service was performed successfully. */
    RADIO_UNSUPPORTED_DEVICE,         /**< The connected device is not an Atmel AT86RF230. */
    RADIO_INVALID_ARGUMENT,           /**< One or more of the supplied function arguments are invalid. */
    RADIO_TIMED_OUT,                  /**< The requested service timed out. */
    RADIO_WRONG_STATE,                /**< The end-user tried to do an invalid state transition. */
    RADIO_BUSY_STATE,                 /**< The radio transceiver is busy receiving or transmitting. */
    RADIO_STATE_TRANSITION_FAILED,    /**< The requested state transition could not be completed. */
    RADIO_CCA_IDLE,                   /**< Channel is clear, available to transmit a new frame. */
    RADIO_CCA_BUSY,                   /**< Channel busy. */
    RADIO_TRX_BUSY,                   /**< Transceiver is busy receiving or transmitting data. */
    RADIO_BAT_LOW,                    /**< Measured battery voltage is lower than voltage threshold. */
    RADIO_BAT_OK,                     /**< Measured battery voltage is above the voltage threshold. */
    RADIO_CRC_FAILED,                 /**< The CRC failed for the actual frame. */
    RADIO_CHANNEL_ACCESS_FAILURE,     /**< The channel access failed during the auto mode. */
    RADIO_NO_ACK,                     /**< No acknowledge frame was received. */
}radio_status_t;

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

/*============================ PROCESS EVENTS ================================*/
#define INPUT_CAPTURE_EVENT           20
#define HANDLE_PACKET_EVENT           21
#define BEACON_MISSED_EVENT           22
#define TX_MODE_TIMER_EVENT           23

/*============================ TDMA PARAMETER ================================*/
#define MAX_CLIENTS                   3
#define PAYLOAD_PER_CLIENT            4
#define MAX_RESPONSE_PAYLOAD          4

#define BEACON_HEADER_LENGTH          12
#define RESPONSE_HEADER_LENGTH        12

#define BEACON_PAYLOAD_LENGTH          (MAX_CLIENTS * PAYLOAD_PER_CLIENT)
#define RESPONSE_PAYLOAD_LENGTH        (MAX_RESPONSE_PAYLOAD)


            
#ifdef SLOTTED_KOORDINATOR
#define RF230_MAX_TX_FRAME_LENGTH     (BEACON_HEADER_LENGTH + (MAX_CLIENTS * PAYLOAD_PER_CLIENT) + 2)
#else 
#define RF230_MAX_TX_FRAME_LENGTH     (RESPONSE_HEADER_LENGTH + MAX_RESPONSE_PAYLOAD + 2)
#endif /* SLOTTED_KOORDINATOR */


#define TDMA_PAN_ID                  (0xff12)
#define TDMA_PAN_ID_0                  (0x12)
#define TDMA_PAN_ID_1                  (0xff)



#endif /* RF231_SLOTTED_H */
