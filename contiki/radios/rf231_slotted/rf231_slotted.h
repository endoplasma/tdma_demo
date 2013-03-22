#ifndef RF231_SLOTTED_H
#define RF231_SLOTTED_H

/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include "rf231_slotted_hal.h"

/*============================ EXTERN DEFINITIONS ============================*/
extern const struct radio_driver rf231_slotted_driver;

#define NUM_PERIODS           8            /** The number of Periods used to calculate the average. User only powers of 2 */
#define NUM_PERIODS_BASE      3            /** log2(NUM_PERIOD) used for fast division */
ädefine PERIOD_BUFFER_LENGTH  8            /** length of the period buffer power of 2 */
#define PERIOD_BUFFER_MASK    NUM_PERIOD-1 /** Mask for fast Buffer boundary wrapping */


/*============================ PROCESS EVENTS ================================*/
#define INPUT_CAPTURE_EVENT   20


#endif /* RF231_SLOTTED_H */
