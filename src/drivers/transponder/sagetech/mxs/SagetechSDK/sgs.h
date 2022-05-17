/**
 * @copyright Copyright (c) 2020 Sagetech, Inc. All rights reserved.
 *
 * @file sgs.h
 * @author jim.billmeyer
 *
 * @date Mar 30, 2021
 *
 * The TCAS/Mode S support.
 */

#ifndef SGS_H_
#define SGS_H_

#include <stdint.h>
#include <stdbool.h>

/// XPNDR Surveillance Message Types
#define SG_MSG_TYPE_SURV_DF0       0xA6

typedef enum
{
   riNone      = 0,
   riCASTA     = 2,
   riCASRA     = 3,
   riCASRAHorz = 4
} sgs_ri_t;

typedef enum
{
   survNormal,
   survReduced,
   survHybrid
} sgs_surv_t;

typedef struct
{
   uint32_t   addr;        /// The Mode S address.
   float      slantRange;  /// The slant range in nmiles.
   float      bearing;     /// The azimuth/bearing between the ownship and the target in degrees.
   float      baroAlt;     /// The barometric altitude in feet.
   uint8_t    quant;       /// The altitude quantization. Typically 25 or 100 feet.
   sgs_ri_t   ri;          /// The RI field indicates whether there is no TCAS, TCAS TA only, TCAS with RA..
   sgs_surv_t surv;        /// The surveillance mode used to elicit a reply.
   float      toa;         /// The time of applicability of the message.
} sgs_df0_t;

/**
 * Decode the DF0 message from the data stream.
 *
 * @param data The data stream.
 * @param df0  The returned parsed DF0 message.
 *
 * @return true if successful or false on failure.
 */
bool sgsDecodeDF0(uint8_t *data, sgs_df0_t *df0);

/**
 * Decode the clock synchronization message from the data stream.
 *
 * @param data     The data stream.
 * @param ticktime The ticktime of the MX transponder.
 *
 * @return true if successful or false on failure.
 */
bool sgsDecodeClock(uint8_t *data, uint64_t *ticktime);

#endif /* SGS_H_ */
