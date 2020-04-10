/*
 * uorb_converter.h
 *
 *  Created on: Apr 10, 2020
 *      Author: hovergames
 */

#ifndef SRC_DRIVERS_UAVCANNODE_V1_UORB_CONVERTER_H_
#define SRC_DRIVERS_UAVCANNODE_V1_UORB_CONVERTER_H_


/* uORB */
#include <px4_platform_common/posix.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

/* canard */
#include <canard.h>
#include <canard_dsdl.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Temporary development UAVCAN topic service ID to publish/subscribe from */
#define PORT_ID                                  4421

/* Temporary topic to test UAVCAN */

#define TOPIC_SIZE                               sizeof(struct vehicle_gps_position_s)

void uorbConverterInit(CanardInstance *ins);

void uorbProcessSub(int timeout_msec);

void uorbProcessPub(CanardTransfer *pub_msg);


#endif /* SRC_DRIVERS_UAVCANNODE_V1_UORB_CONVERTER_H_ */
