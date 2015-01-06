#ifndef __TOPIC_VEHICLE_LANDED_H__
#define __TOPIC_VEHICLE_LANDED_H__

#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct vehicle_land_detected_s {
	uint64_t timestamp;	   /**< timestamp of the setpoint */
	bool landed;           /**< true if vehicle is currently landed on the ground*/
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_land_detected);

#endif //__TOPIC_VEHICLE_LANDED_H__
