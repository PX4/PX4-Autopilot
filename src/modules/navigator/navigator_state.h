/*
 * navigator_state.h
 *
 *  Created on: 27.01.2014
 *      Author: ton
 */

#ifndef NAVIGATOR_STATE_H_
#define NAVIGATOR_STATE_H_

typedef enum {
	NAV_STATE_NONE = 0,
	NAV_STATE_READY,
	NAV_STATE_LOITER,
	NAV_STATE_MISSION,
	NAV_STATE_RTL,
	NAV_STATE_LAND,
	NAV_STATE_MAX
} nav_state_t;

#endif /* NAVIGATOR_STATE_H_ */
