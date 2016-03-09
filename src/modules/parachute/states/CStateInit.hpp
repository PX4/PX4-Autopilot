/*
 * CStateInit.hpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#ifndef SRC_MODULES_PARACHUTE_CSTATEINIT_HPP_
#define SRC_MODULES_PARACHUTE_CSTATEINIT_HPP_

#include "../CState.hpp"
#include "../CStateMachine.hpp"

#include "../CRingBuffer.hpp"


#define ALTITUDE_HIGH_RESOLUTION_BUFFER_SIZE 30
#define ALTITUDE_LOW_RESOLUTION_BUFFER_SIZE 3

// delay in microseconds (µs)
#define ALTITUDE_HIGH_RESOLUTION_BUFFER_DELAY 1000000
#define ALTITUDE_LOW_RESOLUTION_BUFFER_DELAY 30000000

#if ALTITUDE_HIGH_RESOLUTION_BUFFER_DELAY > ALTITUDE_LOW_RESOLUTION_BUFFER_DELAY
#error "high resolution buffer delay time must be smaller then low resoluton buffer delay time!"
#endif

#if PARACHUTE_TASK_DELAY > ALTITUDE_LOW_RESOLUTION_BUFFER_DELAY
#error "low resolution buffer delay time must be smaller then task delay time!"
#endif



class CStateInit : public CState
{
public:
	CStateInit(CStateMachine* pParent
#if MAVLINK_VERBOSE > 0
		, int iMavLinkFd
#endif
		);
	virtual ~CStateInit();

	void init() override;
	void run() override;

	static const float m_scfLimVal;
	static const float m_scfLimValOffset;

private:
	CRingBuffer<float, ALTITUDE_HIGH_RESOLUTION_BUFFER_SIZE> 	m_AltitudeHighResolutionBuffer;
	CRingBuffer<float, ALTITUDE_LOW_RESOLUTION_BUFFER_SIZE> 	m_AltitudeLowResolutionBuffer;

	hrt_abstime m_tLastTimeHighResolution;
	hrt_abstime m_tLastTimeLowResolution;

	bool m_bLimValExeeded;
};


#endif /* SRC_MODULES_PARACHUTE_CSTATEINIT_HPP_ */
