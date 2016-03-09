/*
 * CState.hpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#ifndef SRC_MODULES_PARACHUTE_CSTATE_HPP_
#define SRC_MODULES_PARACHUTE_CSTATE_HPP_

#if MAVLINK_VERBOSE > 0
#include <stdio.h>
#include <mavlink/mavlink_log.h>
#endif

class CStateMachine;


class CState
{
public:
	CState(CStateMachine* pParent
#if MAVLINK_VERBOSE > 0
			, int m_iMavLinkFd
#endif
			);

	virtual ~CState();

	virtual void init();
	virtual void run();

	static CState m_sStateDummy;

protected:
	CStateMachine* m_pModule;

#if MAVLINK_VERBOSE > 0
	int				m_iMavLinkFd;
#endif

};



#endif /* SRC_MODULES_PARACHUTE_CSTATE_HPP_ */
