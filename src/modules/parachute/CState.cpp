/*
 * CState.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#include "CState.hpp"

CState CState::m_sStateDummy(nullptr
#if MAVLINK_VERBOSE > 0
		, 0
#endif
		);

CState::CState(
		CStateMachine* pParent
#if MAVLINK_VERBOSE > 0
		, int iMavLinkFd
#endif
		)
	: m_pModule(pParent)
#if MAVLINK_VERBOSE > 0
	, m_iMavLinkFd(iMavLinkFd)
#endif
{
}

CState::~CState()
{
}

void
CState::init()
{
}

void
CState::run()
{
}
