/*
 * CStateLanding.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#include "CStateLanding.hpp"


//-------------------------------------------------
// CTOR
//-------------------------------------------------
CStateLanding::CStateLanding(CStateMachine* pParent
#if MAVLINK_VERBOSE > 0
	, int iMavLinkFd
#endif
	)
	: CState(pParent
#if MAVLINK_VERBOSE > 0
			, iMavLinkFd
#endif
			)
{

}


//-------------------------------------------------
// DTOR
//-------------------------------------------------
CStateLanding::~CStateLanding()
{

}


//-------------------------------------------------
// init
//-------------------------------------------------
void
CStateLanding::init()
{
#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: LANDING state...\n");
#endif

	m_pModule->setServo(6, -1.0);
}


//-------------------------------------------------
// run
//-------------------------------------------------
void
CStateLanding::run()
{
	// TODO: replace following code with a delay(inf or 1s)
	if(m_pModule->m_sPosition.alt > 5000 && m_pModule->m_sPosition.alt < 5100)
	{
		m_pModule->changeState(CStateMachine::INIT);
	}
}
