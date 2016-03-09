/*
 * CStateFlight.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#include "CStateFlight.hpp"


const float CStateFlight::m_scfLimVal	= 1000.0f;



//-------------------------------------------------
// CTOR
//-------------------------------------------------
CStateFlight::CStateFlight(CStateMachine* pParent
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
CStateFlight::~CStateFlight()
{

}


//-------------------------------------------------
// init
//-------------------------------------------------
void
CStateFlight::init()
{

#if MAVLINK_VERBOSE > 0
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: FLIGHT state...\n");
#endif

	m_pModule->setActuatorState(CStateMachine::ARMED);

	m_pModule->setServo(6, 0.0);

	m_pModule->setAutoRTLMode();
}


//-------------------------------------------------
// run
//-------------------------------------------------
void
CStateFlight::run()
{
	if(true == m_pModule->isBatteryCritical())
	{
		m_pModule->changeState(CStateMachine::LANDING);
		return;
	}

	if(false == m_pModule->isPositionStable())
	{
		m_pModule->changeState(CStateMachine::LANDING);
		return;
	}

	if(true == m_pModule->isFailsafe())
	{
		m_pModule->changeState(CStateMachine::LANDING);
		return;
	}

	if(true == m_pModule->isManualMode())
	{
		/*
		 * when RC is connected, we can easily change to manual mode
		 * since testing if RC is available is not possible, we check
		 * whether we are in manual mode or not.
		*/
		return; // do not change state, when there is a connection to ground control
	}


	if(m_pModule->m_sPosition.alt < m_scfLimVal)
	{
		m_pModule->changeState(CStateMachine::LANDING);
		return;
	}
}
