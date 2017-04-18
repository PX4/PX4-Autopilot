/*
 * CStateInit.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#include "CStateInit.hpp"



const float CStateInit::m_scfLimVal 		= 1000.0;
const float CStateInit::m_scfLimValOffset 	= 100.0;


//-------------------------------------------------
// CTOR
//-------------------------------------------------
CStateInit::CStateInit(CStateMachine* pParent
#if MAVLINK_VERBOSE > 0
	, int iMavLinkFd
#endif
	)
	: CState(pParent
#if MAVLINK_VERBOSE > 0
			, iMavLinkFd
#endif
			)
	, m_tLastTimeHighResolution(0)
	, m_tLastTimeLowResolution(0)
	, m_bLimValExeeded(false)
{

}


//-------------------------------------------------
// DTOR
//-------------------------------------------------
CStateInit::~CStateInit()
{

}


//-------------------------------------------------
// init
//-------------------------------------------------
void
CStateInit::init()
{
#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: INIT state...\n");
#endif

	bool l_bResult;
	l_bResult = m_pModule->setAutoLoiterMode();


#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: setAutoLoiterMode(): %d\n", l_bResult);
#endif

	m_pModule->setServo(6, 1.0);

	l_bResult = m_pModule->setVehicleState(CStateMachine::ARMED);

#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: setVehicleState(ARMED): %d\n", l_bResult);
#endif

	l_bResult = m_pModule->setActuatorState(CStateMachine::DISARMED);

#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: setActuatorState(DISARMED): %d\n", l_bResult);
#endif
}


//-------------------------------------------------
// run
//-------------------------------------------------
void
CStateInit::run()
{
	float 	l_fCurrentAltitude 	= m_pModule->m_sPosition.alt;
	float	l_fAverageAltitude 	= 0.0;

	unsigned int	l_uiBufferLen	= 	0;
	hrt_abstime 	l_tCurrentTime	= 	0;


	// fill high resolution buffer
	l_tCurrentTime		= hrt_absolute_time();

	if(l_tCurrentTime - ALTITUDE_HIGH_RESOLUTION_BUFFER_DELAY > m_tLastTimeHighResolution)
	{
#if MAVLINK_VERBOSE > 2
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: HighResolutionBuffer [size: %d]", m_AltitudeHighResolutionBuffer.length());
#endif
		m_tLastTimeHighResolution = hrt_absolute_time();

		m_AltitudeHighResolutionBuffer.add(l_fCurrentAltitude);
	}


	// fill low resolution buffer
	l_tCurrentTime		= hrt_absolute_time();

	if(l_tCurrentTime - ALTITUDE_LOW_RESOLUTION_BUFFER_DELAY > m_tLastTimeLowResolution)
	{
		l_uiBufferLen = m_AltitudeHighResolutionBuffer.length();
		for(unsigned int i = 0; i < l_uiBufferLen; i++)
		{
			l_fAverageAltitude += m_AltitudeHighResolutionBuffer[i];
		}
		l_fAverageAltitude /= l_uiBufferLen;

		m_AltitudeLowResolutionBuffer.add(l_fAverageAltitude);

		m_tLastTimeLowResolution = hrt_absolute_time();

#if MAVLINK_VERBOSE > 2
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: LowResolutionBuffer [size: %d]", m_AltitudeLowResolutionBuffer.length());
#endif
	}

/* TODO: remove comments, to activate following code!
 * code has been commented because it may lead to an unwanted behavior while testing
	if(true == m_pModule->isBatteryCritical())
	{
#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: Battery critical -> landing");
#endif
		m_pModule->changeState(CStateMachine::LANDING);
		return;
	}

	if(false == m_pModule->isPositionStable())
	{
#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: GPS failure -> landing");
#endif
		m_pModule->changeState(CStateMachine::LANDING);
		return;
	}

	if(true == m_pModule->isFailsafe())
	{
#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: Failsafe mode -> landing");
#endif
		m_pModule->changeState(CStateMachine::LANDING);
		return;
	}
*/

	if(m_AltitudeLowResolutionBuffer.length() > 2)
	{
		l_fAverageAltitude = m_AltitudeLowResolutionBuffer[2];

		if(l_fAverageAltitude > (m_scfLimVal + m_scfLimValOffset))
		{
#if MAVLINK_VERBOSE > 2
			mavlink_and_console_log_info(m_iMavLinkFd, "PARA: LimVal exceeded!");
#endif
			m_bLimValExeeded = true;
		}
		else
		{
			if(true == m_bLimValExeeded)
			{
				if( l_fAverageAltitude < m_scfLimVal)
				{
					m_pModule->changeState(CStateMachine::LANDING);
				}
				else
				{
					if(l_fAverageAltitude < m_AltitudeLowResolutionBuffer[1])
					{
						if(l_fAverageAltitude < (m_AltitudeLowResolutionBuffer[0] - m_scfLimValOffset))
						{
							m_pModule->changeState(CStateMachine::FLIGHT);
						}
					}
				}
			}
		}
	}
}
