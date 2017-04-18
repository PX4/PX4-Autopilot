/*
 * CStateMachine.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: friedrich
 */

#include "CStateMachine.hpp"


CStateMachine::CStateMachine()
	: m_pCurrentState(&CState::m_sStateDummy)
	, m_pStateInit(nullptr)
	, m_pStateFlight(nullptr)
	, m_pStateLanding(nullptr)
	, m_eState(LANDING)
	, m_bStateHasChanged(true)
	, m_actuatorsPub(-1)
	, m_parachutePub(-1)
	, m_iPositionHanlde(-1)

, m_iVehicleStatusHandle(-1)
	, m_tPositionValidTime(0)
{
}


CStateMachine::~CStateMachine()
{
#if MAVLINK_VERBOSE > 0
	close(m_iMavLinkFd);
#endif

	delete m_pStateInit;
	delete m_pStateFlight;
	delete m_pStateLanding;

	close(m_actuatorsPub);
	close(m_parachutePub);
	close(m_iPositionHanlde);
}


int
CStateMachine::init()
{
	//bool l_bResult = false;

#if MAVLINK_VERBOSE > 0
	m_iMavLinkFd = open(MAVLINK_LOG_DEVICE, 0);
#endif
#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: mavlink opened\n");
#endif

	/*
	 * do subscriptions
	 */
	subscribe();
#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: subscribe [done]\n");
#endif

	/*
	 * do advertisements
	 */
	advertise();
#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: advertise [done]\n");
#endif

#ifdef USE_RC_PARAM
	param_init();

#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: param_init [done]\n");
#endif

#endif // USE_RC_PARAM

	// init states
	m_pStateInit = new CStateInit(this
#if MAVLINK_VERBOSE > 0
			, m_iMavLinkFd
#endif
			);

	m_pStateFlight = new CStateFlight(this
#if MAVLINK_VERBOSE > 0
			, m_iMavLinkFd
#endif
			);

	m_pStateLanding = new CStateLanding(this
#if MAVLINK_VERBOSE > 0
			, m_iMavLinkFd
#endif
			);

	if(!m_pStateInit || !m_pStateFlight || !m_pStateLanding)
	{
#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_critical(m_iMavLinkFd, "[E] PARA: not enough memory!\n");
#endif
		return -1;
	}
	else
	{
#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: init states [done]\n");
#endif
	}

	memset(&m_sVehicleStatus, 0 , sizeof(m_sVehicleStatus));
	orb_copy(ORB_ID(vehicle_status), m_iVehicleStatusHandle, &m_sVehicleStatus);

	#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: get vehicle status [done]\n");
#endif

	waitForGPSLock();
#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: GPS locked!\n");
#endif
	changeState(INIT);

	return 0;
}


// changeState()
int
CStateMachine::changeState(EState_t state)
{
#if MAVLINK_VERBOSE > 1
	mavlink_and_console_log_critical(m_iMavLinkFd, "PARA: changeState, old: %d, new:%d\n", m_eState, state);
#endif

	if( state == m_eState )
	{
		return 0;
	}

	m_eState = state;
	m_bStateHasChanged = true;

	switch(state)
	{
	case INIT: 		m_pCurrentState = (CStateInit*)		m_pStateInit; 	break;
	case FLIGHT: 	m_pCurrentState = (CStateFlight*)	m_pStateFlight; 	break;
	case LANDING: 	m_pCurrentState = (CStateLanding*)	m_pStateLanding; 	break;
	default:
#if MAVLINK_VERBOSE > 0
	mavlink_and_console_log_critical(m_iMavLinkFd, "[E] PARA: invalid state!\n");
#endif
		m_pCurrentState = &CState::m_sStateDummy;
		break;
	}

	publishState();

	return 0;
}

/*
 * isPositionStable
 * @return 	true: when latest filtered GPS data has been refreshed
 * 				within the last <POSITION_STABLE_TIMEOUT> ms.
 */
bool
CStateMachine::isPositionStable()
{
	hrt_abstime 	l_tCurrentTime = hrt_absolute_time();

	if(m_sPosition.timestamp + POSITION_STABLE_TIMEOUT > l_tCurrentTime)
	{
		return true;
	}

	return false;
}

/*
bool
CStateMachine::isGPSLocked()
{
	bool l_bReturn = false;
	struct vehicle_status_s status;

	memset(&status, 0 , sizeof(status));

	int arm_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
	orb_copy(ORB_ID(vehicle_status), arm_sub_fd, &status);

	if(status.condition_global_position_valid)
	{
		m_tGPSLockedTime = hrt_absolute_time();
	}

#ifdef USE_RC_PARAM
	if(0 != m_param.GPSLocked)
	{
		l_bReturn = true;
	}
#endif


#if MAVLINK_VERBOSE > 2
	mavlink_and_console_log_info(m_iMavLinkFd, "PARA: GPS V: %d, GPS F: %d, local V:%d\n", status.condition_global_position_valid, status.gps_failure, status.condition_local_position_valid);
#endif

	return ((status.condition_global_position_valid) && (status.gps_failure == false)) || l_bReturn;
}
*/

void
CStateMachine::waitForGPSLock()
{
	bool l_bUpdated = false;

	do
	{
		orb_check(m_iVehicleStatusHandle, &l_bUpdated);

#ifdef USE_RC_PARAM
		param_update();

		if(0 != m_param.GPSLocked)
		{
			return;
		}
#endif

		if(true == l_bUpdated)
		{
			orb_copy(ORB_ID(vehicle_status), m_iVehicleStatusHandle, &m_sVehicleStatus);

			if(false == m_sVehicleStatus.condition_global_position_valid)
			{
				l_bUpdated = false;
			}
			else
			{
				return;
			}
		}
#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: GPS not locked!\n");
#endif

		usleep((useconds_t)500000);
	}
	while(!l_bUpdated);
}


void
CStateMachine::run()
{
#if MAVLINK_VERBOSE > 3
	mavlink_and_console_log_info(m_iMavLinkFd, "vehicle state: %d\n", getVehicleState());
	mavlink_and_console_log_info(m_iMavLinkFd, "parachute task is running in state: %d\n", m_eState);
#endif

#ifdef USE_RC_PARAM
	param_update();
#endif

	if(true == m_bStateHasChanged)
	{
		m_bStateHasChanged = false;

		// initiaize state
		m_pCurrentState->init();
	}

	positionPoll(&m_sPosition);

#ifdef USE_RC_PARAM
	if(m_param.altitude >= 0)
	{
		m_sPosition.alt = m_param.altitude;
	}
#endif

#if MAVLINK_VERBOSE > 3
	mavlink_and_console_log_info(m_iMavLinkFd, "FS: %d, BC: %d, RC: %d", this->isFailsafe(), this->isBatteryCritical(), this->isRCLinked());
#endif
	m_pCurrentState->run();
}


/*
 * setActuatorState()
 * @param	EArmingState_t eState: new state [armed|disarmed]
 * @return	bool: changing state was successful (OK = 0 =false)
 */
bool
CStateMachine::setActuatorState(EArmedState_t eState)
{
	struct actuator_armed_s arm;
	memset(&arm, 0 , sizeof(arm));

	arm.timestamp 		= hrt_absolute_time();
	arm.ready_to_arm 	= (ARMED == eState);
	arm.armed 			= (ARMED == eState);

	orb_advert_t arm_pub_fd = orb_advertise(ORB_ID(actuator_armed), &arm);
	orb_publish(ORB_ID(actuator_armed), arm_pub_fd, &arm);

	int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

	if(eState == ARMED)
	{
		return !(arm.ready_to_arm && arm.armed);
	}
	else
	{
		return !(!arm.ready_to_arm && !arm.armed);
	}
}


CStateMachine::EArmedState_t
CStateMachine::getVehicleState()
{
	bool l_bUpdated = false;

	orb_check(m_iVehicleStatusHandle, &l_bUpdated);
	if(l_bUpdated)
	{
		orb_copy(ORB_ID(vehicle_status), m_iVehicleStatusHandle, &m_sVehicleStatus);
	}

	return (m_sVehicleStatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? ARMED : DISARMED;
}


bool
CStateMachine::isBatteryCritical()
{
	bool l_bUpdated = false;

	orb_check(m_iVehicleStatusHandle, &l_bUpdated);
	if(l_bUpdated)
	{
		orb_copy(ORB_ID(vehicle_status), m_iVehicleStatusHandle, &m_sVehicleStatus);
	}

	return (m_sVehicleStatus.condition_battery_voltage_valid) ?
			m_sVehicleStatus.battery_warning == vehicle_status_s::VEHICLE_BATTERY_WARNING_CRITICAL
			: false;
}


bool
CStateMachine::isFailsafe()
{
	bool l_bUpdated = false;

	orb_check(m_iVehicleStatusHandle, &l_bUpdated);
	if(l_bUpdated)
	{
		orb_copy(ORB_ID(vehicle_status), m_iVehicleStatusHandle, &m_sVehicleStatus);
	}

	return m_sVehicleStatus.failsafe;
}


bool
CStateMachine::isManualMode()
{
	bool l_bUpdated = false;

	orb_check(m_iVehicleStatusHandle, &l_bUpdated);
	if(l_bUpdated)
	{
		orb_copy(ORB_ID(vehicle_status), m_iVehicleStatusHandle, &m_sVehicleStatus);
	}

	return m_sVehicleStatus.main_state == m_sVehicleStatus.MAIN_STATE_MANUAL;
}


bool
CStateMachine::isDataLinked()
{
	bool l_bUpdated = false;

	orb_check(m_iVehicleStatusHandle, &l_bUpdated);
	if(l_bUpdated)
	{
		orb_copy(ORB_ID(vehicle_status), m_iVehicleStatusHandle, &m_sVehicleStatus);
	}

	return !m_sVehicleStatus.data_link_lost;
}


bool
CStateMachine::sendCommanderCmd(enum VEHICLE_CMD command, int param1, int param2)
{
	struct vehicle_command_s cmd;
	struct vehicle_status_s status;

	memset(&status, 0 , sizeof(status));
	memset(&cmd, 	0 , sizeof(cmd));


	int arm_sub_fd = orb_subscribe(ORB_ID(vehicle_status));

	orb_copy(ORB_ID(vehicle_status), arm_sub_fd, &status);

	cmd.command = command;
	cmd.target_system = status.system_id;
	cmd.source_system = status.system_id;
	cmd.target_component = 50;
	cmd.source_component = 123;
	if(command == VEHICLE_CMD_COMPONENT_ARM_DISARM)
	{
		cmd.param1 = param1;
	}
	else if(command == VEHICLE_CMD_DO_SET_MODE)
	{
		cmd.param1 = param1;
		cmd.param2 = param2;
	}

	orb_advertise(ORB_ID(vehicle_command), &cmd);

	orb_copy(ORB_ID(vehicle_status), arm_sub_fd, &status);

	switch(command)
	{
	case VEHICLE_CMD_COMPONENT_ARM_DISARM:
		return (param1 == ARMED) ? (status.main_state == status.ARMING_STATE_ARMED) : (status.main_state == status.ARMING_STATE_STANDBY);

	case VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
		return (status.nav_state == status.NAVIGATION_STATE_AUTO_RTL);

	default: return false;
	}
}


/*
 * getActuatorState()
 */
CStateMachine::EArmedState_t
CStateMachine::getActuatorState()
{
	struct actuator_armed_s arm;
	memset(&arm, 0 , sizeof(arm));

	int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

	return (EArmedState_t)(arm.ready_to_arm && arm.armed);
}


/**
 * setServo
 * @param channel: channel of servo to set [0,8]
 * @param value: value of the servo [-1,1]
 */
void
CStateMachine::setServo(short channel, double value)
{
	if(channel < 0 || channel > m_sUORBActuatorsControl.NUM_ACTUATOR_CONTROLS)
	{
		return;
	}

	m_sUORBActuatorsControl.control[channel] = value;
	m_sUORBActuatorsControl.timestamp = hrt_absolute_time();

	publishActuators();
}


#ifdef USE_RC_PARAM
void
CStateMachine::param_init()
{
	m_param_handles.GPSLocked	= param_find("RGS_GPSLOCKED");
	m_param_handles.altitude 	= param_find("RGS_ALTITUDE");
	m_param_handles.state_out	= param_find("RGS_STATE_OUT");

	m_param.altitude 	= -1.0;
	m_param.GPSLocked	= 0;
}
#endif


#ifdef USE_RC_PARAM
void
CStateMachine::param_update()
{
	param_get(m_param_handles.altitude, 	&m_param.altitude);
	param_get(m_param_handles.GPSLocked,	&m_param.GPSLocked);
}
#endif

void
CStateMachine::positionPoll(vehicle_global_position_s * pPosition)
{
	bool l_bDataChanged = false;

	if(nullptr == pPosition)
	{
		return;
	}

	orb_check(m_iPositionHanlde, &l_bDataChanged);

	if(true == l_bDataChanged)
	{
		orb_copy(ORB_ID(vehicle_global_position), m_iPositionHanlde, pPosition);
	}
}

/*
 * subscribe for uORB topics
 */
void
CStateMachine::subscribe()
{
	m_iPositionHanlde = orb_subscribe(ORB_ID(vehicle_global_position));
	m_iVehicleStatusHandle = orb_subscribe(ORB_ID(vehicle_status));
}

/*
 * advertise uORB topics
 */
void
CStateMachine::advertise()
{
	/*
	 * advertise Parachute state 1-3 ?
	 * 		1) para servo is in init state - we are on the way to sky
	 * 		2) flight mode - encapsulate balloon, return to home
	 * 		3) landing mode - eject parachute
	 */

	/*
	 * initialize data
	 */
	memset(&m_sUORBParaState, 			0, sizeof(m_sUORBParaState)			);
	memset(&m_sUORBActuatorsControl, 	0, sizeof(m_sUORBActuatorsControl)	);

	m_sUORBParaState.state = m_eState;

	/*
	 * advertise data
	 */
	m_parachutePub = orb_advertise(ORB_ID(parachute_state), 		&m_sUORBParaState			);
	m_actuatorsPub = orb_advertise(ORB_ID(actuator_controls_3),		&m_sUORBActuatorsControl	);
}
