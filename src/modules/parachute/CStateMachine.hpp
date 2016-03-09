/*
 * CStateMachine.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: friedrich
 */

#ifndef CSTATEMACHINE_HPP_
#define CSTATEMACHINE_HPP_

#include <drivers/drv_hrt.h>

#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>


#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

/*
 * include uORB messages
 */
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parachute_state.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>

#include "CRingBuffer.hpp"

#include "../commander/px4_custom_mode.h"

// states
class CStateInit;
class CStateFlight;
class CStateLanding;
#include "CState.hpp"
#include "states/CStateInit.hpp"
#include "states/CStateFlight.hpp"
#include "states/CStateLanding.hpp"

#ifndef OK
#define OK 0
#endif

#define POSITION_STABLE_TIMEOUT 10000000

/*
 * CStateMachine
 * This class holds the following 3 states:
 * 	- INIT
 * 	- FLIGHT
 * 	- LANDING
 * 	and a dummy state as default initialization and
 * 	as fallback mechanism state
 * 	This state machine does the state handling autonomously
 * 	and provides the following interface calls:
 * 	- init()
 * 	- run()
 */
class CStateMachine
{
public:
	friend class CStateInit;
	friend class CStateFlight;
	friend class CStateLanding;
	/**
	 * uORB advertising content
	 */
	typedef enum EState
	{
		INIT 	= 0,		/**< UAV moving up with balloon attached */
		FLIGHT 	= 1,		/**< Balloon is detached */
		LANDING = 2, 		/**< parachute active	*/

		MAX_STATES = 3
	} EState_t;

	typedef enum EArmedState
	{
		DISARMED = 0,
		ARMED = 1
	} EArmedState_t;


	// from commander.cpp:
	enum MAV_MODE_FLAG {
		MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, 	/* 0b00000001 Reserved for future use. | */
		MAV_MODE_FLAG_TEST_ENABLED = 2, 		/* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
		MAV_MODE_FLAG_AUTO_ENABLED = 4, 		/* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
		MAV_MODE_FLAG_GUIDED_ENABLED = 8, 		/* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
		MAV_MODE_FLAG_STABILIZE_ENABLED = 16, 	/* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
		MAV_MODE_FLAG_HIL_ENABLED = 32, 		/* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
		MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,/* 0b01000000 remote control input is enabled. | */
		MAV_MODE_FLAG_SAFETY_ARMED = 128, 		/* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
		MAV_MODE_FLAG_ENUM_END = 129, 			/*  | */
	};

private:

	// states
	CState			* m_pCurrentState;

	CStateInit 		* m_pStateInit;
	CStateFlight 	* m_pStateFlight;
	CStateLanding 	* m_pStateLanding;

#ifdef USE_RC_PARAM
	struct {
		float altitude;
		int	  GPSLocked;
	} m_param;

	struct {
		param_t state_out;
		param_t altitude;
		param_t GPSLocked;
	} m_param_handles;
#endif

	EState_t	 	m_eState;
	bool	 		m_bStateHasChanged;

	/*
	 * advertised topics
	 */
	orb_advert_t	m_actuatorsPub;		/**< actuator control topic */
	orb_advert_t	m_parachutePub;		/**< parachute's state topic */

	struct parachute_state_s 			m_sUORBParaState;
	struct actuator_controls_s			m_sUORBActuatorsControl;
	struct vehicle_global_position_s 	m_sPosition;
	struct vehicle_status_s 			m_sVehicleStatus;
	//struct battery_status_s 			m_sBatteryStatus;	/**< battery status */

	/*
	 * subscribe topics
	 */
	int 	m_iPositionHanlde;		/**< vehicle position  */
	int 	m_iVehicleStatusHandle;

	hrt_abstime m_tPositionValidTime;

#if MAVLINK_VERBOSE > 0
	int				m_iMavLinkFd;
#endif

public:
	CStateMachine();
	~CStateMachine();

	void 	run();
	int 	init();

	inline bool stateHasChanged();
	inline EState_t getState();

private:
	int changeState(EState_t);
	bool setActuatorState(EArmedState_t);
	EArmedState_t getActuatorState();

	void setServo(short, double);

	inline bool setVehicleState(EArmedState_t);
	EArmedState_t getVehicleState();

	bool	isFailsafe();
	bool	isPositionStable();
	bool 	isManualMode();
	bool	isDataLinked();
	bool	isBatteryCritical();

	void 	subscribe();
	void 	advertise();
	void 	waitForGPSLock();

	bool	sendCommanderCmd(enum VEHICLE_CMD, int param1 = 0, int param2 = 0);

	void 	positionPoll(vehicle_global_position_s *);

	inline 	void 	publishState();
	inline 	void 	publishActuators();
	inline 	bool 	setAutoRTLMode();
	inline 	bool 	setAutoLoiterMode();

#ifdef USE_RC_PARAM
	void param_init();
	void param_update();
#endif
};

/*
 * getState
 * @return current state of state machine
 */
CStateMachine::EState_t
CStateMachine::getState()
{
	return m_eState;
}

/*
 * stateHasChanged
 * @return true: when state has changed recently -
 * 					before next call of run()
 */
bool
CStateMachine::stateHasChanged()
{
	return m_bStateHasChanged;
}


/*
 * publishState
 * Inform other processes that new data is available to copy
 */
void
CStateMachine::publishState()
{
	m_sUORBParaState.state = m_eState;

	orb_publish(ORB_ID(parachute_state),
				m_parachutePub,
				&m_sUORBParaState);

#ifdef USE_RC_PARAM
	param_set(m_param_handles.state_out, &m_sUORBParaState);
#endif
}


/*
 * publishActuators()
 * write PWM values to PX4IO
 */
void
CStateMachine::publishActuators()
{
	EArmedState_t eState = getActuatorState();
	if(DISARMED == eState)
	{
		// arm system for output
		if(OK != setActuatorState(ARMED))
		{
			#if MAVLINK_VERBOSE > 1
			mavlink_and_console_log_info(m_iMavLinkFd, "PARA: arming actuators \t[failed]\n");
			#endif
		}
		else
		{
			#if MAVLINK_VERBOSE > 1
			mavlink_and_console_log_info(m_iMavLinkFd, "PARA: arming actuators \t[done]\n");
			#endif
		}
	}
	else
	{
		#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: actuators already armed\n");
		#endif
	}

	orb_publish(ORB_ID(actuator_controls_3),
				m_actuatorsPub,
				&m_sUORBActuatorsControl);

	if(OK != setActuatorState(eState))
	{
		#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: reset actuators arming state \t[failed]\n");
		#endif
	}
	else
	{
		#if MAVLINK_VERBOSE > 1
		mavlink_and_console_log_info(m_iMavLinkFd, "PARA: reset actuators arming state \t[done]\n");
		#endif
	}
}


/*
 * setVehicleState
 * @param	eState: new vehicle state (ARMED | DISARMED)
 */
bool
CStateMachine::setVehicleState(EArmedState_t eState)
{
	return sendCommanderCmd(VEHICLE_CMD_COMPONENT_ARM_DISARM, (eState == ARMED));
}


/*
 * setAutoLoiterMode
 * changes the vehicle state to Auto Loiter
 * @return true: when state has changed successfully, false otherwise
 */
bool
CStateMachine::setAutoLoiterMode()
{
	int l_iMode;
	l_iMode =  (PX4_CUSTOM_MAIN_MODE_AUTO << 8) | PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
	bool l_bReturn = sendCommanderCmd(VEHICLE_CMD_DO_SET_MODE, 0, l_iMode);

	l_bReturn |= sendCommanderCmd(VEHICLE_CMD_NAV_LOITER_UNLIM);
	return l_bReturn;
}


/*
 * setAutoRTLMode
 * changes the vehicle state to Auto Return To Launch Mode
 * @return 	true: when state has changed successfully, false otherwise
 */
bool
CStateMachine::setAutoRTLMode()
{
	int l_iBaseMode, l_iCustomMode;
	l_iBaseMode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	l_iCustomMode = PX4_CUSTOM_MAIN_MODE_AUTO;
	bool l_bReturn = sendCommanderCmd(VEHICLE_CMD_DO_SET_MODE, l_iBaseMode, l_iCustomMode);

	l_bReturn |= sendCommanderCmd(VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
	return l_bReturn;
}

#endif /* CSTATEMACHINE_HPP_ */
