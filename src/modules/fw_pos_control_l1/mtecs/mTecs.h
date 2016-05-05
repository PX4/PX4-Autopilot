/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file mTecs.h
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */


#ifndef MTECS_H_
#define MTECS_H_

#include "mTecs_blocks.h"
#include "limitoverride.h"

#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>

#if defined(__PX4_NUTTX)
#include <uORB/topics/tecs_status.h>
#endif // defined(__PX4_NUTTX)

namespace fwPosctrl
{

/* corresponds to TECS_MODE in tecs_status.msg */
enum MTECS_MODE {
	MTECS_MODE_NORMAL = 0,
	MTECS_MODE_UNDERSPEED = 1,
	MTECS_MODE_TAKEOFF = 2,
	MTECS_MODE_LAND = 3,
	MTECS_MODE_LAND_THROTTLELIM = 4,
	MTECS_MODE_BAD_DESCENT = 5,
	MTECS_MODE_CLIMBOUT = 6
};

/* Main class of the mTecs */
class mTecs : public control::SuperBlock
{
public:
	mTecs();
	virtual ~mTecs();

	/*
	 * Control in altitude setpoint and speed mode
	 */
	int updateAltitudeSpeed(float flightPathAngle, float altitude, float altitudeSp, float airspeed,
				float airspeedSp, unsigned mode, LimitOverride limitOverride);

	/*
	 * Control in flightPathAngle setpoint (flollow a slope etc.) and speed mode
	 */
	int updateFlightPathAngleSpeed(float flightPathAngle, float flightPathAngleSp, float airspeed,
				       float airspeedSp, unsigned mode, LimitOverride limitOverride);

	/*
	 * Control in flightPathAngle setpoint (flollow a slope etc.) and acceleration mode (base case)
	 */
	int updateFlightPathAngleAcceleration(float flightPathAngle, float flightPathAngleSp, float airspeedFiltered,
					      float accelerationLongitudinalSp, unsigned mode, LimitOverride limitOverride);

	/*
	 * Reset all integrators
	 */
	void resetIntegrators();

	/*
	 * Reset all derivative calculations
	 */
	void resetDerivatives(float airspeed);

	/* Accessors */
	bool getEnabled() { return _mTecsEnabled.get() > 0; }
	float getThrottleSetpoint() { return _throttleSp; }
	float getPitchSetpoint() { return _pitchSp; }
	float airspeedLowpassUpdate(float input) { return _airspeedLowpass.update(input); }
	float getFlightPathAngleLowpassState() { return _flightPathAngleLowpass.getState(); }
	float getAltitudeLowpassState() { return _altitudeLowpass.getState(); }
	float getAirspeedLowpassState() { return _airspeedLowpass.getState(); }
	float getAirspeedDerivativeLowpassState() { return _airspeedDerivative.getO(); }

protected:
	/* parameters */
	control::BlockParamInt _mTecsEnabled;		/**< 1 if mTecs is enabled */
	control::BlockParamFloat _airspeedMin;		/**< minimal airspeed */

	/* Publications */
#if defined(__PX4_NUTTX)
	uORB::Publication<tecs_status_s> _status;	/**< publish internal values for logging */
#endif // defined(__PX4_NUTTX)

	/* control blocks */
	BlockFFPILimitedCustom _controlTotalEnergy;		/**< FFPI controller for total energy control: output
								  is throttle */
	BlockFFPILimitedCustom _controlEnergyDistribution;	/**< FFPI controller for energy distribution control:
								  output is pitch */
	BlockPDLimited	_controlAltitude;			/**< PD controller for altitude: output is the flight
								  path angle setpoint */
	BlockPDLimited	_controlAirSpeed;			/**< PD controller for airspeed: output is acceleration
								  setpoint */

	/* Other calculation Blocks */
	control::BlockLowPass2 _flightPathAngleLowpass;	/**< low pass filter for the flight path angle */
	control::BlockLowPass2 _altitudeLowpass;	/**< low pass filter for altitude */
	control::BlockLowPass2 _airspeedLowpass;		/**< low pass filter for airspeed */
	control::BlockDerivative _airspeedDerivative;	/**< airspeed derivative calulation */

	/* Output setpoints */
	float _throttleSp;				/**< Throttle Setpoint from 0 to 1 */
	float _pitchSp;					/**< Pitch Setpoint from -pi to pi */

	/* Output Limits in special modes */
	BlockOutputLimiter _BlockOutputLimiterTakeoffThrottle;	    /**< Throttle Limits during takeoff */
	BlockOutputLimiter _BlockOutputLimiterTakeoffPitch;	    /**< Pitch Limit during takeoff */
	BlockOutputLimiter _BlockOutputLimiterUnderspeedThrottle;   /**< Throttle Limits when underspeed is detected */
	BlockOutputLimiter _BlockOutputLimiterUnderspeedPitch;	    /**< Pitch Limit when underspeed is detected */
	BlockOutputLimiter _BlockOutputLimiterLandThrottle;	    /**< Throttle Limits during landing (only in
								      last phase)*/
	BlockOutputLimiter _BlockOutputLimiterLandPitch;	    /**< Pitch Limit during landing */

	/* Time measurements */
	hrt_abstime timestampLastIteration;	/**< Saves the result of hrt_absolute_time() of the last iteration */

	bool _firstIterationAfterReset;		/**< True during the first iteration after a reset */
	bool _dtCalculated;			/**< True if dt has been calculated in this iteration */

	int _counter;
	bool _debug;				///< Set true to enable debug output


	static void	debug_print(const char *fmt, va_list args);
	void		debug(const char *fmt, ...);

	/*
	 * Measure and update the time step dt if this was not already done in the current iteration
	 */
	void updateTimeMeasurement();
};

} /* namespace fwPosctrl */

#endif /* MTECS_H_ */
