/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file fixedwing.cpp
 *
 * Controller library code
 */

#include "fixedwing.hpp"

namespace control
{

namespace fixedwing
{

BlockYawDamper::BlockYawDamper(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	_rLowPass(this, "R_LP"),
	_rWashout(this, "R_HP"),
	_r2Rdr(this, "R2RDR"),
	_rudder(0)
{
}

BlockYawDamper::~BlockYawDamper() {};

void BlockYawDamper::update(float rCmd, float r, float outputScale)
{
	_rudder = outputScale * _r2Rdr.update(rCmd -
					      _rWashout.update(_rLowPass.update(r)));
}

BlockStabilization::BlockStabilization(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	_yawDamper(this, ""),
	_pLowPass(this, "P_LP"),
	_qLowPass(this, "Q_LP"),
	_p2Ail(this, "P2AIL"),
	_q2Elv(this, "Q2ELV"),
	_aileron(0),
	_elevator(0)
{
}

BlockStabilization::~BlockStabilization() {};

void BlockStabilization::update(float pCmd, float qCmd, float rCmd,
				float p, float q, float r, float outputScale)
{
	_aileron = outputScale * _p2Ail.update(
			   pCmd - _pLowPass.update(p));
	_elevator = outputScale * _q2Elv.update(
			    qCmd - _qLowPass.update(q));
	_yawDamper.update(rCmd, r, outputScale);
}

BlockMultiModeBacksideAutopilot::BlockMultiModeBacksideAutopilot(SuperBlock *parent, const char *name) :
	BlockUorbEnabledAutopilot(parent, name),
	_stabilization(this, ""), // no name needed, already unique

	// heading hold
	_psi2Phi(this, "PSI2PHI"),
	_phi2P(this, "PHI2P"),
	_phiLimit(this, "PHI_LIM"),

	// velocity hold
	_v2Theta(this, "V2THE"),
	_theta2Q(this, "THE2Q"),
	_theLimit(this, "THE"),
	_vLimit(this, "V"),

	// altitude/climb rate hold
	_h2Thr(this, "H2THR"),
	_cr2Thr(this, "CR2THR"),

	// guidance block
	_guide(this, ""),

	_trimAil(this, "TRIM_ROLL", false), 	/* general roll trim (full name: TRIM_ROLL) */
	_trimElv(this, "TRIM_PITCH", false), 	/* general pitch trim */
	_trimRdr(this, "TRIM_YAW", false), 	/* general yaw trim */
	_trimThr(this, "TRIM_THR"), 	/* FWB_ specific throttle trim (full name: FWB_TRIM_THR) */
	_trimV(this, "TRIM_V"), 	/* FWB_ specific trim velocity (full name : FWB_TRIM_V) */

	_vCmd(this, "V_CMD"),
	_crMax(this, "CR_MAX"),
	_attPoll(),
	_lastMissionCmd(),
	_timeStamp(0)
{
	_attPoll.fd = _att.getHandle();
	_attPoll.events = POLLIN;
}

void BlockMultiModeBacksideAutopilot::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) { return; } // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) { return; }

	// set dt for all child blocks
	setDt(dt);

	// store old position command before update if new command sent
	if (_missionCmd.updated()) {
		_lastMissionCmd = _missionCmd.getData();
	}

	// check for new updates
	if (_param_update.updated()) { updateParams(); }

	// get new information from subscriptions
	updateSubscriptions();

	// default all output to zero unless handled by mode
	for (unsigned i = 4; i < NUM_ACTUATOR_CONTROLS; i++) {
		_actuators.control[i] = 0.0f;
	}

	// only update guidance in auto mode
	if (_status.main_state == MAIN_STATE_AUTO) {
		// TODO use vehicle_control_mode here?
		// update guidance
		_guide.update(_pos, _att, _missionCmd.current, _lastMissionCmd.current);
	}

	// XXX handle STABILIZED (loiter on spot) as well
	// once the system switches from manual or auto to stabilized
	// the setpoint should update to loitering around this position

	// handle autopilot modes
	if (_status.main_state == MAIN_STATE_AUTO) {

		// calculate velocity, XXX should be airspeed,
		// but using ground speed for now for the purpose
		// of control we will limit the velocity feedback between
		// the min/max velocity
		float v = _vLimit.update(sqrtf(
						 _pos.vel_n * _pos.vel_n +
						 _pos.vel_e * _pos.vel_e +
						 _pos.vel_d * _pos.vel_d));

		// limit velocity command between min/max velocity
		float vCmd = _vLimit.update(_vCmd.get());

		// altitude hold
		float dThrottle = _h2Thr.update(_missionCmd.current.alt - _pos.alt);

		// heading hold
		float psiError = _wrap_pi(_guide.getPsiCmd() - _att.yaw);
		float phiCmd = _phiLimit.update(_psi2Phi.update(psiError));
		float pCmd = _phi2P.update(phiCmd - _att.roll);

		// velocity hold
		// negative sign because nose over to increase speed
		float thetaCmd = _theLimit.update(-_v2Theta.update(vCmd - v));
		float qCmd = _theta2Q.update(thetaCmd - _att.pitch);

		// yaw rate cmd
		float rCmd = 0;

		// stabilization
		float velocityRatio = _trimV.get() / v;
		float outputScale = velocityRatio * velocityRatio;
		// this term scales the output based on the dynamic pressure change from trim
		_stabilization.update(pCmd, qCmd, rCmd,
				      _att.rollspeed, _att.pitchspeed, _att.yawspeed,
				      outputScale);

		// output
		_actuators.control[CH_AIL] = _stabilization.getAileron() + _trimAil.get();
		_actuators.control[CH_ELV] = _stabilization.getElevator() + _trimElv.get();
		_actuators.control[CH_RDR] = _stabilization.getRudder() + _trimRdr.get();
		_actuators.control[CH_THR] = dThrottle + _trimThr.get();

		// XXX limit throttle to manual setting (safety) for now.
		// If it turns out to be confusing, it can be removed later once
		// a first binary release can be targeted.
		// This is not a hack, but a design choice.

		// do not limit in HIL
		if (_status.hil_state != HIL_STATE_ON) {
			/* limit to value of manual throttle */
			_actuators.control[CH_THR] = (_actuators.control[CH_THR] < _manual.throttle) ?
						     _actuators.control[CH_THR] : _manual.throttle;
		}

	} else if (_status.main_state == MAIN_STATE_MANUAL) {
		_actuators.control[CH_AIL] = _manual.roll;
		_actuators.control[CH_ELV] = _manual.pitch;
		_actuators.control[CH_RDR] = _manual.yaw;
		_actuators.control[CH_THR] = _manual.throttle;

	} else if (_status.main_state == MAIN_STATE_ALTCTL ||
		   _status.main_state == MAIN_STATE_POSCTL /* TODO, implement pos control */) {

		// calculate velocity, XXX should be airspeed, but using ground speed for now
		// for the purpose of control we will limit the velocity feedback between
		// the min/max velocity
		float v = _vLimit.update(sqrtf(
						 _pos.vel_n * _pos.vel_n +
						 _pos.vel_e * _pos.vel_e +
						 _pos.vel_d * _pos.vel_d));

		// pitch channel -> rate of climb
		// TODO, might want to put a gain on this, otherwise commanding
		// from +1 -> -1 m/s for rate of climb
		//float dThrottle = _cr2Thr.update(
		//_crMax.get()*_manual.pitch - _pos.vz);

		// roll channel -> bank angle
		float phiCmd = _phiLimit.update(_manual.roll * _phiLimit.getMax());
		float pCmd = _phi2P.update(phiCmd - _att.roll);

		// throttle channel -> velocity
		// negative sign because nose over to increase speed
		float vCmd = _vLimit.update(_manual.throttle *
					    (_vLimit.getMax() - _vLimit.getMin()) +
					    _vLimit.getMin());
		float thetaCmd = _theLimit.update(-_v2Theta.update(vCmd - v));
		float qCmd = _theta2Q.update(thetaCmd - _att.pitch);

		// yaw rate cmd
		float rCmd = 0;

		// stabilization
		_stabilization.update(pCmd, qCmd, rCmd,
				      _att.rollspeed, _att.pitchspeed, _att.yawspeed);

		// output
		_actuators.control[CH_AIL] = _stabilization.getAileron() + _trimAil.get();
		_actuators.control[CH_ELV] = _stabilization.getElevator() + _trimElv.get();
		_actuators.control[CH_RDR] = _stabilization.getRudder() + _trimRdr.get();

		// currently using manual throttle
		// XXX if you enable this watch out, vz might be very noisy
		//_actuators.control[CH_THR] = dThrottle + _trimThr.get();
		_actuators.control[CH_THR] = _manual.throttle;

		// XXX limit throttle to manual setting (safety) for now.
		// If it turns out to be confusing, it can be removed later once
		// a first binary release can be targeted.
		// This is not a hack, but a design choice.

		/* do not limit in HIL */
		if (_status.hil_state != HIL_STATE_ON) {
			/* limit to value of manual throttle */
			_actuators.control[CH_THR] = (_actuators.control[CH_THR] < _manual.throttle) ?
						     _actuators.control[CH_THR] : _manual.throttle;
		}

		// body rates controller, disabled for now
		// TODO

	} else if (
		0 /*_status.manual_control_mode == VEHICLE_MANUAL_CONTROL_MODE_SAS*/) {	// TODO use vehicle_control_mode here?

		_stabilization.update(_manual.roll, _manual.pitch, _manual.yaw,
				      _att.rollspeed, _att.pitchspeed, _att.yawspeed);

		_actuators.control[CH_AIL] = _stabilization.getAileron();
		_actuators.control[CH_ELV] = _stabilization.getElevator();
		_actuators.control[CH_RDR] = _stabilization.getRudder();
		_actuators.control[CH_THR] = _manual.throttle;
	}

	// update all publications
	updatePublications();
}

BlockMultiModeBacksideAutopilot::~BlockMultiModeBacksideAutopilot()
{
	// send one last publication when destroyed, setting
	// all output to zero
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		_actuators.control[i] = 0.0f;
	}

	updatePublications();
}

} // namespace fixedwing

} // namespace control

