#include "BlockSegwayController.hpp"

// px4
#include <geo/geo.h>
#include <drivers/drv_hrt.h>

BlockSegwayController::BlockSegwayController() :
	SuperBlock(NULL, "SEG"),

	// subscriptions
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 3),
	_pos(&getSubscriptions() , ORB_ID(vehicle_global_position), 3),
	_posCmd(&getSubscriptions(), ORB_ID(position_setpoint_triplet), 3),
	_localPos(&getSubscriptions() , ORB_ID(vehicle_local_position), 3),
	_localPosCmd(&getSubscriptions(), ORB_ID(vehicle_local_position_setpoint), 3),
	_manual(&getSubscriptions(), ORB_ID(manual_control_setpoint), 3),
	_status(&getSubscriptions(), ORB_ID(vehicle_status), 3),
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	_encoders(&getSubscriptions(), ORB_ID(encoders), 10), // limit to 100 Hz
	_battery(&getSubscriptions(), ORB_ID(battery_status), 10), // limit to 100 Hz

	// publications
	_attCmd(&getPublications(), ORB_ID(vehicle_attitude_setpoint)),
	_ratesCmd(&getPublications(), ORB_ID(vehicle_rates_setpoint)),
	_globalVelCmd(&getPublications(), ORB_ID(vehicle_global_velocity_setpoint)),
	_actuators(&getPublications(), ORB_ID(actuator_controls_1)),

	_yaw2r(this, "YAW2R"),
	_r2v(this, "R2V"),
	_th2v(this, "TH2V"),
	_q2v(this, "Q2V"),
	_x2vel(this, "X2VEL"),
	_vel2th(this, "VEL2TH"),
	_thLimit(this, "TH_LIM"),
	_velLimit(this, "VEL_LIM"),
	_thStop(this, "TH_STOP"),
	_pulsesPerRev(this, "ENCP_PPR", false),

	_mgl(this, "MGL"),
	_J(this, "J"),
	_k_emf(this, "K_EMF"),
	_k_damp(this, "K_DAMP"),
	_wn_theta(this, "WN_THETA"),
	_zeta_theta(this, "ZETA_THETA"),

	_trimPitch(this, "TRIM_PITCH", false),
	_sysIdAmp(this, "SYSID_AMP"),
	_sysIdFreq(this, "SYSID_FREQ"),
	_attPoll(),
	_timeStamp(0)
{
	orb_set_interval(_att.getHandle(), 10); // set attitude update rate to 100 Hz (period 10 ms)
	_attPoll.fd = _att.getHandle();
	_attPoll.events = POLLIN;
}

float BlockSegwayController::computeVelocityCmd(float posCmd)
{
	return _velLimit.update(_x2vel.update(posCmd - _localPos.x));
}

float BlockSegwayController::computeYawRateCmd(float yawCmd)
{
	float yawError = yawCmd - _att.yaw;

	// wrap yaw error to between -180 and 180
	if (yawError > M_PI_F / 2) { yawError = yawError - 2 * M_PI_F; }

	if (yawError < -M_PI_F / 2) { yawError = yawError + 2 * M_PI_F; }

	return _yaw2r.update(yawError);
}

float BlockSegwayController::computePitchCmd(float velCmd)
{
	// negative sign since need to lean in negative pitch to move forward
	return -_thLimit.update(_vel2th.update(velCmd - _localPos.vx));
}

void BlockSegwayController::update()
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

	// check for new updates
	if (_param_update.updated()) { updateParams(); }

	// get new information from subscriptions
	updateSubscriptions();

	// default all output to zero unless handled by mode
	for (unsigned i = 2; i < NUM_ACTUATOR_CONTROLS; i++) {
		_actuators.control[i] = 0.0f;
	}

	// commands for inner stabilization loop
	float thCmd = 0; // pitch command
	float rCmd = 0; // yaw rate command
	float yawCmd = 0; // always point north for now, can use localPosCmd.yaw later
	float velCmd = 0; // velocity command
	float posCmd = 0; // position command

	// signals for system id
	float sineWave = _sysIdAmp.get() * sinf(2.0f * M_PI_F * _sysIdFreq.get() * _timeStamp / 1.0e6f);
	float squareWave = 0;

	if (sineWave > 0) {
		squareWave = _sysIdAmp.get();

	} else {
		squareWave = -_sysIdAmp.get();
	}

	// modes
	if (_status.main_state == MAIN_STATE_MANUAL) {
		// user controls vel cmd and yaw rate cmd
		velCmd = _manual.x * _velLimit.getMax();
		thCmd = computePitchCmd(velCmd);
		rCmd = _manual.y;

	} else if (_status.main_state == MAIN_STATE_ALTCTL) {
		// user controls vel cmd and yaw cmd
		velCmd = _manual.x * _velLimit.getMax();
		thCmd = computePitchCmd(velCmd);
		yawCmd = _manual.y;
		rCmd = computeYawRateCmd(yawCmd);

	} else if (_status.main_state == MAIN_STATE_ACRO) {
		// user controls th cmd and yaw rate cmd
		thCmd = _manual.x * _thLimit.getMax();
		rCmd = _manual.y;

	} else if (_status.main_state == MAIN_STATE_POSCTL) {
		// user controls pos cmd and yaw rate cmd
		posCmd = 0.5f * _manual.x;
		velCmd = computeVelocityCmd(posCmd);
		thCmd = computePitchCmd(velCmd);
		rCmd = _manual.y;

	} else if (_status.main_state == MAIN_STATE_AUTO_MISSION) {
		posCmd = _localPosCmd.x;
		velCmd = computeVelocityCmd(posCmd);
		thCmd = computePitchCmd(velCmd);
		yawCmd = _localPosCmd.yaw;
		rCmd = computeYawRateCmd(yawCmd);

	} else if (_status.main_state == MAIN_STATE_AUTO_LOITER) {
		posCmd = _localPosCmd.x; // TODO check if local pos cmd is set to loiter pos.
		velCmd = computeVelocityCmd(posCmd);
		thCmd = computePitchCmd(velCmd);
		yawCmd = _localPosCmd.yaw;
		rCmd = computeYawRateCmd(yawCmd);

	} else if (_status.main_state == MAIN_STATE_AUTO_RTL) {
		posCmd = 0;
		velCmd = computeVelocityCmd(posCmd);
		thCmd = computePitchCmd(velCmd);
		yawCmd = _localPosCmd.yaw;
		rCmd = computeYawRateCmd(yawCmd);

	} else if (_status.main_state == MAIN_STATE_OFFBOARD) {
		posCmd = sineWave;
		velCmd = computeVelocityCmd(posCmd);
		thCmd = computePitchCmd(velCmd);
		yawCmd = _localPosCmd.yaw;
		rCmd = computeYawRateCmd(yawCmd);

	} else if (_status.main_state == MAIN_STATE_MAX) {
		posCmd = squareWave;
		velCmd = computeVelocityCmd(posCmd);
		thCmd = computePitchCmd(velCmd);
		yawCmd = _localPosCmd.yaw;
		rCmd = computeYawRateCmd(yawCmd);
	}


	// compute angles and rates
	float th = _att.pitch -_trimPitch.get();
	float th_dot = _att.pitchspeed;
	float alpha_dot_left = _encoders.velocity[0]*2*M_PI_F/_pulsesPerRev.get();
	float alpha_dot_right = _encoders.velocity[1]*2*M_PI_F/_pulsesPerRev.get();
	float alpha_dot = (alpha_dot_left + alpha_dot_right)/2;

	// constants
	float k_emf = _k_emf.get();
	float k_damp = _k_damp.get();
	float wn_theta = _wn_theta.get();
	float zeta_theta = _zeta_theta.get();
	float J = _J.get();
	float mgl = _mgl.get();
	float V_batt = _battery.voltage_filtered_v;

	// dynamic inversion
	float V_pitch = J*wn_theta*(wn_theta*(thCmd - th) + 2*zeta_theta*th_dot) - mgl*sinf(th)/(2*k_emf) + k_damp*alpha_dot/k_emf;
	float V_yaw = k_damp*(alpha_dot_left - alpha_dot_right)/k_emf;

	// compute duty (0-1)
	float dutyPitch = V_pitch/V_batt;
	float dutyYaw = V_yaw/V_batt;

	// output scaling by manual throttle
	dutyPitch *= _manual.z;
	dutyYaw *= _manual.z;

	// attitude set point
	_attCmd.timestamp = _timeStamp;
	_attCmd.pitch_body = thCmd;
	_attCmd.roll_body = 0;
	_attCmd.yaw_body = yawCmd;
	_attCmd.R_valid = false;
	_attCmd.q_d_valid = false;
	_attCmd.q_e_valid = false;
	_attCmd.thrust = 0;
	_attCmd.roll_reset_integral = false;
	_attCmd.update();

	// rates set point
	_ratesCmd.timestamp = _timeStamp;
	_ratesCmd.roll = 0;
	_ratesCmd.pitch = 0;
	_ratesCmd.yaw = rCmd;
	_ratesCmd.thrust = 0;
	_ratesCmd.update();

	// global velocity set point
	_globalVelCmd.vx = velCmd;
	_globalVelCmd.vy = 0;
	_globalVelCmd.vz = 0;
	_globalVelCmd.update();

	// send outputs if armed and pitch less
	// than shut off pitch
	if (_status.arming_state == ARMING_STATE_ARMED &&
	    fabsf(_att.pitch) < _thStop.get()) {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = dutyPitch; // pitch
		_actuators.control[2] = dutyYaw; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();

	} else {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = 0; // pitch
		_actuators.control[2] = 0; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();
	}
}

