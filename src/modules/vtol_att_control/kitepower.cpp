/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file kitepower.cpp
*
* @author Roman Babst		<bapstroman@gmail.com>
* @author Gabriel Koenig		<g.koenig@kitepower.nl>
*
*/

#include "kitepower.h"
#include "vtol_att_control_main.h"


Kitepower::Kitepower(VtolAttitudeControl *attc) :
	VtolType(attc),
	_airspeed_tot(0.0f),
        _flag_enable_mc_motors(true),
	_thrust_transition_start(0.0f),
	_yaw_transition(0.0f),
	_pitch_transition_start(0.0f),
	_loop_perf(perf_alloc(PC_ELAPSED, "vtol_att_control-kitepower")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "vtol att control-kitepower nonfinite input"))
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
        _mc_throttle_weight = 1.0f;

	_flag_was_in_trans_mode = false;


}

Kitepower::~Kitepower()
{

}

void
Kitepower::parameters_update()
{

}

void Kitepower::update_vtol_state()
{

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

    //matrix::Eulerf euler = matrix::Quatf(_v_att->q);
    // Kitepower: not needed at the moment
    //float pitch = euler.theta();

	if (!_attc->is_fixed_wing_requested()) {


		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case MC_MODE:
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode 	= TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
                        //_flag_enable_mc_motors = true;
                        break;

                case TRANSITION_FRONT:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
                        //_flag_enable_mc_motors = true;
			break;


		case TRANSITION_BACK:

                //Kitepower: direct switch for the moment for transition
                        _vtol_schedule.flight_mode = MC_MODE;
                        //_flag_enable_mc_motors = true;
                        break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
                        // initialise a front transition
                        _vtol_schedule.flight_mode 	= TRANSITION_FRONT;
			_vtol_schedule.transition_start = hrt_absolute_time();
                        //_flag_enable_mc_motors = false;
			break;

		case FW_MODE:
                        //_flag_enable_mc_motors = false;
                        break;

                case TRANSITION_FRONT:
                        //Kitpower: direct switch for the moment for front transition
                        _vtol_schedule.flight_mode = FW_MODE;
                        //_flag_enable_mc_motors = false;
			break;

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
                        //_flag_enable_mc_motors = false;

            break;
		}
	}
        update_external_VTOL_state();
}
void Kitepower::update_external_VTOL_state()
{
	// map kitepower specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
                _flag_was_in_trans_mode = false;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
                _flag_was_in_trans_mode = false;
		break;

        case TRANSITION_FRONT:
		_vtol_mode = TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

void Kitepower::update_transition_state()
{
	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_yaw_transition = _v_att_sp->yaw_body;
		_pitch_transition_start = _v_att_sp->pitch_body;
		_thrust_transition_start = _v_att_sp->thrust;
		_flag_was_in_trans_mode = true;
	}

    if (_vtol_schedule.flight_mode == TRANSITION_FRONT) {
        // Kitepower: at the moment empty as we have direct switch to fixed wing
        // create time dependant pitch angle set point + 0.2 rad overlap over the switch value*/

        // create time dependant throttle signal higher than  in MC and growing untillspeed reached */

        // eventually disable mc yaw control once the plane has picked up speed

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {

        // create time dependant pitch angle set point stating at -pi/2 + 0.2 rad overlap over the switch value*/

		//  throttle value is decreesed

        // smoothly move control weight to MC */

	}



	// compute desired attitude and thrust setpoint for the transition
        // Kitepower: at the moment we just use the same values at the beginning of the transition

	_v_att_sp->timestamp = hrt_absolute_time();
	_v_att_sp->roll_body = 0.0f;
        _v_att_sp->yaw_body = _yaw_transition;
        _v_att_sp->pitch_body = _pitch_transition_start;
        _v_att_sp->thrust = _thrust_transition_start;

	math::Quaternion q_sp;
	q_sp.from_euler(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body);
	memcpy(&_v_att_sp->q_d[0], &q_sp.data[0], sizeof(_v_att_sp->q_d));
}

void Kitepower::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust = _thrust_transition;
}

void Kitepower::calc_tot_airspeed()
{
	float airspeed = math::max(1.0f, _airspeed->indicated_airspeed_m_s);	// prevent numerical drama
	// calculate momentary power of one engine
	float P = _batt_status->voltage_filtered_v * _batt_status->current_a / _params->vtol_motor_count;
	P = math::constrain(P, 1.0f, _params->power_max);
	// calculate prop efficiency
	float power_factor = 1.0f - P * _params->prop_eff / _params->power_max;
	float eta = (1.0f / (1 + expf(-0.4f * power_factor * airspeed)) - 0.5f) * 2.0f;
	eta = math::constrain(eta, 0.001f, 1.0f);	// live on the safe side
	// calculate induced airspeed by propeller
	float v_ind = (airspeed / eta - airspeed) * 2.0f;
	// calculate total airspeed
	float airspeed_raw = airspeed + v_ind;
	// apply low-pass filter
	_airspeed_tot = _params->arsp_lp_gain * (_airspeed_tot - airspeed_raw) + airspeed_raw;
}

void Kitepower::scale_mc_output()
{
	// scale around tuning airspeed
	float airspeed;
	calc_tot_airspeed();	// estimate air velocity seen by elevons

	// if airspeed is not updating, we assume the normal average speed
	if (bool nonfinite = !PX4_ISFINITE(_airspeed->indicated_airspeed_m_s) ||
			     hrt_elapsed_time(&_airspeed->timestamp) > 1e6) {
		airspeed = _params->mc_airspeed_trim;

		if (nonfinite) {
			perf_count(_nonfinite_input_perf);
		}

	} else {
		airspeed = _airspeed_tot;
		airspeed = math::constrain(airspeed, _params->mc_airspeed_min, _params->mc_airspeed_max);
	}

	_vtol_vehicle_status->airspeed_tot = airspeed;	// save value for logging
	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	float airspeed_scaling = _params->mc_airspeed_trim / ((airspeed < _params->mc_airspeed_min) ? _params->mc_airspeed_min :
				 airspeed);
	_actuators_mc_in->control[1] = math::constrain(_actuators_mc_in->control[1] * airspeed_scaling * airspeed_scaling,
				       -1.0f, 1.0f);
}

void Kitepower::update_mc_state()
{
        VtolType::update_mc_state();
        _mc_roll_weight = 1.0f;
        _mc_pitch_weight = 1.0f;
        _mc_yaw_weight = 1.0f;
        _mc_throttle_weight = 1.0f;

        /*for the moment let the motor spin during fixed wing, nice for simulation
        if (_flag_enable_mc_motors) {
                set_max_mc(1940);
                _flag_enable_mc_motors = false;
        }*/

}

void Kitepower::update_fw_state()
{
        VtolType::update_fw_state();
        _mc_roll_weight = 0.0f;
        _mc_pitch_weight = 0.0f;
        _mc_yaw_weight = 0.0f;
        _mc_throttle_weight = 0.0f;

        /*for the moment let the motor spin during fixed wing, nice for simulation
        if (!_flag_enable_mc_motors) {
                set_max_mc(940);
                _flag_enable_mc_motors = true;
        }*/


}

/**
* Write data to actuator output topic.
*/

void Kitepower::fill_actuator_outputs()
{
    //Kitepower: fail saife mode needs to be added -> probably later on dronecore side
    //fill multicopter controls, index 0
    _actuators_out_0->timestamp = _actuators_mc_in->timestamp;
    _actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
            _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]*_mc_roll_weight;
    _actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
            _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH]*_mc_pitch_weight;
    _actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
            _actuators_mc_in->control[actuator_controls_s::INDEX_YAW]*_mc_yaw_weight; //needs to be tested
    _actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
            _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE]*_mc_throttle_weight;

    //fill in fixed wing controls, important actuator for
    //in the fw_att_controller there is a coordinate frame transformation while switching to fw mode (neutral position is switched 90° around pitch axis)
    //Thus we have a coordinate frame for mc and one
    //for fw. Thus mc_yaw!=fw_yaw.
    _actuators_out_1->timestamp = _actuators_fw_in->timestamp;
    _actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =0.0f;
    _actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
            _actuators_fw_in->control[actuator_controls_s::INDEX_PITCH]*(1-_mc_pitch_weight);
    _actuators_out_1->control[actuator_controls_s::INDEX_YAW] = _actuators_fw_in->control[actuator_controls_s::INDEX_YAW]*(1-_mc_yaw_weight); //yaw axis in multicopter mode means roll axis in fixedwing, thus no roll control

}


/**
* Disable all multirotor motors when in fw mode.
*/
/*for the moment let the motor spin during fixed wing, nice for simulation
void
Kitepower::set_max_mc(unsigned pwm_value)
{
        int ret;
        unsigned servo_count;
        const char *dev = PWM_OUTPUT0_DEVICE_PATH;
        int fd = px4_open(dev, 0);

        if (fd < 0) {
                PX4_WARN("can't open %s", dev);
        }

        ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
        struct pwm_output_values pwm_values;
        memset(&pwm_values, 0, sizeof(pwm_values));

        for (int i = 0; i < _params->vtol_motor_count; i++) {
                pwm_values.values[i] = pwm_value;
                pwm_values.channel_count = _params->vtol_motor_count;
        }

        ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

        if (ret != OK) {
                PX4_WARN("failed setting max values");
        }

        px4_close(fd);
}*/
