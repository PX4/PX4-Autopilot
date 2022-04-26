/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "FixedwingPositionINDIControl.hpp"

using namespace std;

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Matrix;
using matrix::Euler;
using matrix::Quatf;
using matrix::AxisAnglef;
using matrix::Vector3f;
using matrix::Vector;
using matrix::wrap_pi;


FixedwingPositionINDIControl::FixedwingPositionINDIControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _actuators_0_pub(ORB_ID(actuator_controls_0)),
    _attitude_sp_pub(ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// limit to 50 Hz
	//_vehicle_angular_velocity_sub.set_interval_ms(20);

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionINDIControl::~FixedwingPositionINDIControl()
{
	perf_free(_loop_perf);
}

int
FixedwingPositionINDIControl::parameters_update()
{
	updateParams();

	// INDI parameters
    _K_x *= 0.f;
    _K_v *= 0.f;
    _K_a *= 0.f;
    _K_q *= 0.f;
    _K_w *= 0.f;
    _K_x(0,0) = _param_k_x_roll.get();
    _K_x(1,1) = _param_k_x_pitch.get();
    _K_x(2,2) = _param_k_x_yaw.get();
    _K_v(0,0) = _param_k_v_roll.get();
    _K_v(1,1) = _param_k_v_pitch.get();
    _K_v(2,2) = _param_k_v_yaw.get();
    _K_a(0,0) = _param_k_a_roll.get();
    _K_a(1,1) = _param_k_a_pitch.get();
    _K_a(2,2) = _param_k_a_yaw.get();
    _K_q(0,0) = _param_k_q_roll.get();
    _K_q(1,1) = _param_k_q_pitch.get();
    _K_q(2,2) = _param_k_q_yaw.get();
    _K_w(0,0) = _param_k_w_roll.get();
    _K_w(1,1) = _param_k_w_pitch.get();
    _K_w(2,2) = _param_k_w_yaw.get();

    // aircraft parameters
    _inertia(0,0) = _param_fw_inertia_roll.get();
    _inertia(1,1) = _param_fw_inertia_pitch.get();
    _inertia(2,2) = _param_fw_inertia_yaw.get();
    _mass = _param_fw_mass.get();
    _area = _param_fw_wing_area.get();
    _rho = _param_rho.get();
    _C_L0 = _param_fw_c_l0.get();
    _C_L1 = _param_fw_c_l1.get();
    _C_D0 = _param_fw_c_d0.get();
    _C_D1 = _param_fw_c_d1.get();
    _C_D2 = _param_fw_c_d2.get();

    // filter parameters
    _a1 = _param_filter_a1.get();
    _a2 = _param_filter_a2.get();
    _b1 = _param_filter_b1.get();
    _b2 = _param_filter_b2.get();
    _b3 = _param_filter_b3.get();

    // actuator gains
    _K_actuators *= 0.0f;
    _K_actuators(0,0) = _param_k_act_roll.get();
    _K_actuators(1,1) = _param_k_act_pitch.get();
    _K_actuators(2,2) = _param_k_act_yaw.get();

	// sanity check parameters
    // TODO: include sanity check

	return PX4_OK;
}

void
FixedwingPositionINDIControl::airspeed_poll()
{
	bool airspeed_valid = _airspeed_valid;
	airspeed_validated_s airspeed_validated;

	if (_airspeed_validated_sub.update(&airspeed_validated)) {

		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && (airspeed_validated.calibrated_airspeed_m_s > 0.0f)) {

			airspeed_valid = true;

			_airspeed_last_valid = airspeed_validated.timestamp;
			_airspeed = airspeed_validated.calibrated_airspeed_m_s;
		}

	} else {
		// no airspeed updates for one second
		if (airspeed_valid && (hrt_elapsed_time(&_airspeed_last_valid) > 1_s)) {
			airspeed_valid = false;
		}
	}
    _airspeed_valid = airspeed_valid;
}

void
FixedwingPositionINDIControl::airflow_aoa_poll()
{
	bool aoa_valid = _aoa_valid;
	airflow_aoa_s aoa_validated;

	if (_airflow_aoa_sub.update(&aoa_validated)) {

		if (PX4_ISFINITE(aoa_validated.aoa_rad)
		    && (aoa_validated.valid)) {

			aoa_valid = true;

			_aoa_last_valid = aoa_validated.timestamp;
			_aoa = aoa_validated.aoa_rad;
		}

	} else {
		// no aoa updates for one second
		if (aoa_valid && (hrt_elapsed_time(&_aoa_last_valid) > 1_s)) {
			aoa_valid = false;
		}
	}
    _aoa_valid = aoa_valid;
}

void
FixedwingPositionINDIControl::airflow_slip_poll()
{
	bool slip_valid = _slip_valid;
	airflow_slip_s slip_validated;

	if (_airflow_slip_sub.update(&slip_validated)) {

		if (PX4_ISFINITE(slip_validated.slip_rad)
		    && (slip_validated.valid)) {

			slip_valid = true;

			_slip_last_valid = slip_validated.timestamp;
			_slip = slip_validated.slip_rad;
		}

	} else {
		// no aoa updates for one second
		if (slip_valid && (hrt_elapsed_time(&_slip_last_valid) > 1_s)) {
			slip_valid = false;
		}
	}
    _slip_valid = slip_valid;
}

void
FixedwingPositionINDIControl::vehicle_status_poll()
{
    if(_vehicle_status_sub.update(&_vehicle_status)){
        //print_message(_vehicle_status);
    }
}

void
FixedwingPositionINDIControl::vehicle_attitude_poll()
{
	if (_vehicle_attitude_sub.update(&_attitude)) {
        // get rotation between NED frames
        Dcmf R_ned_frd(Quatf(_attitude.q));
        // get rotation from FRD to ENU frame (change of basis)
        Dcmf R_enu_frd(_R_ned_to_enu*R_ned_frd);
		_att = Quatf(R_enu_frd);
        //Eulerf e(R_ned_frd);
        //PX4_INFO("attitude euler angles:\t%.4f\t%.4f\t%.4f", (double)e(0),(double)e(1),(double)e(2));
        //PX4_INFO("attitude quaternion:\t%.4f\t%.4f\t%.4f\t%.4f", (double)_att(0),(double)_att(1),(double)_att(2),(double)_att(3));
    }
    if(hrt_absolute_time()-_attitude.timestamp > 50_ms && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD){
        PX4_ERR("attitude sample is too old");
    }
}

void
FixedwingPositionINDIControl::vehicle_angular_velocity_poll()
{   //
    // no need to check if it was updated as the main loop is fired based on an update...
    //
    //PX4_INFO("angular velocity:\t%.4f\t%.4f\t%.4f", (double)_omega(0),(double)_omega(1),(double)_omega(2));
    _omega = Vector3f(_angular_vel.xyz);
    if(hrt_absolute_time()-_angular_vel.timestamp > 50_ms && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD){
        PX4_ERR("angular velocity sample is too old");
    }
}

void
FixedwingPositionINDIControl::vehicle_angular_acceleration_poll()
{
	if (_vehicle_angular_acceleration_sub.update(&_angular_accel)) {
		_alpha = Vector3f(_angular_accel.xyz);
        //PX4_INFO("angular accel:\t%.4f\t%.4f\t%.4f", (double)_alpha(0),(double)_alpha(1),(double)_alpha(2));
    }
    if(hrt_absolute_time()-_angular_accel.timestamp > 50_ms && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD){
        PX4_ERR("angular acceleration sample is too old");
    }
}

void
FixedwingPositionINDIControl::vehicle_local_position_poll()
{
    //vehicle_local_position_s pos;
    if (_vehicle_local_position_sub.update(&_local_pos)){
		_pos = _R_ned_to_enu*Vector3f{_local_pos.x,_local_pos.y,_local_pos.z};
        _vel = _R_ned_to_enu*Vector3f{_local_pos.vx,_local_pos.vy,_local_pos.vz};
        _acc = _R_ned_to_enu*Vector3f{_local_pos.ax,_local_pos.ay,_local_pos.az};
        //PX4_INFO("local position:\t%.4f\t%.4f\t%.4f", (double)_pos(0),(double)_pos(1),(double)_pos(2));
        //PX4_INFO("local velocity:\t%.4f\t%.4f\t%.4f", (double)_vel(0),(double)_vel(1),(double)_vel(2));
        //PX4_INFO("local acceleration:\t%.4f\t%.4f\t%.4f", (double)_acc(0),(double)_acc(1),(double)_acc(2));
    }
    if(hrt_absolute_time()-_local_pos.timestamp > 50_ms && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD){
        PX4_ERR("local position sample is too old");
    }
}

void
FixedwingPositionINDIControl::soaring_controller_status_poll()
{
    if (_soaring_controller_status_sub.update(&_soaring_controller_status)){
        if (!_soaring_controller_status.soaring_controller_running){
            //PX4_INFO("Soaring controller turned off");
        }
        if (_soaring_controller_status.timeout_detected){
            //PX4_INFO("Controller timeout detected");
        }
    }
}

void
FixedwingPositionINDIControl::_set_wind_estimate(Vector3f wind)
{
    _wind_estimate = wind;
    return;
}

void
FixedwingPositionINDIControl::_read_trajectory_coeffs_csv()
{
        /*
        std::ifstream file_reader;
        std::string line;
        std::vector<float> vars;
        file_reader.open(model_input_mean_path_, std::ifstream::in);
        for (int i = 0; i < _num_basis_funs; i++){
            file_reader >> line;
            _basis_coeffs_x(i) = std::stof(line);
        }
        file_reader.close();
        */
    /*   
    // 30 m/s trajectory
    _basis_coeffs_x(0) = -0.001788f;
    _basis_coeffs_x(1) = -4309.983870f;
    _basis_coeffs_x(2) = 27561.833442f;
    _basis_coeffs_x(3) = -79243.452679f;
    _basis_coeffs_x(4) = 132885.860293f;
    _basis_coeffs_x(5) = -132032.442095f;
    _basis_coeffs_x(6) = 48834.507728f;
    _basis_coeffs_x(7) = 67523.410338f;
    _basis_coeffs_x(8) = -123589.704954f;
    _basis_coeffs_x(9) = 71024.161695f;
    _basis_coeffs_x(10) = 44098.153846f;
    _basis_coeffs_x(11) = -128538.025554f;
    _basis_coeffs_x(12) = 131172.181978f;
    _basis_coeffs_x(13) = -78230.807474f;
    _basis_coeffs_x(14) = 26663.839579f;
    _basis_coeffs_x(15) = -3913.262861f;

    _basis_coeffs_y(0) = 0.004280f;
    _basis_coeffs_y(1) = 4570.380553f;
    _basis_coeffs_y(2) = -25639.293350f;
    _basis_coeffs_y(3) = 72621.110788f;
    _basis_coeffs_y(4) = -127949.875874f;
    _basis_coeffs_y(5) = 141930.443715f;
    _basis_coeffs_y(6) = -74162.126946f;
    _basis_coeffs_y(7) = -49777.387350f;
    _basis_coeffs_y(8) = 138193.241851f;
    _basis_coeffs_y(9) = -110424.181996f;
    _basis_coeffs_y(10) = -20559.874806f;
    _basis_coeffs_y(11) = 153605.364533f;
    _basis_coeffs_y(12) = -194515.736549f;
    _basis_coeffs_y(13) = 139509.473902f;
    _basis_coeffs_y(14) = -58671.117200f;
    _basis_coeffs_y(15) = 11343.451391f;

    _basis_coeffs_z(0) = 100.f; //3.499054f;
    _basis_coeffs_z(1) = -9110.823700f;
    _basis_coeffs_z(2) = 49921.539080f;
    _basis_coeffs_z(3) = -131589.929685f;
    _basis_coeffs_z(4) = 211653.192338f;
    _basis_coeffs_z(5) = -208769.604426f;
    _basis_coeffs_z(6) = 82855.943814f;
    _basis_coeffs_z(7) = 97934.653205f;
    _basis_coeffs_z(8) =  -195851.952006f;
    _basis_coeffs_z(9) =  127689.165562f;
    _basis_coeffs_z(10) =   53559.445863f;
    _basis_coeffs_z(11) =   -208279.480139f;
    _basis_coeffs_z(12) =  238865.680043f;
    _basis_coeffs_z(13) = -162710.396962f;
    _basis_coeffs_z(14) =  66968.079887f;
    _basis_coeffs_z(15) = -13187.455563;
    */

   // 100m radius circle trajec
    _basis_coeffs_x(0) = -0.000064f;
    _basis_coeffs_x(1) = -3020.233571f;
    _basis_coeffs_x(2) = 10609.960177f;
    _basis_coeffs_x(3) = -17956.458964f;
    _basis_coeffs_x(4) = 15735.479961f;
    _basis_coeffs_x(5) = -2399.573434f;
    _basis_coeffs_x(6) = -11421.854705f;
    _basis_coeffs_x(7) = 12388.936542f;
    _basis_coeffs_x(8) = 120.944433f;
    _basis_coeffs_x(9) = -12530.869640f;
    _basis_coeffs_x(10) = 11346.431128f;
    _basis_coeffs_x(11) = 2643.369342f;
    _basis_coeffs_x(12) = -15999.009519f;
    _basis_coeffs_x(13) = 18127.094775f;
    _basis_coeffs_x(14) = -10676.696033f;
    _basis_coeffs_x(15) = 3032.667571f;

    _basis_coeffs_y(0) = -100.005984f;
    _basis_coeffs_y(1) = -4686.100637f;
    _basis_coeffs_y(2) = 21963.998713f;
    _basis_coeffs_y(3) = -50566.542718f;
    _basis_coeffs_y(4) = 71908.811359f;
    _basis_coeffs_y(5) = -61683.065460f;
    _basis_coeffs_y(6) = 15730.546677f;
    _basis_coeffs_y(7) = 39386.062413f;
    _basis_coeffs_y(8) = -63952.599923f;
    _basis_coeffs_y(9) = 39525.510553f;
    _basis_coeffs_y(10) = 15526.730604f;
    _basis_coeffs_y(11) = -61505.752706f;
    _basis_coeffs_y(12) = 71804.582542f;
    _basis_coeffs_y(13) = -50525.803330f;
    _basis_coeffs_y(14) = 21954.858741f;
    _basis_coeffs_y(15) = -4685.311429f;

    _basis_coeffs_z(0) = 100.0f;
    _basis_coeffs_z(1) = 0.0f;
    _basis_coeffs_z(2) = 0.0f;
    _basis_coeffs_z(3) = 0.0f;
    _basis_coeffs_z(4) = 0.0f;
    _basis_coeffs_z(5) = 0.0f;
    _basis_coeffs_z(6) = 0.0f;
    _basis_coeffs_z(7) = 0.0f;
    _basis_coeffs_z(8) = 0.0f;
    _basis_coeffs_z(9) = 0.0f;
    _basis_coeffs_z(10) = 0.0f;
    _basis_coeffs_z(11) = 0.0f;
    _basis_coeffs_z(12) = 0.0f;
    _basis_coeffs_z(13) = 0.0f;
    _basis_coeffs_z(14) = 0.0f;
    _basis_coeffs_z(15) = 0.0f;
}

void
FixedwingPositionINDIControl::Run()
{
    // only run controller if pos, vel, acc changed
    perf_begin(_loop_perf);

	if (_vehicle_angular_velocity_sub.update(&_angular_vel))
    {   
        // only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}
		//const float dt = math::constrain((pos.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
		_last_run = _local_pos.timestamp;

        // run polls
        _set_wind_estimate(Vector3f(0.f,0.f,0.f));
        vehicle_status_poll();
        airspeed_poll();
        airflow_aoa_poll();
        airflow_slip_poll();

        vehicle_local_position_poll();
        vehicle_attitude_poll();
        vehicle_angular_velocity_poll();
        vehicle_angular_acceleration_poll();
        soaring_controller_status_poll();

        // ============================
        // compute reference kinematics
        // ============================
        // get reference values
        float t_ref = _get_closest_t(_pos);
        // downscale velocity to match current one, 
        // terminal time is determined such that current velocity is met
        Vector3f v_ref_ = _get_velocity_ref(t_ref, 1.f);
        float T = sqrtf((v_ref_*v_ref_)/(_vel*_vel+0.001f));
        //PX4_INFO("local velocity:\t%.4f\t%.4f\t%.4f", (double)v_ref_(0),(double)v_ref_(1),(double)v_ref_(2));
        //PX4_INFO("T= \t%.1f", (double)T);
        Vector3f pos_ref = _get_position_ref(t_ref);                    // in inertial ENU
        Vector3f vel_ref = _get_velocity_ref(t_ref,T);                  // in inertial ENU
        Vector3f acc_ref = _get_acceleration_ref(t_ref,T);              // gravity-corrected acceleration (ENU)
        Quatf q = _get_attitude_ref(t_ref,T);
        Vector3f omega_ref = _get_angular_velocity_ref(t_ref,T);        // body angular velocity
        Vector3f alpha_ref = _get_angular_acceleration_ref(t_ref,T);    // body angular acceleration
        //PX4_INFO("local position ref:\t%.4f\t%.4f\t%.4f", (double)pos_ref(0),(double)pos_ref(1),(double)pos_ref(2));
        //PX4_INFO("alpha ref:\t%.4f\t%.4f\t%.4f", (double)alpha_ref(0),(double)alpha_ref(1),(double)alpha_ref(2));
        //PX4_INFO("vel ref:\t%.4f\t%.4f\t%.4f", (double)vel_ref(0),(double)vel_ref(1),(double)vel_ref(2));
        //PX4_INFO("vel:\t%.4f\t%.4f\t%.4f", (double)_vel(0),(double)_vel(1),(double)_vel(2));



        // =====================
        // compute control input
        // =====================
        Vector3f ctrl = _compute_NDI_stage_1(pos_ref, vel_ref, acc_ref, omega_ref, alpha_ref);

        // =================================
        // publish offboard control commands
        // =================================
        offboard_control_mode_s ocm{};
        ocm.actuator = true;
        ocm.timestamp = hrt_absolute_time();
        _offboard_control_mode_pub.publish(ocm);

        // Publish actuator controls only once in OFFBOARD
		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {

            // =====================
            // publish control input
            // =====================
            //_angular_accel_sp = {}; 
            _angular_accel_sp.timestamp = hrt_absolute_time();
            _angular_accel_sp.timestamp_sample = hrt_absolute_time();
            _angular_accel_sp.xyz[0] = ctrl(0);
            _angular_accel_sp.xyz[1] = ctrl(1);
            _angular_accel_sp.xyz[2] = ctrl(2);
            _angular_accel_sp_pub.publish(_angular_accel_sp);
            //print_message(_angular_accel_sp);

            // =========================
            // publish attitude setpoint
            // =========================
            //_attitude_sp = {};
            Quatf q_sp(_R_enu_to_ned*Dcmf(q));
            _attitude_sp.timestamp = hrt_absolute_time();
            _attitude_sp.q_d[0] = q_sp(0);
            _attitude_sp.q_d[1] = q_sp(1);
            _attitude_sp.q_d[2] = q_sp(2);
            _attitude_sp.q_d[3] = q_sp(3);
            _attitude_sp_pub.publish(_attitude_sp);

            // ======================
            // publish rates setpoint
            // ======================
            //_angular_vel_sp = {};
            _angular_vel_sp.timestamp = hrt_absolute_time();
            _angular_vel_sp.roll = omega_ref(0);
            _angular_vel_sp.pitch = omega_ref(1);
            _angular_vel_sp.yaw = omega_ref(2);
            _angular_vel_sp_pub.publish(_angular_vel_sp);

            // ============================
            // compute actuator deflections
            // ============================
            Vector3f ctrl1 = _compute_NDI_stage_2(ctrl);
            Vector3f ctrl2 = _compute_actuator_deflections(ctrl1);
            
            // =========================
            // publish acutator controls
            // =========================
            //_actuators = {};
            _actuators.timestamp = hrt_absolute_time();
            _actuators.timestamp_sample = hrt_absolute_time();
            _actuators.control[actuator_controls_s::INDEX_ROLL] = ctrl2(0);
            _actuators.control[actuator_controls_s::INDEX_PITCH] = ctrl2(1);
            _actuators.control[actuator_controls_s::INDEX_YAW] = ctrl2(2);
            _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.3f;
            _actuators_0_pub.publish(_actuators);
            //print_message(_actuators);
        }

        // ===========================
        // publish rate control status
        // ===========================
        rate_ctrl_status_s rate_ctrl_status{};
        rate_ctrl_status.timestamp = hrt_absolute_time();
        rate_ctrl_status.rollspeed_integ = 0.0f;
        rate_ctrl_status.pitchspeed_integ = 0.0f;
        rate_ctrl_status.yawspeed_integ = 0.0f;
        _rate_ctrl_status_pub.publish(rate_ctrl_status);

        // ==============================
        // publish soaring control status
        // ==============================
        //_soaring_controller_heartbeat_s _soaring_controller_heartbeat{};
        _soaring_controller_heartbeat.timestamp = hrt_absolute_time();
        _soaring_controller_heartbeat.heartbeat = hrt_absolute_time();
        _soaring_controller_heartbeat_pub.publish(_soaring_controller_heartbeat);
            
    }
    perf_end(_loop_perf);
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_basis_funs(float t)
{
    Vector<float, _num_basis_funs> vec;
    vec(0) = 1.0f;
    float sigma = 1.0f/float(_num_basis_funs);
    for(uint i=1; i<_num_basis_funs; i++){
        float fun1 = sinf(M_PI_F*t);
        float fun2 = exp(-powf((t-float(i)/float(_num_basis_funs)),2)/sigma);
        vec(i) = fun1*fun2;
    }
    return vec;
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_d_dt_basis_funs(float t)
{
    Vector<float, _num_basis_funs> vec;
    vec(0) = 0.0f;
    float sigma = 1.0f/_num_basis_funs;
    for(uint i=1; i<_num_basis_funs; i++){
        float fun1 = sinf(M_PI_F*t);
        float fun2 = exp(-powf((t-float(i)/_num_basis_funs),2)/sigma);
        vec(i) = fun2*(M_PI_F*sigma*cosf(M_PI_F*t)-2*(t-float(i)/_num_basis_funs)*fun1)/sigma;
    }
    return vec;
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_d2_dt2_basis_funs(float t)
{
    Vector<float, _num_basis_funs> vec;
    vec(0) = 0.0f;
    float sigma = 1.0f/_num_basis_funs;
    for(uint i=1; i<_num_basis_funs; i++){
        float fun1 = sinf(M_PI_F*t);
        float fun2 = exp(-powf((t-float(i)/_num_basis_funs),2)/sigma);
        vec(i) = fun2 * (fun1 * (4*powf((float(i)/_num_basis_funs-t),2) - \
                        sigma*(powf(M_PI_F,2)*sigma + 2)) + 4*M_PI_F*sigma*(float(i)/_num_basis_funs-t)*cosf(M_PI_F*t))/(powf(sigma,2));
 
    }
    return vec;
}

Vector3f
FixedwingPositionINDIControl::_get_position_ref(float t)
{
    Vector<float, _num_basis_funs> basis = _get_basis_funs(t);
    float x = _basis_coeffs_x*basis;
    float y = _basis_coeffs_y*basis;
    float z = _basis_coeffs_z*basis;
    return Vector3f{x, y, z};
}

Vector3f
FixedwingPositionINDIControl::_get_velocity_ref(float t, float T)
{
    Vector<float, _num_basis_funs> basis = _get_d_dt_basis_funs(t);
    float x = _basis_coeffs_x*basis;
    float y = _basis_coeffs_y*basis;
    float z = _basis_coeffs_z*basis;
    return Vector3f{x, y, z}/T;
}

Vector3f
FixedwingPositionINDIControl::_get_acceleration_ref(float t, float T)
{
    Vector<float, _num_basis_funs> basis = _get_d2_dt2_basis_funs(t);
    float x = _basis_coeffs_x*basis;
    float y = _basis_coeffs_y*basis;
    float z = _basis_coeffs_z*basis;
    return Vector3f{x, y, z}/powf(T,2);
}

Quatf
FixedwingPositionINDIControl::_get_attitude_ref(float t, float T)
{
    Vector3f vel = _get_velocity_ref(t,T);
    Vector3f vel_air = vel - _wind_estimate;
    Vector3f acc = _get_acceleration_ref(t,T);
    // add gravity
    acc(2) += 9.81f;
    // compute required force
    Vector3f f = _mass*acc;
    // compute force component projected onto lift axis
    Vector3f vel_normalized = vel_air.normalized();
    Vector3f f_lift = f - (f*vel_normalized)*vel_normalized;
    Vector3f lift_normalized = f_lift.normalized();
    Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
    // compute rotation matrix between ENU and FRD frame
    Dcmf R_bi;
    R_bi(0,0) = vel_normalized(0);
    R_bi(0,1) = vel_normalized(1);
    R_bi(0,2) = vel_normalized(2);
    R_bi(1,0) = wing_normalized(0);
    R_bi(1,1) = wing_normalized(1);
    R_bi(1,2) = wing_normalized(2);
    R_bi(2,0) = lift_normalized(0);
    R_bi(2,1) = lift_normalized(1);
    R_bi(2,2) = lift_normalized(2);
    R_bi.renormalize();
    // compute required AoA
    Vector3f f_phi = R_bi*f_lift;
    float AoA = ((2.f*f_phi(2))/(_rho*_area*(vel_air*vel_air)+0.001f) - _C_L0)/_C_L1;
    // compute final rotation matrix
    Eulerf e(0.f, AoA, 0.f);
    Dcmf R_pitch(e);
    Dcmf Rotation(R_pitch*R_bi);
    // switch from FRD to ENU frame
    Rotation(1,0) *= -1;
    Rotation(1,1) *= -1;
    Rotation(1,2) *= -1;
    Rotation(2,0) *= -1;
    Rotation(2,1) *= -1;
    Rotation(2,2) *= -1;
    /*
    float determinant = Rotation(0,0)*(Rotation(1,1)*Rotation(2,2)-Rotation(2,1)*Rotation(1,2)) - 
                        Rotation(1,0)*(Rotation(0,1)*Rotation(2,2)-Rotation(2,1)*Rotation(0,2)) + 
                        Rotation(2,0)*(Rotation(0,1)*Rotation(1,2)-Rotation(1,1)*Rotation(0,2));
    PX4_INFO("determinant: %.2f", (double)determinant);
    PX4_INFO("length: %.2f", (double)(wing_normalized*wing_normalized));
    */



    Quatf q(Rotation.transpose());
    return q;
}

Vector3f
FixedwingPositionINDIControl::_get_angular_velocity_ref(float t, float T)
{
    float dt = 0.001;
    float t_lower = fmaxf(0.f,t-dt);
    float t_upper = fminf(t+dt,1.f);
    Dcmf R_i0(_get_attitude_ref(t_lower, T));
    Dcmf R_i1(_get_attitude_ref(t_upper, T));
    Dcmf R_10 = R_i1.transpose()*R_i0;
    AxisAnglef w_01(R_10);
    return -w_01.axis()*w_01.angle()/(T*(t_upper-t_lower));
}

Vector3f
FixedwingPositionINDIControl::_get_angular_acceleration_ref(float t, float T)
{
    float dt = 0.001;
    float t_lower = fmaxf(0.f,t-dt);
    float t_upper = fminf(t+dt,1.f);
    // compute roational velocity in inertial frame
    Dcmf R_i0(_get_attitude_ref(t_lower, T));
    AxisAnglef w_0(R_i0*_get_angular_velocity_ref(t_lower, T));
    // compute roational velocity in inertial frame
    Dcmf R_i1(_get_attitude_ref(t_upper, T));
    AxisAnglef w_1(R_i1*_get_angular_velocity_ref(t_upper, T));
    // compute gradient via finite differences
    Vector3f dw_dt = (w_1.axis()*w_1.angle() - w_0.axis()*w_0.angle()) / (T*(t_upper-t_lower));
    // transform back to body frame
    return R_i0.transpose()*dw_dt;
}

float
FixedwingPositionINDIControl::_get_closest_t(Vector3f pos)
{
    const uint n = 500;
    Vector<float, n> distances;
    float t_ref;
    // compute all distances
    for(uint i=0; i<n; i++){
        t_ref = float(i)/float(n);
        Vector3f pos_ref = _get_position_ref(t_ref);
        //PX4_INFO("trajectory time + point: \t%.2f\t%.2f\t%.2f\t%.2f", (double)t_ref, (double)pos_ref(0), (double)pos_ref(1), (double)pos_ref(2));
        distances(i) = (pos_ref - pos)*(pos_ref - pos);
    }

    // get index of smallest distance
    float t = 0.f;
    float min_dist = distances(0);
    for(uint i=1; i<n; i++){
        if(distances(i)<min_dist){
            min_dist = distances(i);
            t = float(i);
        }
    }
    t = t/float(n);
    //PX4_INFO("closest point: \t%.2f\t%.2f\t%.2f", (double)_get_position_ref(t)(0), (double)_get_position_ref(t)(1), (double)_get_position_ref(t)(2));
    //PX4_INFO("closest t: %.2f", (double)t);
    //PX4_INFO("closest distance:%.2f", (double)sqrtf(min_dist));
    return t;
}

Quatf
FixedwingPositionINDIControl::_get_attitude(Vector3f vel, Vector3f f)
{
    Vector3f vel_air = vel - _wind_estimate;
    // catch case, where the aircraft needs to accelerate downwards
    // only become inverted, if we need to accelerate a lot:
    /*
    Vector3f f_body = Dcmf(_att).transpose()*f;
    bool inverted = false;
    if(f_body(2)>9.81*_mass){
        inverted = true;
    }
    */
    // compute force component projected onto lift axis
    Vector3f vel_normalized = vel_air.normalized();
    Vector3f f_lift = f - (f*vel_normalized)*vel_normalized;
    Vector3f lift_normalized = f_lift.normalized();
    Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
    // compute rotation matrix between ENU and FRD frame
    Dcmf R_bi;
    R_bi(0,0) = vel_normalized(0);
    R_bi(0,1) = vel_normalized(1);
    R_bi(0,2) = vel_normalized(2);
    R_bi(1,0) = wing_normalized(0);
    R_bi(1,1) = wing_normalized(1);
    R_bi(1,2) = wing_normalized(2);
    R_bi(2,0) = lift_normalized(0);
    R_bi(2,1) = lift_normalized(1);
    R_bi(2,2) = lift_normalized(2);
    R_bi.renormalize();
    // compute required AoA
    Vector3f f_phi = R_bi*f_lift;
    float AoA = ((2.f*f_phi(2))/(_rho*_area*(vel_air*vel_air)+0.001f) - _C_L0)/_C_L1;
    // compute final rotation matrix
    Eulerf e(0.f, AoA, 0.f);
    Dcmf R_pitch(e);
    Dcmf Rotation(R_pitch*R_bi);
    // switch from FRD to ENU frame
    Rotation(1,0) *= -1;
    Rotation(1,1) *= -1;
    Rotation(1,2) *= -1;
    Rotation(2,0) *= -1;
    Rotation(2,1) *= -1;
    Rotation(2,2) *= -1;

    Quatf q(Rotation.transpose());
    return q;
}

Vector3f
FixedwingPositionINDIControl::_compute_NDI_stage_1(Vector3f pos_ref, Vector3f vel_ref, Vector3f acc_ref, Vector3f omega_ref, Vector3f alpha_ref)
{
    Dcmf R_ib(_att);
    Dcmf R_bi(R_ib.transpose());
    // get acceleration command in world frame (without gravity)
    Vector3f acc_command = R_ib*(_K_x*R_bi*(pos_ref-_pos) + _K_v*R_bi*(vel_ref-_vel) + _K_a*R_bi*(acc_ref-_acc)) + acc_ref;
    // add gravity
    acc_command(2) += 9.81f;
    // get force comand in world frame
    Vector3f f_command = _mass*acc_command;
    // get required attitude (assuming we can fly the target velocity)
    Dcmf R_ref(_get_attitude(vel_ref,f_command));
    // get attitude error
    Dcmf R_ref_true(R_ref.transpose()*R_ib);
    // get required rotation vector (in body frame)
    AxisAnglef q_err(R_ref_true);
    Vector3f w_err = -q_err.angle()*q_err.axis();
    PX4_INFO("force command: \t%.2f\t%.2f\t%.2f", (double)f_command(0), (double)f_command(1), (double)f_command(2));
    //PX4_INFO("FRD body frame rotation vec: \t%.2f\t%.2f\t%.2f", (double)w_err(0), (double)w_err(1), (double)w_err(2));
    // compute angular acceleration command (in body frame)
    Vector3f rot_acc_command = _K_q*w_err + _K_w*(omega_ref-_omega) + alpha_ref;
    rot_acc_command =  1.0f*_K_q*w_err + 1.0f*_K_w*(omega_ref-_omega) + alpha_ref;;

    // apply LP filtered values for incremental part

    
    return rot_acc_command;
}

Vector3f 
FixedwingPositionINDIControl::_compute_NDI_stage_2(Vector3f ctrl)
{

    // compute the expected current body moment
    Vector3f moment = _inertia*_alpha + _omega.cross(_inertia*_omega);
    moment = 1.f*Vector3f{1.0f*_actuators.control[actuator_controls_s::INDEX_ROLL], 1.f*_actuators.control[actuator_controls_s::INDEX_PITCH], 0.1f*_actuators.control[actuator_controls_s::INDEX_YAW]};
    float c_ail = 1.f/400.f;
    float c_ele = 1.f/400.f;
    float c_rud = 0.1f/400.f;
    // compute velocity in body frame
    Dcmf R_ib(_att);
    Vector3f vel_body = R_ib.transpose()*_vel;
    //Vector3f vel_body_2 = Dcmf(Quatf(_attitude.q)).transpose()*Vector3f{_local_pos.vx,_local_pos.vy,_local_pos.vz};
    //PX4_INFO("ENU body frame velocity: \t%.2f\t%.2f\t%.2f", (double)vel_body_2(0), (double)vel_body_2(1), (double)vel_body_2(2));
    //PX4_INFO("FRD body frame velocity: \t%.2f\t%.2f\t%.2f", (double)vel_body(0), (double)vel_body(1), (double)vel_body(2));
    // compute moments
    moment(0) = 0.5f*c_ail*sqrtf(vel_body*vel_body)*vel_body(0)*_actuators.control[actuator_controls_s::INDEX_ROLL];
    moment(1) = 0.5f*c_ele*sqrtf(vel_body*vel_body)*vel_body(0)*_actuators.control[actuator_controls_s::INDEX_PITCH];
    moment(0) = 0.5f*c_rud*sqrtf(vel_body*vel_body)*vel_body(0)*_actuators.control[actuator_controls_s::INDEX_YAW];
    Vector3f moment_filtered = _apply_LP_filter(moment, _m_list, _m_lpf_list);
    Vector3f alpha_filtered = _apply_LP_filter(_alpha, _l_list, _l_lpf_list);
    Vector3f command = _inertia*(ctrl-alpha_filtered) + moment_filtered;
    //PX4_INFO("filtered alpha: \t%.2f\t%.2f", (double)(_l_list(0))(2), (double)(_l_lpf_list(0))(1));
    //command = _inertia*ctrl + _omega.cross(_inertia*_omega);
    return 0.f*command + 1.f*(ctrl);


}

Vector3f
FixedwingPositionINDIControl::_apply_LP_filter(Vector3f new_input, Vector<Vector3f, 3>  &old_input, Vector<Vector3f, 2>  &old_output)
{
    old_input(0) = old_input(1);
    old_input(1) = old_input(2);
    old_input(2) = new_input;
    //
    Vector3f output = Vector3f{0.f,0.f,0.f};
    //
    output += _a1*old_output(1);
    output += _a2*old_output(0);
    //
    output += _b1*old_input(2);
    output += _b2*old_input(1);
    output += _b3*old_input(0);
    //
    old_output(0) = old_output(1);
    old_output(1) = output;
    //
    return output;
}

Vector3f
FixedwingPositionINDIControl::_compute_actuator_deflections(Vector3f ctrl)
{   
    // compute airspeed scaling
    const float airspeed_constrained = constrain(_airspeed, 5.f, 50.f);
    float airspeed_scaling = 20.f/(powf(airspeed_constrained,2)+1.f);
    airspeed_scaling = 1.0f;

    // compute the normalized actuator deflection, including airspeed scaling
    Vector3f deflection = airspeed_scaling*_K_actuators*ctrl;

    // limit actuator deflection
    for(int i=0; i<3; i++){
        deflection(i) = constrain(deflection(i),-1.f,1.f);
    }
    return deflection;
}



int FixedwingPositionINDIControl::task_spawn(int argc, char *argv[])
{
	FixedwingPositionINDIControl *instance = new FixedwingPositionINDIControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
FixedwingPositionINDIControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle position callback registration failed!");
		return false;
	}
    _read_trajectory_coeffs_csv();

    // initialize transformations
    _R_ned_to_enu *= 0.f;
	_R_ned_to_enu(0,1) = 1.f;
	_R_ned_to_enu(1,0) = 1.f;
	_R_ned_to_enu(2,2) = -1.f;
	_R_ned_to_enu.renormalize();
    _R_enu_to_ned = _R_ned_to_enu;
	return true;
}

int FixedwingPositionINDIControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingPositionINDIControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_dyn_soar_control is the fixed wing controller for soaring tasks.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_dyn_soar_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_dyn_soar_control_main(int argc, char *argv[])
{
	return FixedwingPositionINDIControl::main(argc, argv);
}
