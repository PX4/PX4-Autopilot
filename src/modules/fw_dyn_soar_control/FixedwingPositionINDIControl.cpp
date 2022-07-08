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
	// limit to 100 Hz
	_vehicle_angular_velocity_sub.set_interval_ms(1000.f/_sample_frequency);


	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionINDIControl::~FixedwingPositionINDIControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingPositionINDIControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle position callback registration failed!");
		return false;
	}
    PX4_INFO("Starting FW_DYN_SOAR_CONTROLLER");
    //_read_trajectory_coeffs_csv("trajectory5.csv");
    //_read_trajectory_coeffs_csv("trajectory4.csv");
    //_read_trajectory_coeffs_csv("trajectory3.csv");
    //_read_trajectory_coeffs_csv("trajectory2.csv");
    //_read_trajectory_coeffs_csv("trajectory1.csv");
    char filename[] = "trajectory0.csv";
    _read_trajectory_coeffs_csv(filename);

    // initialize transformations
    _R_ned_to_enu *= 0.f;
	_R_ned_to_enu(0,1) = 1.f;
	_R_ned_to_enu(1,0) = 1.f;
	_R_ned_to_enu(2,2) = -1.f;
	_R_ned_to_enu.renormalize();
    _R_enu_to_ned = _R_ned_to_enu;

	return true;
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
    _K_x(0,0) = _param_lin_k_x.get();
    _K_x(1,1) = _param_lin_k_y.get();
    _K_x(2,2) = _param_lin_k_z.get();
    _K_v(0,0) = _param_lin_c_x.get()*2.f*sqrtf(_param_lin_k_x.get());
    _K_v(1,1) = _param_lin_c_y.get()*2.f*sqrtf(_param_lin_k_y.get());
    _K_v(2,2) = _param_lin_c_z.get()*2.f*sqrtf(_param_lin_k_z.get());
    _K_a(0,0) = _param_lin_ff_x.get();
    _K_a(1,1) = _param_lin_ff_y.get();
    _K_a(2,2) = _param_lin_ff_z.get();
    _K_q(0,0) = _param_rot_k_roll.get();
    _K_q(1,1) = _param_rot_k_pitch.get();
    _K_q(2,2) = _param_rot_k_yaw.get(); // rudder is controlled via turn coordination, not INDI
    _K_w(0,0) = _param_rot_c_roll.get()*2.f*sqrtf(_param_rot_k_roll.get());
    _K_w(1,1) = _param_rot_c_pitch.get()*2.f*sqrtf(_param_rot_k_pitch.get());
    _K_w(2,2) = _param_rot_c_yaw.get()*2.f*sqrtf(_param_rot_k_yaw.get()); // rudder is controlled via turn coordination, not INDI

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
    _aoa_offset = _param_aoa_offset.get();
    _stall_speed = _param_stall_speed.get();


    // actuator gains
    _k_ail = _param_k_act_roll.get();
    _k_ele = _param_k_act_pitch.get();
    _k_rud = _param_k_act_yaw.get();
    _k_d_roll = _param_k_damping_roll.get();
    _k_d_pitch = _param_k_damping_pitch.get();
    _k_d_yaw = _param_k_damping_yaw.get();

    // trajectory origin
    _origin_lat = _param_origin_lat.get();
    _origin_lon = _param_origin_lon.get();
    _origin_alt = _param_origin_alt.get();
    // check if local NED reference frame origin has changed:
    // || (_local_pos.vxy_reset_counter != _pos_reset_counter
    // initialize projection
    map_projection_init_timestamped(&_global_local_proj_ref, _local_pos.ref_lat, _local_pos.ref_lon,
                    _local_pos.ref_timestamp);
    // project the origin of the soaring ENU frame to the current NED frame
    map_projection_project(&_global_local_proj_ref, _origin_lat, _origin_lon, &_origin_N, &_origin_E);
    _origin_D =  _local_pos.ref_alt - _origin_alt;
    PX4_INFO("local reference frame updated");
    


    _loiter = _param_loiter.get();
    _select_trajectory(0.0f);

    _thrust = _param_thrust.get();

    _switch_manual = _param_switch_manual.get();


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
FixedwingPositionINDIControl::manual_control_setpoint_poll()
{
    if(_switch_manual){
        _manual_control_setpoint_sub.update(&_manual_control_setpoint);
        _thrust = _manual_control_setpoint.z;
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
        // transform to soaring frame
        _pos = _pos - _R_ned_to_enu * Vector3f{_origin_N, _origin_E, _origin_D};

        //PX4_INFO("local position:\t%.4f\t%.4f\t%.4f", (double)_pos(0),(double)_pos(1),(double)_pos(2));
        //PX4_INFO("local velocity:\t%.4f\t%.4f\t%.4f", (double)_vel(0),(double)_vel(1),(double)_vel(2));
        //PX4_INFO("local acceleration:\t%.4f\t%.4f\t%.4f", (double)_acc(0),(double)_acc(1),(double)_acc(2));
    }
    if(hrt_absolute_time()-_local_pos.timestamp > 50_ms && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD){
        PX4_ERR("local position sample is too old");
    }
}

void
FixedwingPositionINDIControl::actuator_controls_poll()
{
    _actuator_controls_sub.update(&_actuators);
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
FixedwingPositionINDIControl::_select_trajectory(float initial_energy)
{
    // select loiter trajectory for loiter test
    char filename[16];
    switch (_loiter)
    {
    case 0:
        strcpy(filename,"trajectory0.csv");
        break;
    case 1:
        strcpy(filename,"trajectory1.csv");
        break;
    case 2:
        strcpy(filename,"trajectory2.csv");
        break;
    case 3:
        strcpy(filename,"trajectory3.csv");
        break;
    case 4:
        strcpy(filename,"trajectory4.csv");
        break;
    case 5:
        strcpy(filename,"trajectory5.csv");
        break;
    case 6:
        strcpy(filename,"trajectory6.csv");
        break;
    case 7:
        strcpy(filename,"trajectory7.csv");
        break;

    default:
        strcpy(filename,"trajectory0.csv");
    }
    

    _read_trajectory_coeffs_csv(filename);
}

void
FixedwingPositionINDIControl::_read_trajectory_coeffs_csv(char *filename)
{

    // =======================================================================
    bool error = false;

    char home_dir[200] = "/home/marvin/Documents/master_thesis_ADS/PX4/Git/ethzasl_fw_px4/src/modules/fw_dyn_soar_control/trajectories/";
    //char home_dir[200] = PX4_ROOTFSDIR"/fs/microsd/trajectories/";
    //PX4_ERR(home_dir);
    strcat(home_dir,filename);
    FILE* fp = fopen(home_dir, "r");

    if (fp == nullptr) {
        PX4_ERR("Can't open file");
        error = true;
    }
    else {
        // Here we have taken size of
        // array 1024 you can modify it
        const uint buffersize = _num_basis_funs*32;
        char buffer[buffersize];

        int row = 0;
        int column = 0;

        // loop over rows
        while (fgets(buffer,
                     buffersize, fp)) {
            column = 0;
 
            // Splitting the data
            char* value = strtok(buffer, ",");
            
            // loop over columns
            while (value) {
                if (*value=='\0'||*value==' ') {
                    // simply skip extra characters
                    continue;
                }
                switch(row){
                    case 0:
                        _basis_coeffs_x(column) = (float)atof(value);
                        break;
                    case 1:
                        _basis_coeffs_y(column) = (float)atof(value);
                        break;
                    case 2:
                        _basis_coeffs_z(column) = (float)atof(value);
                        break;

                    default:
                        break;
                }
                //PX4_INFO("row: %d, col: %d, read value: %.3f", row, column, (double)atof(value));
                value = strtok(NULL, ",");
                column++;
                
            }
            row++;
        }
        int failure = fclose(fp);
        if (failure==-1) {
            PX4_ERR("Can't close file");
        }
    }
    // =======================================================================


    // go back to safety mode loiter circle
    if(error){
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

}

void
FixedwingPositionINDIControl::Run()
{
    if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

    perf_begin(_loop_perf);

    // only run controller if pos, vel, acc changed
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
		//_last_run = _local_pos.timestamp;

        // check if local NED reference frame origin has changed:
        // || (_local_pos.vxy_reset_counter != _pos_reset_counter
        if (!map_projection_initialized(&_global_local_proj_ref)
            || (_global_local_proj_ref.timestamp != _local_pos.ref_timestamp)
		    || (_local_pos.xy_reset_counter != _pos_reset_counter)
            || (_local_pos.z_reset_counter != _alt_reset_counter)) {
            // initialize projection
            map_projection_init_timestamped(&_global_local_proj_ref, _local_pos.ref_lat, _local_pos.ref_lon,
                            _local_pos.ref_timestamp);
            // project the origin of the soaring ENU frame to the current NED frame
            map_projection_project(&_global_local_proj_ref, _origin_lat, _origin_lon, &_origin_N, &_origin_E);
            _origin_D =  _local_pos.ref_alt - _origin_alt;
            PX4_INFO("local reference frame updated");
        }
        // update reset counters
        _pos_reset_counter = _local_pos.xy_reset_counter;
        _alt_reset_counter = _local_pos.z_reset_counter;

        // run polls
        vehicle_status_poll();
        airspeed_poll();
        airflow_aoa_poll();
        airflow_slip_poll();
        manual_control_setpoint_poll();
        vehicle_local_position_poll();
        vehicle_attitude_poll();
        vehicle_angular_velocity_poll();
        vehicle_angular_acceleration_poll();
        soaring_controller_status_poll();

        // ===============================
        // compute wind pseudo-measurement
        // ===============================
        Dcmf R_ib(_att);
        Dcmf R_bi(R_ib.transpose());
        // compute expected AoA from g-forces:
        Vector3f body_force = _mass*R_bi*(_acc + Vector3f{0.f,0.f,9.81f});
        // approximate lift force, since implicit equation cannot be solved analytically:
        // since alpha<<1, we approximate the lift force L = sin(alpha)*Fx - cos(alpha)*Fz
        // as L = alpha*Fx - Fz
        float Fx = body_force(0);
        float Fz = -body_force(2);
        float AoA_approx = (((2.f*Fz)/(_rho*_area*(fmaxf(_airspeed*_airspeed,_stall_speed*_stall_speed))+0.001f) - _C_L0)/_C_L1) / 
                            (1 - ((2.f*Fx)/(_rho*_area*(fmaxf(_airspeed*_airspeed,_stall_speed*_stall_speed))+0.001f)/_C_L1));
        AoA_approx = constrain(AoA_approx,-0.2f,0.2f);
        Vector3f vel_air = R_ib*(Vector3f{_airspeed,0.f,tanf(AoA_approx-_aoa_offset)*_airspeed});
        Vector3f wind = _vel - vel_air;
        wind(0) = _lp_filter_wind[0].apply(wind(0));
        wind(1) = _lp_filter_wind[1].apply(wind(1));
        wind(2) = _lp_filter_wind[2].apply(wind(2));
        _set_wind_estimate(wind);
        //PX4_INFO("wind estimate:\t%.4f\t%.4f\t%.4f", (double)_wind_estimate(0),(double)_wind_estimate(1),(double)_wind_estimate(2));


        // only run actuators poll, when our module is not publishing:
        if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
            actuator_controls_poll();
        }

        // ============================
        // compute reference kinematics
        // ============================
        // get reference values
        float t_ref = _get_closest_t(_pos);
        // downscale velocity to match current one, 
        // terminal time is determined such that current velocity is met
        Vector3f v_ref_ = _get_velocity_ref(t_ref, 1.0f);
        float T = sqrtf((v_ref_*v_ref_)/(_vel*_vel+0.001f));
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
        Vector3f ctrl = _compute_INDI_stage_1(pos_ref, vel_ref, acc_ref, omega_ref, alpha_ref);
        Vector3f ctrl1 = _compute_INDI_stage_2(ctrl);

        // ============================
        // compute actuator deflections
        // ============================
        Vector3f ctrl2 = _compute_actuator_deflections(ctrl1);

        // =================================
        // publish offboard control commands
        // =================================
        offboard_control_mode_s ocm{};
        ocm.actuator = true;
        ocm.timestamp = hrt_absolute_time();
        _offboard_control_mode_pub.publish(ocm);

        // Publish actuator controls only once in OFFBOARD
		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {

            // ========================================
            // publish controller position in ENU frame
            // ========================================
            _soaring_controller_position.timestamp = hrt_absolute_time();
            for (int i=0; i<3; i++){
                _soaring_controller_position.pos[i] = _pos(i);
                _soaring_controller_position.vel[i] = _vel(i);
                _soaring_controller_position.acc[i] = _acc(i);
            }
            _soaring_controller_position_pub.publish(_soaring_controller_position);

            // ====================================
            // publish controller position setpoint
            // ====================================
            _soaring_controller_position_setpoint.timestamp = hrt_absolute_time();
            for (int i=0; i<3; i++){
                _soaring_controller_position_setpoint.pos[i] = pos_ref(i);
                _soaring_controller_position_setpoint.vel[i] = vel_ref(i);
                _soaring_controller_position_setpoint.acc[i] = acc_ref(i);
            }
            _soaring_controller_position_setpoint_pub.publish(_soaring_controller_position_setpoint);

            // =====================
            // publish control input
            // =====================
            //_angular_accel_sp = {}; 
            _angular_accel_sp.timestamp = hrt_absolute_time();
            //_angular_accel_sp.timestamp_sample = hrt_absolute_time();
            _angular_accel_sp.xyz[0] = ctrl(0);
            _angular_accel_sp.xyz[1] = ctrl(1);
            _angular_accel_sp.xyz[2] = ctrl(2);
            _angular_accel_sp_pub.publish(_angular_accel_sp);

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
            
            // =========================
            // publish acutator controls
            // =========================
            //_actuators = {};
            _actuators.timestamp = hrt_absolute_time();
            _actuators.timestamp_sample = hrt_absolute_time();
            _actuators.control[actuator_controls_s::INDEX_ROLL] = ctrl2(0);
            _actuators.control[actuator_controls_s::INDEX_PITCH] = ctrl2(1);
            _actuators.control[actuator_controls_s::INDEX_YAW] = ctrl2(2);
            _actuators.control[actuator_controls_s::INDEX_THROTTLE] = _thrust;
            _actuators_0_pub.publish(_actuators);
            //print_message(_actuators);

            // =====================
            // publish wind estimate
            // =====================
            //_soaring_controller_wind = {};
            _soaring_controller_wind.timestamp = hrt_absolute_time();
            _soaring_controller_wind.wind_estimate[0] = wind(0);
            _soaring_controller_wind.wind_estimate[1] = wind(1);
            _soaring_controller_wind.wind_estimate[2] = wind(2);
            _soaring_controller_wind.wind_estimate_filtered[0] = _wind_estimate(0);
            _soaring_controller_wind.wind_estimate_filtered[1] = _wind_estimate(1);
            _soaring_controller_wind.wind_estimate_filtered[2] = _wind_estimate(2);
            _soaring_controller_wind_pub.publish(_soaring_controller_wind);



            if (_counter==100) {
                _counter = 0;
                //PX4_INFO("frequency: \t%.3f", (double)(1000000*100)/(hrt_absolute_time()-_last_time));
                _last_time = hrt_absolute_time();
            }
            else {
                _counter += 1;
            }
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

        // ====================
        // publish debug values
        // ====================
        Vector3f vel_body = R_bi*_vel;
        _slip = atan2f(vel_body(1), vel_body(0));
        _debug_value.timestamp = hrt_absolute_time();
        _debug_value.value = _slip;
        _debug_value_pub.publish(_debug_value);

        perf_end(_loop_perf);  
    }
    
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_basis_funs(float t)
{
    Vector<float, _num_basis_funs> vec;
    vec(0) = 1.0f;
    float sigma = 1.0f/_num_basis_funs;
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
    float AoA = ((2.f*f_phi(2))/(_rho*_area*(vel_air*vel_air)+0.001f) - _C_L0)/_C_L1 - _aoa_offset;
    // compute final rotation matrix
    Eulerf e(0.f, AoA, 0.f);
    Dcmf R_pitch(e);
    Dcmf Rotation(R_pitch*R_bi);
    // switch from FRD to ENU frame
    Rotation(1,0) *= -1.f;
    Rotation(1,1) *= -1.f;
    Rotation(1,2) *= -1.f;
    Rotation(2,0) *= -1.f;
    Rotation(2,1) *= -1.f;
    Rotation(2,2) *= -1.f;
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
    /*
    const uint n = 100;
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
    */

    //PX4_INFO("closest point: \t%.2f\t%.2f\t%.2f", (double)_get_position_ref(t)(0), (double)_get_position_ref(t)(1), (double)_get_position_ref(t)(2));
    //PX4_INFO("closest t: %.2f", (double)t);
    //PX4_INFO("closest distance:%.2f", (double)sqrtf(min_dist));

    
    const uint n_1 = 20;
    Vector<float, n_1> distances;
    float t_ref;
    // =======
    // STAGE 1
    // =======
    // STAGE 1: compute all distances
    for(uint i=0; i<n_1; i++){
        t_ref = float(i)/float(n_1);
        Vector3f pos_ref = _get_position_ref(t_ref);
        distances(i) = (pos_ref - pos)*(pos_ref - pos);
    }

    // STAGE 1: get index of smallest distance
    float t_1 = 0.f;
    float min_dist = distances(0);
    for(uint i=1; i<n_1; i++){
        if(distances(i)<min_dist){
            min_dist = distances(i);
            t_1 = float(i);
        }
    }
    t_1 = t_1/float(n_1);

    // =======
    // STAGE 2
    // =======
    const uint n_2 = 2*n_1-1;
    Vector<float, n_2+1> distances_2;
    float t_lower = fmod(t_1 - 1.0f/n_1,1.0f);
    // STAGE 2: compute all distances
    for(uint i=0; i<=n_2; i++){
        t_ref = fmod(t_lower + float(i)*2.f/float(n_1*n_2),1.0f);
        Vector3f pos_ref = _get_position_ref(t_ref);
        distances_2(i) = (pos_ref - pos)*(pos_ref - pos);
    }

    // STAGE 2: get index of smallest distance
    float t_2 = 0.f;
    min_dist = distances_2(0);
    for(uint i=1; i<=n_2; i++){
        if(distances_2(i)<min_dist){
            min_dist = distances_2(i);
            t_2 = float(i);
        }
    }
    t_2 = fmod(t_lower + float(t_2)*2.f/(float(n_2)*float(n_1)),1.0f);

    return t_2;
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
    float AoA = ((2.f*f_phi(2))/(_rho*_area*(vel_air*vel_air)+0.001f) - _C_L0)/_C_L1 - _aoa_offset;
    // compute final rotation matrix
    Eulerf e(0.f, AoA, 0.f);
    Dcmf R_pitch(e);
    Dcmf Rotation(R_pitch*R_bi);
    // switch from FRD to ENU frame
    Rotation(1,0) *= -1.f;
    Rotation(1,1) *= -1.f;
    Rotation(1,2) *= -1.f;
    Rotation(2,0) *= -1.f;
    Rotation(2,1) *= -1.f;
    Rotation(2,2) *= -1.f;

    Quatf q(Rotation.transpose());
    return q;
}

Vector3f
FixedwingPositionINDIControl::_compute_INDI_stage_1(Vector3f pos_ref, Vector3f vel_ref, Vector3f acc_ref, Vector3f omega_ref, Vector3f alpha_ref)
{
    Dcmf R_ib(_att);
    Dcmf R_bi(R_ib.transpose());
    // apply LP filter to acceleration
    Vector3f acc_filtered;
    acc_filtered(0) = _lp_filter_accel[0].apply(_acc(0));
    acc_filtered(1) = _lp_filter_accel[1].apply(_acc(1));
    acc_filtered(2) = _lp_filter_accel[2].apply(_acc(2));

    // =========================================
    // apply PD control law on the body position
    // =========================================
    Vector3f acc_command = R_ib*(_K_x*R_bi*(pos_ref-_pos) + _K_v*R_bi*(vel_ref-_vel) + _K_a*R_bi*(acc_ref-acc_filtered)) + acc_ref;
    // add gravity
    acc_command(2) += 9.81f;

    // ==================================
    // compute expected aerodynamic force
    // ==================================
    Vector3f f_current;
    Vector3f vel_body = R_bi*(_vel - _wind_estimate); //TODO: correct to airspeed!!!
    float AoA = atan2f(vel_body(2), vel_body(0)) + _aoa_offset;
    float C_l = _C_L0 + _C_L1*AoA;
    float C_d = _C_D0 + _C_D1*AoA + _C_D2*powf(AoA,2);
    float factor = -0.5f*_rho*_area*sqrtf(vel_body*vel_body);
    Vector3f w_x = vel_body;
    Vector3f w_z = w_x.cross(Vector3f{0.f,1.f,0.f});
    f_current = R_ib*(factor*(C_l*w_z + C_d*w_x)) + Vector3f{0.f,0.f,-_mass*9.81f};
    // apply LP filter to force
    Vector3f f_current_filtered;
    f_current_filtered(0) = _lp_filter_force[0].apply(f_current(0));
    f_current_filtered(1) = _lp_filter_force[1].apply(f_current(1));
    f_current_filtered(2) = _lp_filter_force[2].apply(f_current(2));

    // ===============================
    // get force comand in world frame
    // ===============================
    Vector3f f_command = _mass*(acc_command - acc_filtered) + f_current_filtered;

    // ==========================================================================
    // get required attitude (assuming we can fly the target velocity), and error
    // ==========================================================================
    Dcmf R_ref(_get_attitude(vel_ref,f_command));
    // get attitude error
    Dcmf R_ref_true(R_ref.transpose()*R_ib);
    // get required rotation vector (in body frame)
    AxisAnglef q_err(R_ref_true);
    Vector3f w_err;
    // project rotation angle to [-pi,pi]
    if (q_err.angle()*q_err.angle()<M_PI_F*M_PI_F){
        w_err = -q_err.angle()*q_err.axis();
    }
    else{
        if (q_err.angle()>0.f){
            w_err = (2.f*M_PI_F-(float)fmod(q_err.angle(),2.f*M_PI_F))*q_err.axis();
        }
        else{
            w_err = (-2.f*M_PI_F-(float)fmod(q_err.angle(),2.f*M_PI_F))*q_err.axis();
        }
    }
    
    // =========================================
    // apply PD control law on the body attitude
    // =========================================
    Vector3f omega_filtered;
    omega_filtered(0) = _lp_filter_omega[0].apply(_omega(0));
    omega_filtered(1) = _lp_filter_omega[1].apply(_omega(1));
    omega_filtered(2) = _lp_filter_omega[2].apply(_omega(2));
    Vector3f rot_acc_command = _K_q*w_err + _K_w*(omega_ref-omega_filtered) + alpha_ref;

    // ==========================================
    // input meant for tuning the INDI controller
    // ==========================================
    /*
    if(hrt_absolute_time()%2000000>1000000){
        rot_acc_command = Vector3f{2.0f,1.f,0.f};
        //rot_acc_command = Vector3f{0.f,0.f,0.5f};
    }
    else{
        rot_acc_command = Vector3f{-2.0f,-1.f,0.f};
        //rot_acc_command = Vector3f{0.f,0.f,-0.5f};
    }
    */
    //PX4_INFO("force command: \t%.2f", (double)(f_command*f_command));
    //PX4_INFO("force command: \t%.2f\t%.2f\t%.2f", (double)f_command(0), (double)f_command(1), (double)f_command(2));
    //PX4_INFO("FRD body frame rotation vec: \t%.2f\t%.2f\t%.2f", (double)w_err(0), (double)w_err(1), (double)w_err(2));
    if (sqrtf(w_err*w_err)>M_PI_F){
        PX4_ERR("rotation angle larger than pi: \t%.2f, \t%.2f, \t%.2f", (double)sqrtf(w_err*w_err), (double)q_err.angle(), (double)(q_err.axis()*q_err.axis()));
    }

    // ====================================
    // manual attitude setpoint feedthrough
    // ====================================
    if (_switch_manual){
        // get an attitude setpoint from the current manual inputs
        float roll_ref = 1.f * _manual_control_setpoint.y * M_PI_4_F;
        float pitch_ref = -1.f* _manual_control_setpoint.x * M_PI_4_F;
        Eulerf E_current(Quatf(_attitude.q));
        float yaw_ref = E_current.psi();
        Dcmf R_ned_frd_ref(Eulerf(roll_ref, pitch_ref, yaw_ref));
        Dcmf R_enu_frd_ref(_R_ned_to_enu*R_ned_frd_ref);
        Quatf att_ref(R_enu_frd_ref);
        R_ref = Dcmf(att_ref);

        // get attitude error
        R_ref_true = Dcmf(R_ref.transpose()*R_ib);
        // get required rotation vector (in body frame)
        q_err = AxisAnglef(R_ref_true);
        // project rotation angle to [-pi,pi]
        if (q_err.angle()*q_err.angle()<M_PI_F*M_PI_F){
            w_err = -q_err.angle()*q_err.axis();
        }
        else{
            if (q_err.angle()>0.f){
                w_err = (2.f*M_PI_F-(float)fmod(q_err.angle(),2.f*M_PI_F))*q_err.axis();
            }
            else{
                w_err = (-2.f*M_PI_F-(float)fmod(q_err.angle(),2.f*M_PI_F))*q_err.axis();
            }
        }

        // compute rot acc command
        rot_acc_command = _K_q*w_err + _K_w*(Vector3f{0.f,0.f,0.f}-omega_filtered);
        
    }

    // ==============================================================
    // overwrite rudder rot_acc_command with turn coordination values
    // ==============================================================
    Vector3f vel_air = _vel - _wind_estimate;
    Vector3f vel_normalized = vel_air.normalized();
    Vector3f f = _mass*_acc;
    Vector3f f_normalized = f.normalized();
    // compute ideal angular velocity
    Vector3f omega_turn_ref_normalized = vel_normalized.cross(f_normalized);
    Vector3f omega_turn_ref;
    if (_airspeed_valid&&_airspeed>_stall_speed) {
        omega_turn_ref = sqrtf(_acc*_acc) / (_airspeed) * R_bi * omega_turn_ref_normalized.normalized();
        //PX4_INFO("yaw rate ref, yaw rate: \t%.2f\t%.2f", (double)(omega_turn_ref(2)), (double)(omega_filtered(2)));
    }
    else {
        omega_turn_ref = sqrtf(_acc*_acc) / (_stall_speed) * R_bi * omega_turn_ref_normalized.normalized();
        //PX4_ERR("No valid airspeed message detected or airspeed to low");
    }
    
    // not really a accel command, rather a FF-P command
    rot_acc_command(2) = _K_q(2,2)*omega_turn_ref(2) + _K_w(2,2)*(omega_turn_ref(2) - omega_filtered(2));

    return rot_acc_command;
}

Vector3f 
FixedwingPositionINDIControl::_compute_INDI_stage_2(Vector3f ctrl)
{
    // compute velocity in body frame
    Dcmf R_ib(_att);
    Vector3f vel_body = R_ib.transpose()*(_vel-_wind_estimate);
    float q = fmaxf(0.5f*sqrtf(vel_body*vel_body)*vel_body(0), 0.5f*_stall_speed*_stall_speed);    // dynamic pressure, saturates at stall speed
    //Vector3f vel_body_2 = Dcmf(Quatf(_attitude.q)).transpose()*Vector3f{_local_pos.vx,_local_pos.vy,_local_pos.vz};
    //PX4_INFO("ENU body frame velocity: \t%.2f\t%.2f\t%.2f", (double)vel_body_2(0), (double)vel_body_2(1), (double)vel_body_2(2));
    //PX4_INFO("FRD body frame velocity: \t%.2f\t%.2f\t%.2f", (double)vel_body(0), (double)vel_body(1), (double)vel_body(2));
    // filter omega at the same rate as the moments
    Vector3f omega_filtered;
    omega_filtered(0) = _lp_filter_omega_2[0].apply(_omega(0));
    omega_filtered(1) = _lp_filter_omega_2[1].apply(_omega(1));
    omega_filtered(2) = _lp_filter_omega_2[2].apply(_omega(2));
    // compute moments
    Vector3f moment;
    moment(0) = _k_ail*q*_actuators.control[actuator_controls_s::INDEX_ROLL] - _k_d_roll*q*omega_filtered(0);
    moment(1) = _k_ele*q*_actuators.control[actuator_controls_s::INDEX_PITCH] - _k_d_pitch*q*omega_filtered(1);
    moment(2) = _k_rud*q*_actuators.control[actuator_controls_s::INDEX_YAW] - _k_d_yaw*q*omega_filtered(2);
    // introduce artificial time delay that is also present in acceleration
    Vector3f moment_filtered;
    moment_filtered(0) = _lp_filter_delay[0].apply(moment(0));
    moment_filtered(1) = _lp_filter_delay[1].apply(moment(1));
    moment_filtered(2) = _lp_filter_delay[2].apply(moment(2)); 
    // No filter for alpha, since it is already filtered...
    Vector3f alpha_filtered = _alpha;
    Vector3f moment_command = _inertia * (ctrl - alpha_filtered) + moment_filtered;
    // perform dynamic inversion
    Vector3f deflection;
    deflection(0) = (moment_command(0) + _k_d_roll*q*omega_filtered(0))/fmaxf((_k_ail*q),0.0001f);
    deflection(1) = (moment_command(1) + _k_d_pitch*q*omega_filtered(1))/fmaxf((_k_ele*q),0.0001f);
    deflection(2) = (moment_command(2) + _k_d_yaw*q*omega_filtered(2))/fmaxf((_k_rud*q),0.0001f);

    // overwrite rudder deflection with NDI turn coordination (no INDI)
    Vector3f moment_ref = _inertia*ctrl + _omega.cross(_inertia*_omega);
    deflection(2) = (moment_ref(2) + _k_d_yaw*q*_omega(2)) / fmaxf((_k_rud*q),0.0001f);

    return deflection;
}


Vector3f
FixedwingPositionINDIControl::_compute_actuator_deflections(Vector3f ctrl)
{   
    // compute the normalized actuator deflection, including airspeed scaling
    Vector3f deflection = ctrl;

    // limit actuator deflection
    for(int i=0; i<3; i++){
        deflection(i) = constrain(deflection(i),-1.f,1.f);
    }
    /*
    // add servo slew
    float current_ail = _actuators.control[actuator_controls_s::INDEX_ROLL];
    float current_ele = _actuators.control[actuator_controls_s::INDEX_PITCH];
    float current_rud = _actuators.control[actuator_controls_s::INDEX_YAW];
    //
    float max_rate = 0.5f/0.18f;    //
    float dt = 1.f/_sample_frequency;
    //
    deflection(0) = constrain(deflection(0),current_ail-dt*max_rate,current_ail+dt*max_rate);
    deflection(1) = constrain(deflection(1),current_ele-dt*max_rate,current_ele+dt*max_rate);
    deflection(2) = constrain(deflection(2),current_rud-dt*max_rate,current_rud+dt*max_rate);
    */
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
fw_dyn_soar_control is the fixed wing controller for dynamic soaring tasks.

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
