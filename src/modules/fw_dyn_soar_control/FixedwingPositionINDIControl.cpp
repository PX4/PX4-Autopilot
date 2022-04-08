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
	_alpha_sp_pub(ORB_ID(vehicle_angular_acceleration_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// limit to 50 Hz
	_local_pos_sub.set_interval_ms(20);

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionINDIControl::~FixedwingPositionINDIControl()
{
	perf_free(_loop_perf);
}

void
FixedwingPositionINDIControl::_set_wind_estimate(Vector3f wind)
{
    _wind_estimate = wind;
    return;
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_basis_funs(float t)
{
    Vector<float, _num_basis_funs> vec;
    vec(0) = 1;
    float sigma = 0.5/_num_basis_funs;
    for(uint i=1; i<_num_basis_funs; i++){
        float fun1 = sinf(M_PI_F*t);
        float fun2 = exp(-powf((t-i/_num_basis_funs),2)/sigma);
        vec(i) = fun1*fun2;
    }
    return vec;
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_d_dt_basis_funs(float t)
{
    Vector<float, _num_basis_funs> vec;
    vec(0) = 1;
    float sigma = 0.5/_num_basis_funs;
    for(uint i=1; i<_num_basis_funs; i++){
        float fun1 = sinf(M_PI_F*t);
        float fun2 = exp(-powf((t-i/_num_basis_funs),2)/sigma);
        vec(i) = fun2*(M_PI_F*sigma*cosf(M_PI_F*t)-2*(t-i/_num_basis_funs)*fun1)/sigma;
    }
    return vec;
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_d2_dt2_basis_funs(float t)
{
    Vector<float, _num_basis_funs> vec;
    vec(0) = 1;
    float sigma = 0.5/_num_basis_funs;
    for(uint i=1; i<_num_basis_funs; i++){
        float fun1 = sinf(M_PI_F*t);
        float fun2 = exp(-powf((t-i/_num_basis_funs),2)/sigma);
        vec(i) = fun2 * (fun1 * (4*powf((i/_num_basis_funs-t),2) - \
                        sigma*(powf(M_PI_F,2)*sigma + 2)) + 4*M_PI_F*sigma*(i/_num_basis_funs-t)*cosf(M_PI_F*t))/(powf(sigma,2));
 
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
    Vector3f f = _param_fw_mass.get()*acc;
    // compute force component projected onto lift axis
    Vector3f vel_normalized = vel_air.normalized();
    Vector3f f_lift = f - f*vel_normalized;
    Vector3f lift_normalized = f_lift.normalized();
    Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
    // compute rotation matrix
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
    // compute required AoA
    Vector3f f_phi = R_bi*f_lift;
    float AoA = ((2.f*f_phi(2))/(_param_rho.get()*_param_fw_wing_area.get()*(vel_air*vel_air)) - _param_fw_c_l0.get())/_param_fw_c_l1.get();
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
    const uint n = 100;
    Vector<float, n> distances;
    // compute all distances
    for(uint i=0; i<n; i++){
        float t_ref = float(i)/n;
        Vector3f pos_ref = _get_position_ref(t_ref);
        distances(i) = (pos_ref - pos)*(pos_ref - pos);
    }
    // get index of smallest distance
    float t = 0;
    float min_dist = distances(0);
    for(uint i=1; i<n; i++){
        if(distances(i)<min_dist){
            min_dist = distances(i);
            t = float(i)/n;
        }
    }
    return t;
}

Quatf
FixedwingPositionINDIControl::_get_attitude(Vector3f vel, Vector3f f)
{
    Vector3f vel_air = vel - _wind_estimate;
    // compute force component projected onto lift axis
    Vector3f vel_normalized = vel_air.normalized();
    Vector3f f_lift = f - (f*vel_normalized)*vel_normalized;
    Vector3f lift_normalized = f_lift.normalized();
    Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
    // compute rotation matrix
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
    // compute required AoA
    Vector3f f_phi = R_bi*f_lift;
    float AoA = ((2.f*f_phi(2))/(_param_rho.get()*_param_fw_wing_area.get()*(vel_air*vel_air)) - _param_fw_c_l0.get())/_param_fw_c_l1.get();
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

void
FixedwingPositionINDIControl::_compute_NDI_control_input(Vector3f pos, Vector3f vel, Vector3f acc, Quatf att, Vector3f omega, Vector3f alpha)
{
    Dcmf R_ib(att);
    Dcmf R_bi(R_ib.transpose());
    // get reference values
    float t_ref = _get_closest_t(pos);
    // downscale velocity to match current one, 
    // terminal time is determined such that current velocity is met
    Vector3f v_ref_ = _get_velocity_ref(t_ref, 1.f);
    float T = sqrt((v_ref_*v_ref_)/(vel*vel+0.01f));
    // get time-scaled version of reference trajectory
    Vector3f x_ref = _get_position_ref(t_ref);
    Vector3f v_ref = _get_velocity_ref(t_ref,T);
    Vector3f a_ref = _get_acceleration_ref(t_ref,T);
    Vector3f omega_ref = _get_angular_velocity_ref(t_ref,T);
    Vector3f alpha_ref = _get_angular_acceleration_ref(t_ref,T);
    // get acceleration command in world frame (without gravity)
    Vector3f acc_command = R_ib*(_K_x*R_bi*(x_ref-pos) + _K_v*R_bi*(v_ref-vel) + _K_a*R_bi*(a_ref-acc)) + a_ref;
    // add gravity
    acc_command(2) += 9.81f;
    // get force comand in world frame
    Vector3f f_command = _param_fw_mass.get()*acc_command;
    // get required attitude (assuming we can fly the target velocity)
    Dcmf R_ref(_get_attitude(v_ref,f_command));
    // get attitude error
    Dcmf R_ref_true(R_ref.transpose()*R_ib);
    // get required rotation vector (in body frame)
    AxisAnglef q_err(R_ref_true);
    Vector3f w_err = -q_err.angle()*q_err.axis();
    //Vector3f v_norm = (vel-_wind_estimate).unit();
    //Vector3f v_ref_norm = (v_ref-_wind_estimate).unit();
    // compute angular acceleration command (in body frame)
    Vector3f rot_acc_command = _K_q*w_err + _K_w*(omega_ref-omega) + alpha_ref;
    // publish control input
    _angular_accel_sp = {}; 
    _angular_accel_sp.timestamp = hrt_absolute_time();
    _angular_accel_sp.xyz[0] = rot_acc_command(0);
    _angular_accel_sp.xyz[1] = rot_acc_command(1);
    _angular_accel_sp.xyz[2] = rot_acc_command(2);
    _alpha_sp_pub.publish(_angular_accel_sp);
    return;
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
