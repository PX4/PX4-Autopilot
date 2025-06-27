
/**
 * @file indi_att_control_main.cpp
 * INDI‐based multicopter attitude controller (replacement for mc_att_control).
 *
 * @author Rohan Inamdar <rninamdar@wpi.edu>
 *
 * This module subscribes to:
 *   • vehicle_attitude       (current quaternion q)
 *   • vehicle_rates_setpoint (desired body‐rates p_des, q_des, r_des)
 *   • sensor_gyro            (raw IMU gyro, 3‐axis)
 *   • esc_status             (optional: actual motor RPM feedback)
 *
 * NOTE: Used chatgpt for improved comments to make it easier to convert between greek vars in the papers to the code
 *
 * It runs at ~400 Hz and implements the Adaptive INDI algorithm:
 *   1) Low‐pass filter raw gyro -> Ω_f
 *   2) Numerical differentiate -> Ω̇_dot_f
 *   3) (If present) filter ESC RPM -> ω_f, differentiate -> ω̇_dot_f
 *   4) Compute “desired angular acceleration” ν (mu) from tilt-prioritized quaternion law
 *   5) Solve Δω = psuedoinv(G_eff)*(ν − Ω̇_dot_f), then ω_c = ω_f + Δω
 *   6) Publish actuator_controls_0 (the 4 commanded rotor RPMs)
 *   7) Adapt G via LMS:  G = G − μ·(G·z − ΔΩ̇)·z^T …
 */

#include "indi_att_control.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/esc_status.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>
#include <px4_platform_common/px4_kmalloc.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/math.h>

using namespace matrix;

INDIController::INDIController(bool vtol) :
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
    // read parameters (mu2, mu1, filter cutoffs, etc)
    parameters_updated();
}

INDIController::~INDIController()
{
    // no dynamic allocations to free here
}

bool INDIController::init()
{
    // Subscribe to vehicle_attitude (this will drive the 400Hz Run() callback)
    if (!_vehicle_attitude_sub.registerCallback()) {
        PX4_ERR("indi_att_control: vehicle_attitude registerCallback failed");
        return false;
    }

    // Need sensor_gyro and vehicle_rates_setpoint, but we will poll() them in Run()
    _last_gyro_time = hrt_absolute_time();

    return true;
}

void INDIController::parameters_updated()
{
    // load MU2 (roll/pitch/yaw), MU1[8], and filter parameters from PX4 params
    // NOTE: These are exclusively for LMS Gain adaptation! Not used for now but implemented in futureu use
    _mu2_roll  = _param_indi_mu2_roll.get();
    _mu2_pitch = _param_indi_mu2_pitch.get();
    _mu2_yaw   = _param_indi_mu2_yaw.get();

    _mu1[0] = _param_indi_mu1_0.get();
    _mu1[1] = _param_indi_mu1_1.get();
    _mu1[2] = _param_indi_mu1_2.get();
    _mu1[3] = _param_indi_mu1_3.get();
    _mu1[4] = _param_indi_mu1_4.get();
    _mu1[5] = _param_indi_mu1_5.get();
    _mu1[6] = _param_indi_mu1_6.get();
    _mu1[7] = _param_indi_mu1_7.get();

    // configure gyro filters:
    float wn_gyro = _param_indi_gyro_wn.get();
    float z_gyro  = _param_indi_gyro_z.get();
    for (int i = 0; i < 3; i++) {
        _gyro_filter[i].setCutoffFrequency(wn_gyro, z_gyro);
    }

    // configure rpm filters:
    float wn_rpm = _param_indi_rpm_wn.get();
    float z_rpm  = _param_indi_rpm_z.get();
    for (int j = 0; j < 4; j++) {
        _rpm_filter[j].setCutoffFrequency(wn_rpm, z_rpm);
    }

    // angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

    // thrust limits
    _max_thrust = _param_mpc_thr_max.get();
    _minimum_thrust = _param_mpc_thr_min.get();

    // init G to a small diagonal guess, e.g. G = [G1·diag(ω_f) G2]. Goal is to overwrite via LMS.
    _G = matrix::Matrix<float,3,8>::Zero();

    // row 0 constants for G1 (roll)
    //     [ -b*ct, +b*ct, +b*ct, -b*ct ] / Ivx
    _G1_base(0,0) = -b*ct / Ivx;
    _G1_base(0,1) = +b*ct / Ivx;
    _G1_base(0,2) = +b*ct / Ivx;
    _G1_base(0,3) = -b*ct / Ivx;

    // row 1 constants for G1 (pitch)
    //     [ -l*ct, -l*ct, +l*ct, +l*ct ] / Ivy
    _G1_base(1,0) = -l*ct / Ivy;
    _G1_base(1,1) = -l*ct / Ivy;
    _G1_base(1,2) = +l*ct / Ivy;
    _G1_base(1,3) = +l*ct / Ivy;

    // row 2 constants for G1 (yaw)
    //     [  cq, -cq, +cq, -cq ] / Ivz
    _G1_base(2,0) = +cq   / Ivz;
    _G1_base(2,1) = -cq   / Ivz;
    _G1_base(2,2) = +cq   / Ivz;
    _G1_base(2,3) = -cq   / Ivz;

    // the rest stay small/zero until adapted

    float Ts = param_indi_inner_loop_f.get(); //Assume 400H, in seconds (0.0025 s = 2.5ms)

    float scale = Ip / (Ivz) / Ts;

    // rows 0–1 stay zero, row2 = [ +s, –s, +s, –s ]
    _G(2,4) = +scale;
    _G(2,5) = -scale;
    _G(2,6) = +scale;
    _G(2,7) = -scale;
}

void INDIController::Run()
{
    // Check for exit:
    if (should_exit()) {
        _vehicle_attitude_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }

    const hrt_abstime now = hrt_absolute_time();

    // 2) read raw gyro:
    if (_gyro_sub.copy(&gyro)) {
    //FIXME: rotate into NED
        // compute dt from raw gyro timestamp:
        float dt = (now - _last_gyro_time) * 1e-6f; // micros => seconds
        dt = math::constrain(dt, 0.001f, 0.01f);    // clamp between 1 ms and 10 ms
        _last_gyro_time = gyro.timestamp;

        matrix::Vector3f gyro_raw{gyro.x, gyro.y, gyro.z};

        // 2a) filter each axis:
        for (int i = 0; i < 3; i++) {
            _Omega_f(i) = _gyro_filter[i].apply(gyro_raw(i));
        }

        // 2b) numerical differentiate -> _Omega_dot_f
        if (_first_gyro_sample) {
            _Omega_dot_f.setZero();
            _Omega_prev = _Omega_f;
            _first_gyro_sample = false;
        } else {
            _Omega_dot_f = (_Omega_f - _Omega_prev) / dt;
            _Omega_prev  = _Omega_f;
        }
    }




    // 4) read ESC status -> actual RPM
    // FIXED!: likely need to convert from RPM -> rad/s... needs checking
    if (_esc_sub.copy(&esc)) {
        matrix::Vector4f rpm_raw{
            static_cast<float>(esc.esc_rpm[0]),
            static_cast<float>(esc.esc_rpm[1]),
            static_cast<float>(esc.esc_rpm[2]),
            static_cast<float>(esc.esc_rpm[3])
        };

        rpm_raw = rpm_raw / 60.0 * 2.0 * M_PI_F; // Convert rpm readings from esc to rad/s

        if (_first_rpm_sample) {
            _omega_f     = rpm_raw;
            _omega_dot_f.setZero();
            _omega_prev  = _omega_f;
            _first_rpm_sample = false;
        } else {
            // filter and differentiate
            for (int j = 0; j < 4; j++) {
                _omega_f(j) = _rpm_filter[j].apply(rpm_raw(j));
            }
            _omega_dot_f = (_omega_f - _omega_prev) / dt;
            _omega_prev  = _omega_f;
        }
    }


    // Check for new attitude setpoint
    if (_vehicle_attitude_setpoint_sub.updated()) {
        vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

        if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
            && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {
                //FIXME: add these global constants
            _attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
            _last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
            _q_sp = matrix::Quatf(vehicle_rates_setpoint.q_d);
        }
    }

    // Copy vehicle_attitude (this is what triggers the 400 Hz loop) //FIXME: fix Hz loop lengths
    vehicle_attitude_s v_att{};
    if (_vehicle_attitude_sub.update(&v_att)) {
        // 1) copy quat:
        _q = matrix::Quatf(v_att.q);

        _rates_sp = _attitude_control.update(_q);


        autotune_attitude_control_status_s pid_autotune;

        if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
            if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
                    || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
                    || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
                    || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
                && ((now - pid_autotune.timestamp) < 1_s)) {
                _rates_sp += Vector3f(pid_autotune.rate_sp);
            }
        }

        // publish rate setpoint
        vehicle_rates_setpoint_s rates_setpoint{};
        rates_setpoint.roll = _rates_sp(0);
        rates_setpoint.pitch = _rates_sp(1);
        rates_setpoint.yaw = _rates_sp(2);
        rates_setpoint.timestamp = hrt_absolute_time();

        _vehicle_rates_setpoint_pub.publish(rates_setpoint); //not sure if this is necceassry but could be useful for urob debugging

        // 5) compute desired angular acceleration ν (nu) (3×1) via tilt‐prioritized law:
        matrix::Vector3f nu = computeNU(_q);  // tilt‐prioritized quaternion error + rate‐PD

        // 6) Compute Δω via INDI pseudoinverse: ω_c = ω_f + Δω
        computeINIDelta(nu);


    }





    // 7) publish actuator_controls_0 (4 commanded torques)

    //FIXME: convert torque of each motor to body frame xyz torque

    actuator_sp.control[0] = _torque_c(0);
    actuator_sp.control[1] = _torque_c(1) / _max_thrust;
    actuator_sp.control[2] = _torque_c(2) / _max_thrust;
    actuator_sp.control[3] = _torque_c(3) / _max_thrust;
    actuator_sp.timestamp = hrt_absolute_time();
    _actuator_pub.publish(actuator_sp);

    // 8) update LMS adaptation: refine G using the new ΔΩ̇ error
    // TODO: Write tuner to find adpative gains
    //updateAdaptiveLMS();

    // 9) loop back: WorkItem will schedule the next Run() automatically
}

vector::Vector3f INDIController::computeNU(matrix::Quatf &curr_q)
{
    // 1) Build full quaternion‐error: q_e = q_sp x q_current^-1
    matrix::Quaternion q_e  = _q_sp * curr_q.inversed();

    // 2) Tilt‐prioritized split into “reduced‐attitude” and yaw:
    //    reduced‐attitude error (qe_red) only x,y part, yaw error (qe_yaw) only z
    float q_e_w = q_e(0);
    float q_e_x = q_e(1);
    float q_e_y = q_e(2);
    float q_e_z = q_e(3);

    float denom = sqrtf(q_e_w*q_e_w + q_e_z*q_e_z);
    Vector3f qe_red {
        (q_e_w*q_e_x - q_e_y*q_e_z) / denom,
        (q_e_w*q_e_y + q_e_x*q_e_z) / denom,
         0.0f
    };
    Vector3f qe_yaw{ 0.0f, 0.0f, (q_e_z / denom) };

    // 3) PD gains from parameters:
    float k_red = _param_indi_kq_red.get();
    float k_yaw = _param_indi_kq_yaw.get();

    // 4) Rate‐feedback to damp:
    Vector3f rate_err = _rates_sp - _Omega_f;                    // want zero body‐rates
    rate_err(0) *= _param_indi_kp_roll.get();
    rate_err(1) *= _param_indi_kp_pitch.get();
    rate_err(2) *= _param_indi_kp_yaw.get();

    // 5) combine into ν:
    //TODO: add feedforward jerk/snap from NMPC, and accel feed forward
    Vector3f nu = qe_red * k_red
                 + qe_yaw * (k_yaw * (q_e_w >= 0 ? +1.0f : -1.0f))
                 + rate_err;
    return nu;
}

void INDIController::computeINIDelta(const vector::Vector3f &nu)
{
    // Build G_eff = [G1·diag(ω_f)  +  G2] using current _omega_f. Then compute pseudo-inverse.
    //
    // G1·diag(ω_f) is 3×4.  G2 is also 3×4 (the “G2 columns” are stored in columns 4..7 of _G)
    // => G_eff = 3×4 =  G1·diag(ω_f)  +  _G.slice<3,4>(0,4)

    // Build G1·diag(ω_f):
    //    G1 rows:
    //      [  -b*ct , +b*ct , +b*ct , -b*ct  ] / Ivx
    //      [  -l*ct , -l*ct , +l*ct , +l*ct  ] / Ivy
    //      [   cq  ,  -cq  ,  cq  ,  -cq   ] / Ivz
    // NOTE: here g1 is multiply each *by* the current ω_f(i) => effectively G1·diag(ω_f):
    for (int i = 0; i < 4; i++) {
    _G1_diag.col(i) = _G1_base.col(i) * _omega_f(i);
    }

    // get G2 part from columns 4..7 of _G:
    matrix::Matrix<float,3,4> G2_part;
    G2_part(0,0) = _G(0,4);  G2_part(0,1) = _G(0,5);
    G2_part(0,2) = _G(0,6);  G2_part(0,3) = _G(0,7);
    G2_part(1,0) = _G(1,4);  G2_part(1,1) = _G(1,5);
    G2_part(1,2) = _G(1,6);  G2_part(1,3) = _G(1,7);
    G2_part(2,0) = _G(2,4);  G2_part(2,1) = _G(2,5);
    G2_part(2,2) = _G(2,6);  G2_part(2,3) = _G(2,7);

    // G_eff = G1_diag + G2_part
    matrix::Matrix<float,3,4> G_eff = _G1_diag;
    G_eff += G2_part;

    // compute pinv: G_eff⁺ (4×3) = (GᵀG)⁻¹ Gᵀ
    matrix::Matrix<float,4,3> G_eff_pinv;
    {
        //TODO: consider ridge regulaization on GTG before inversion
        matrix::Matrix<float,4,3> Gt = G_eff.transposed();
        matrix::SquareMatrix<float,4> GtG = Gt * G_eff;   // 4×4
        matrix::SquareMatrix<float,4> GtG_inv = GtG.inversed(); // invert 4×4
        G_eff_pinv = GtG_inv * Gt;  // result is 4×3
    }

    matrix::Vector4f delta_omega = _prev_omega_c - _omega_f;

    // Δ = G_eff⁺ * (ν – Ω̇_dot_f + G2 * prev_delta_omega)
    matrix::Vector3f diff = (nu - _Omega_dot_f + G2_part * _prev_delta_omega);
    matrix::Vector4f delta = G_eff_pinv * diff;

    // new commanded RPM = ω_f + Δ
    _omega_c = _omega_f + delta;

    // saturate/clamp each between [torque min, max]
    //FIXME: find toque min max
    for (int i = 0; i < 4; i++) {
        _omega_c(i) = math::constrain(_omega_c(i), rpm_min, rpm_max);\
        _torque_c(i) = ct * (_omega_c(i) ** 2);
    }

    _prev_delta_omega = delta_omega;
    _prev_omega_c = _omega_c;
}

//TODO: see if this works and tune the learning gains
void INDIController::updateAdaptiveLMS()
{
    // z(k) = [ Δω_f  ;  Δω̇_dot_f ]  an 8×1 vector
    matrix::Vector<float,8> z;
    for (int i = 0; i < 4; i++) {
        z(i)   = _omega_f(i)   - _omega_prev(i);      // Δω_f
        z(i+4) = _omega_dot_f(i) - _omega_dot_prev(i);// Δω̇__dot_f
    }

    // error e = G * z  –  ΔΩ̇_f  (3×1)
    matrix::Vector3f dOmega_dot = _Omega_dot_f - _Omega_dot_prev;
    matrix::Vector3f e = (_G * z) - dOmega_dot;

    // Update each row of G: G(i,:) <- G(i,:)  –  μ2(i) * e(i) * z^T * μ1
    //FIXME: When this is implemented make m1 all the same var. Mu1 should NOT change per axis (aka change mu1[j] to just mu1)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 8; j++) {
            float incr = 0.f;
            if (i == 0) {  // roll row
                incr = _mu2_roll * e(0) * z(j) * _mu1[j];
            }
            if (i == 1) {  // pitch row
                incr = _mu2_pitch * e(1) * z(j) * _mu1[j];
            }
            if (i == 2) {  // yaw row
                incr = _mu2_yaw * e(2) * z(j) * _mu1[j];
            }
            _G(i,j) -= incr;
        }
    }

    // Store “previous”:
    _Omega_dot_prev = _Omega_dot_f;
    _omega_prev     = _omega_f;
    _omega_dot_prev = _omega_dot_f;
}

//----------------------------------------------------------------------
// Module boilerplate: task_spawn, custom_command, print_usage, main()
//----------------------------------------------------------------------

int INDIController::task_spawn(int argc, char *argv[])
{
    // ignore “vtol” argument here, basically a direct copy of mc_att_control_main.cpp
    INDIController *instance = new INDIController(false);

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int INDIController::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int INDIController::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This module replaces mc_att_control with an Adaptive INDI attitude controller.
It subscribes to vehicle_attitude, vehicle_rates_setpoint, sensor_gyro, and esc_status,
then publishes actuator_controls_0 (4 x motor RPM) at ~400 Hz.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("indi_att_control", " \n\t\tRun the INDI-based attitude loop.");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

/**
 * INDI attitude control app start / stop handling function
 */
extern "C" __EXPORT int indi_att_control_main(int argc, char *argv[])
{
    return INDIController::main(argc, argv);
}
