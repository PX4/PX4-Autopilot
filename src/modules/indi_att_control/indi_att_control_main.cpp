
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

    // init G to a small diagonal guess, e.g. G = [G1·diag(ω_f) G2]. Goal is to overwrite via LMS.
    _G = matrix::Matrix<float,3,8>::Zero();

    const float m = 0.75f;          // quad mass [kg]
    const float ct = 1.0f;          // thrust coefficient (from your param)
    const float cq = 2.37e-8f;       // torque coeff
    const float l  = 0.14f;         // arm length [m]
    const float beta = 56.0f * (M_PI_F/180.0f);  // geometry angle
    const float Ip = 0.002f;        // rotor inertia [kg·m²]
    const float Ivx = 0.0025f, Ivy = 0.0021f, Ivz = 0.0043f; // vehicle inertia

    // row 0 constants for G1 (roll)
    //     [ -l*ct, +l*ct, +l*ct, -l*ct ] / Ivx
    _G1_base(0,0) = -l*ct / Ivx;
    _G1_base(0,1) = +l*ct / Ivx;
    _G1_base(0,2) = +l*ct / Ivx;
    _G1_base(0,3) = -l*ct / Ivx;

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

    //NOTE: this is NOT a completed G2! Still need to divide by Ts (the inner loop speed) which I pass when I call
    // the INDI function as dt.
    const float Ip  = 0.0020f;     // rotor inertia
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

    // Copy vehicle_attitude (this is what triggers the 400 Hz loop)
    vehicle_attitude_s v_att{};
    if (!_vehicle_attitude_sub.update(&v_att)) {
        return; // no new attitude -> skip
    }

    // compute dt from raw gyro timestamp:
    hrt_abstime now = hrt_absolute_time();
    float dt = (now - _last_gyro_time) * 1e-6f; // seconds
    dt = math::constrain(dt, 0.001f, 0.01f);    // clamp between 1 ms and 10 ms
    _last_gyro_time = now;

    // 1) copy quat:
    _q = matrix::Quatf(v_att.q);

    // 2) read raw gyro:
    if (_gyro_sub.copy(&gyro)) {
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

    // 3) read desired body‐rates:
    vehicle_rates_setpoint_s rates_sp{};
    if (_vehicle_rates_sub.copy(&rates_sp)) {
        _p_des = rates_sp.roll;
        _q_des = rates_sp.pitch;
        _r_des = rates_sp.yaw;
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

        rpm_raw = rpm_raw / 60 * 2 * PI; // Convert rpm readings from esc to rad/s

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

    // 5) compute desired angular acceleration ν (nu) (3×1) via tilt‐prioritized law:
    vector::Vector3f nu = computeNU();  // tilt‐prioritized quaternion error + rate‐PD

    // 6) Compute Δω via INDI pseudoinverse: ω_c = ω_f + Δω
    computeINIDelta(nu, dt);

    // 7) publish actuator_controls_0 (4 commanded RPMs)
    float max_rpm = _param_indi_max_rpm.get();
    actuator_sp.control[0] = _omega_c(0) * (60 / 2 / PI) / max_rpm; // Converts rad/s -> rpm -> fraction of max speed for actuator publisher
    actuator_sp.control[1] = _omega_c(1)* (60 / 2 / PI) / max_rpm; // Converts rad/s -> rpm -> fraction of max speed for actuator publisher
    actuator_sp.control[2] = _omega_c(2)* (60 / 2 / PI) / max_rpm; // Converts rad/s -> rpm -> fraction of max speed for actuator publisher
    actuator_sp.control[3] = _omega_c(3)* (60 / 2 / PI) / max_rpm; // Converts rad/s -> rpm -> fraction of max speed for actuator publisher
    actuator_sp.timestamp = hrt_absolute_time();
    _actuator_pub.publish(actuator_sp);

    // 8) update LMS adaptation: refine G using the new ΔΩ̇ error
    // TODO: Write tuner to find adpative gains
    //updateAdaptiveLMS();

    // 9) loop back: WorkItem will schedule the next Run() automatically
}

vector::Vector3f INDIController::computeNU()
{
    // 1) Build quaternion error q_e = q_des * q ^ -1
    //    NOTE: here “q_des” is indirectly encoded by (p_des,q_des,r_des) -> we do tilt‐prioritized directly.
    //    From other papers, normally they take the current quaternion _q, compute “desired tilt quaternion”
    //    from (p_des,q_des) as a small AxisAnglef( roll_sp, pitch_sp, 0 ), then combine with yaw_sp
    //
    // For simplicity, I assume _p_des/_q_des/_r_des already come from a higher‐level “tilt‐prioritized”
    // attitude setpoint (from NMPC)
    // FIXME: if not then reconstruct q_sp from (p_des,q_des,r_des) here

    // *** INDI paper uses feedforward: nu = [p_dot_des + k1*(p_des - p) ;  q_Δω̇__dot_fdot_des + k2*(q_des - q) ;  r_dot_des + k3*(r_des - r) ]
    // but in this “simple” template, I assume no feed‐forward p_dot_des, r_dot_des (or set them to zero)
    // FIXME: See if feedfoward is actually necessary

    vector::Vector3f omega_f_body = _Omega_f; // (p, q, r)
    vector::Vector3f omega_err;
    omega_err(0) = _p_des - omega_f_body(0);
    omega_err(1) = _q_des - omega_f_body(1);
    omega_err(2) = _r_des - omega_f_body(2);

    // FIXME: tune gains in QGroundControl
    float k1 = _params_indi_kp_roll.get();
    float k2 = _params_indi_kp_pitch.get();
    float k3 = _params_indi_kp_yaw.get();

    vector::Vector3f rate_term;
    rate_term(0) = k1 * omega_err(0);
    rate_term(1) = k2 * omega_err(1);
    rate_term(2) = k3 * omega_err(2);

    // FIXME: No explicit feed‐forward p_dot_des; if there is jerk/snap from NMPC add it here. (KIND OF NECESSARY!)

    vector::Vector3f nu = rate_term;
    return nu;
}

void INDIController::computeINIDelta(const vector::Vector3f &nu, const float dt)
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

    // Δ = G_eff⁺ * (ν – Ω̇_dot_f)
    vector::Vector3f diff = (nu - _Omega_dot_f);
    vector::Vector4f delta = G_eff_pinv * diff;

    // new commanded RPM = ω_f + Δ
    _omega_c = _omega_f + delta;

    // saturate/clamp each between [rpm_min, rpm_max]
    const float rpm_min = 0.0f, rpm_max = _param_indi_max_rpm.get() * 60 / 2/ PI; // clamp and convert to rad / s from max rpm
    for (int i = 0; i < 4; i++) {
        _omega_c(i) = math::constrain(_omega_c(i), rpm_min, rpm_max);
    }
}

//TODO: see if this works and tune the learning gains
void INDIController::updateAdaptiveLMS()
{
    // z(k) = [ Δω_f  ;  Δω̇_dot_f ]  an 8×1 vector
    vector::Vector<float,8> z;
    for (int i = 0; i < 4; i++) {
        z(i)   = _omega_f(i)   - _omega_prev(i);      // Δω_f
        z(i+4) = _omega_dot_f(i) - _omega_dot_prev(i);// Δω̇__dot_f
    }

    // error e = G * z  –  ΔΩ̇_f  (3×1)
    vector::Vector3f dOmega_dot = _Omega_dot_f - _Omega_dot_prev;
    vector::Vector3f e = (_G * z) - dOmega_dot;

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
